#!/usr/bin/env python3
"""
arctos_can_control_panel.py

Small desktop app with two buttons for the CANable bring-up/shutdown
routine documented in arctos_can_sop.tex:

  "Set Up CAN"                    -- binds the CANable adapter to can0 at
                                      500 kbit/s (slcand -s6, matching the
                                      corrected scripts/setup_canable.sh),
                                      after it has been plugged into the
                                      VirtualBox VM's USB.
  "Disable Motor && Safe to Unplug" -- scans the bus for any responding
                                      joint, sends it an explicit disable
                                      (0xF3 0x00), then brings can0 down and
                                      stops slcand so the adapter can be
                                      safely unplugged.

Bringing the interface up/down needs root; this app requests that via
pkexec (a graphical password prompt) rather than embedding sudo, so it
never needs to run with elevated privileges itself. Reading/writing CAN
frames once can0 is up does not need root and runs as the normal user,
matching how every script in arctos_can_control does it.

Run directly:
    python3 arctos_can_control_panel.py
or launch via the "Arctos CAN Control Panel" entry in the application menu
(see install_desktop_entry.sh in this same folder).
"""
import glob
import os
import subprocess
import threading
import time
import tkinter as tk
from tkinter import scrolledtext

try:
    import can
except ImportError:
    can = None

IFACE = "can0"
BITRATE_SLCAND_CODE = "s6"  # 500 kbit/s, per arctos_can_sop.tex
SCAN_ID_RANGE = range(1, 21)
READ_ENCODER = 0x31
ENABLE_MOTOR = 0xF3


def checksum(motor_id, data_bytes):
    return (motor_id + sum(data_bytes)) & 0xFF


def find_canable_device():
    by_id_dir = "/dev/serial/by-id"
    if os.path.isdir(by_id_dir):
        for name in os.listdir(by_id_dir):
            if "canable" in name.lower():
                return os.path.join(by_id_dir, name)
    candidates = sorted(glob.glob("/dev/ttyACM*"))
    return candidates[0] if candidates else None


def can0_is_up():
    try:
        out = subprocess.run(["ip", "link", "show", IFACE],
                              capture_output=True, text=True, timeout=5)
        return out.returncode == 0 and "UP" in out.stdout
    except Exception:
        return False


def run_pkexec(shell_command, log):
    """Runs one shell command as root via a graphical pkexec prompt."""
    log(f"$ pkexec bash -c \"{shell_command}\"")
    result = subprocess.run(["pkexec", "bash", "-c", shell_command],
                             capture_output=True, text=True)
    if result.stdout.strip():
        log(result.stdout.strip())
    if result.stderr.strip():
        log(result.stderr.strip())
    return result.returncode == 0


def setup_can(log):
    log("=== Set Up CAN ===")
    device = find_canable_device()
    if device is None:
        log("ERROR: No CANable device found. Is it plugged in and attached to this VM in "
            "VirtualBox's USB device menu (not just the host)?")
        return False
    log(f"Found CANable device: {device}")

    cmd = (
        f"pkill slcand 2>/dev/null; "
        f"ip link delete {IFACE} 2>/dev/null; "
        f"sleep 0.3; "
        f"slcand -o -c -{BITRATE_SLCAND_CODE} '{device}' {IFACE} && "
        f"sleep 0.3 && "
        f"ip link set {IFACE} txqueuelen 1000 && "
        f"ip link set up {IFACE}"
    )
    ok = run_pkexec(cmd, log)
    time.sleep(0.5)

    if can0_is_up():
        log(f"SUCCESS: {IFACE} is up at 500 kbit/s. Ready to talk to a joint.")
        return True
    log(f"FAILED: {IFACE} did not come up. Check the log above for the actual error.")
    return False


def scan_and_disable(log):
    """Read-only scan for responders, then explicit disable for each one found.
    Returns the list of CAN IDs that were found and disabled."""
    if can is None:
        log("ERROR: python-can is not installed for this Python interpreter.")
        return []

    found = []
    bus = can.interface.Bus(channel=IFACE, interface="socketcan")
    try:
        log(f"Scanning CAN IDs {SCAN_ID_RANGE.start}-{SCAN_ID_RANGE.stop - 1} for responders...")
        for motor_id in SCAN_ID_RANGE:
            while bus.recv(timeout=0.0) is not None:
                pass
            data = [READ_ENCODER]
            crc = checksum(motor_id, data)
            bus.send(can.Message(arbitration_id=motor_id, data=data + [crc], is_extended_id=False))
            end = time.time() + 0.3
            responded = False
            while time.time() < end:
                resp = bus.recv(timeout=max(0.0, end - time.time()))
                if resp and resp.arbitration_id == motor_id and len(resp.data) >= 2 and resp.data[0] == READ_ENCODER:
                    responded = True
                    break
            if responded:
                log(f"  0x{motor_id:02X}: responded")
                found.append(motor_id)

        if not found:
            log("No joints responded -- nothing to disable.")
            return []

        for motor_id in found:
            data = [ENABLE_MOTOR, 0x00]
            crc = checksum(motor_id, data)
            bus.send(can.Message(arbitration_id=motor_id, data=data + [crc], is_extended_id=False))
            log(f"  Sent DISABLE to 0x{motor_id:02X}")
        time.sleep(0.3)
        return found
    finally:
        bus.shutdown()


def disable_and_make_safe(log):
    log("=== Disable Motor && Safe to Unplug ===")
    if not can0_is_up():
        log(f"{IFACE} is not up -- nothing to disable. Already safe to unplug.")
        return True

    try:
        disabled = scan_and_disable(log)
    except Exception as e:
        log(f"ERROR while scanning/disabling over CAN: {e}")
        log("Proceeding to bring the interface down anyway.")
        disabled = None

    if disabled:
        log(f"Disabled {len(disabled)} joint(s): {[hex(i) for i in disabled]}")

    cmd = f"ip link set {IFACE} down; pkill slcand 2>/dev/null"
    run_pkexec(cmd, log)
    time.sleep(0.3)

    if not can0_is_up():
        log("SUCCESS: interface is down. Safe to unplug the CANable adapter now.")
        return True
    log(f"WARNING: {IFACE} still appears up. Do not unplug yet -- check the log above.")
    return False


class App:
    def __init__(self, root):
        self.root = root
        root.title("Arctos CAN Control Panel")
        root.geometry("640x420")

        header = tk.Label(root, text="Arctos CAN Control Panel", font=("sans-serif", 14, "bold"))
        header.pack(pady=(12, 4))

        subtitle = tk.Label(root, text=f"Interface: {IFACE}    |    slcand bitrate: -{BITRATE_SLCAND_CODE} (500 kbit/s)",
                             fg="#555555")
        subtitle.pack(pady=(0, 10))

        button_frame = tk.Frame(root)
        button_frame.pack(pady=6)

        self.setup_btn = tk.Button(button_frame, text="Set Up CAN", width=24, height=2,
                                    bg="#2e7d32", fg="white", font=("sans-serif", 11, "bold"),
                                    command=self.on_setup)
        self.setup_btn.grid(row=0, column=0, padx=8)

        self.disable_btn = tk.Button(button_frame, text="Disable Motor &&\nSafe to Unplug",
                                      width=24, height=2, bg="#c62828", fg="white",
                                      font=("sans-serif", 11, "bold"), command=self.on_disable)
        self.disable_btn.grid(row=0, column=1, padx=8)

        self.status_var = tk.StringVar(value="Ready.")
        status_label = tk.Label(root, textvariable=self.status_var, fg="#333333")
        status_label.pack(pady=(6, 0))

        self.log_widget = scrolledtext.ScrolledText(root, height=16, font=("monospace", 9))
        self.log_widget.pack(fill="both", expand=True, padx=10, pady=10)
        self.log_widget.configure(state="disabled")

        if can is None:
            self.log("WARNING: python-can is not importable in this Python environment -- "
                      "the disable button's CAN scan will fail until that is fixed.")

    def log(self, message):
        def _append():
            self.log_widget.configure(state="normal")
            self.log_widget.insert("end", message + "\n")
            self.log_widget.see("end")
            self.log_widget.configure(state="disabled")
        self.root.after(0, _append)

    def set_buttons_enabled(self, enabled):
        state = "normal" if enabled else "disabled"
        self.setup_btn.configure(state=state)
        self.disable_btn.configure(state=state)

    def set_status(self, text):
        self.root.after(0, lambda: self.status_var.set(text))

    def run_in_background(self, target, busy_status):
        self.set_buttons_enabled(False)
        self.set_status(busy_status)

        def worker():
            try:
                target(self.log)
            except Exception as e:
                self.log(f"UNEXPECTED ERROR: {e}")
            finally:
                self.set_status("Ready.")
                self.root.after(0, lambda: self.set_buttons_enabled(True))

        threading.Thread(target=worker, daemon=True).start()

    def on_setup(self):
        self.run_in_background(setup_can, "Setting up CAN (a password prompt may appear)...")

    def on_disable(self):
        self.run_in_background(disable_and_make_safe, "Disabling motor and shutting down CAN...")


def main():
    root = tk.Tk()
    App(root)
    root.mainloop()


if __name__ == "__main__":
    main()
