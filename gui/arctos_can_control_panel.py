#!/usr/bin/env python3
"""
arctos_can_control_panel.py

Small desktop app with four buttons for the CANable bring-up/shutdown
routine documented in arctos_can_sop.tex:

  "Set Up CAN"      -- binds the CANable adapter to can0 at 500 kbit/s
                        (slcand -s6, matching the corrected
                        scripts/setup_canable.sh), after it has been plugged
                        into the VirtualBox VM's USB.
  "Enable Motor"    -- scans the bus for any responding joint and sends it
                        an explicit enable (0xF3 0x01). The joint starts
                        holding position as soon as this lands, so it can
                        move if it was left off-target -- keep clear of the
                        arm before using it.
  "Disable Motor"   -- scans the bus for any responding joint and sends it
                        an explicit disable (0xF3 0x00). Does not touch the
                        CAN interface itself.
  "Shut Down CAN"   -- brings can0 down and stops slcand, so the adapter can
                        be safely unplugged. Does not touch the motor --
                        run "Disable Motor" first if a joint might still be
                        enabled.

These are deliberately separate (not combined into one "make everything
safe" action) so each can be used independently -- e.g. disabling a motor
mid-session without tearing down the CAN link, or bringing the link down
when nothing was ever enabled.

Bringing the interface up/down needs root; this app requests that via
pkexec (a graphical password prompt) rather than embedding sudo, so it
never needs to run with elevated privileges itself. Reading/writing CAN
frames once can0 is up does not need root and runs as the normal user,
matching how every script in arctos_can_control does it.

The background/leaf artwork is generated entirely by generate_background.py
in this same folder (no third-party stock image), so there is no
redistribution-licensing question in shipping gui/assets/floral_background.png
in this public repo.

Run directly:
    python3 arctos_can_control_panel.py
or launch via the "CAN Control Panel" entry in the application menu
(see install_desktop_entry.sh in this same folder).
"""
import glob
import math
import os
import subprocess
import threading
import time
import tkinter as tk
import webbrowser

try:
    import can
except ImportError:
    can = None

try:
    from PIL import Image, ImageOps, ImageTk
except ImportError:
    Image = None
    ImageOps = None
    ImageTk = None

IFACE = "can0"
BITRATE_SLCAND_CODE = "s6"  # 500 kbit/s, per arctos_can_sop.tex
SCAN_ID_RANGE = range(1, 21)
READ_ENCODER = 0x31
ENABLE_MOTOR = 0xF3

SUPPORT_EMAIL = "irisdabun@gmail.com"

GUI_DIR = os.path.dirname(os.path.abspath(__file__))
BACKGROUND_PATH = os.path.join(GUI_DIR, "assets", "floral_background.png")

CANVAS_WIDTH, CANVAS_HEIGHT = 560, 420

# Sampled directly from the leaves in assets/floral_background.png (average
# of green-dominant pixels), so the buttons match the actual photo rather
# than a guessed green.
LEAF_GREEN = "#6e8645"
LEAF_GREEN_DARK = "#42502a"
LEAF_GREEN_LIGHT = "#a1b086"
LEAF_GREEN_DISABLED = "#b7bfae"
STEM_COLOR = "#4a3f2a"
TEXT_CREAM = "#faf7ee"
TITLE_BROWN = "#4a3f2a"
BEIGE_BG = "#f5f2e8"


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


def scan_for_responders(bus, log):
    """Read-only scan of SCAN_ID_RANGE. Returns the CAN IDs that answered."""
    found = []
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
    return found


def scan_and_set_enable(log, enable):
    """Read-only scan for responders, then an explicit enable (0xF3 0x01) or
    disable (0xF3 0x00) for each one found. Returns the list of CAN IDs that
    were found and commanded."""
    if can is None:
        log("ERROR: python-can is not installed for this Python interpreter.")
        return []

    word = "ENABLE" if enable else "DISABLE"
    bus = can.interface.Bus(channel=IFACE, interface="socketcan")
    try:
        found = scan_for_responders(bus, log)
        if not found:
            log(f"No joints responded -- nothing to {word.lower()}.")
            return []

        for motor_id in found:
            data = [ENABLE_MOTOR, 0x01 if enable else 0x00]
            crc = checksum(motor_id, data)
            bus.send(can.Message(arbitration_id=motor_id, data=data + [crc], is_extended_id=False))
            log(f"  Sent {word} to 0x{motor_id:02X}")
        time.sleep(0.3)
        return found
    finally:
        bus.shutdown()


def enable_motor(log):
    log("=== Enable Motor ===")
    log("CAUTION: an enabled joint holds position under power and can move. "
        "Keep clear of the arm.")
    if not can0_is_up():
        log(f"{IFACE} is not up -- run \"Set Up CAN\" first.")
        return False

    try:
        enabled = scan_and_set_enable(log, enable=True)
    except Exception as e:
        log(f"ERROR while scanning/enabling over CAN: {e}")
        return False

    if enabled:
        log(f"Enabled {len(enabled)} joint(s): {[hex(i) for i in enabled]}")
    return True


def disable_motor(log):
    log("=== Disable Motor ===")
    if not can0_is_up():
        log(f"{IFACE} is not up -- nothing to scan/disable.")
        return True

    try:
        disabled = scan_and_set_enable(log, enable=False)
    except Exception as e:
        log(f"ERROR while scanning/disabling over CAN: {e}")
        return False

    if disabled:
        log(f"Disabled {len(disabled)} joint(s): {[hex(i) for i in disabled]}")
    return True


def shutdown_can(log):
    log("=== Shut Down CAN ===")
    if not can0_is_up():
        log(f"{IFACE} is already down. Safe to unplug the CANable adapter.")
        return True

    cmd = f"ip link set {IFACE} down; pkill slcand 2>/dev/null"
    run_pkexec(cmd, log)
    time.sleep(0.3)

    if not can0_is_up():
        log("SUCCESS: interface is down. Safe to unplug the CANable adapter now.")
        return True
    log(f"WARNING: {IFACE} still appears up. Do not unplug yet -- check the log above.")
    return False


def open_support_email(log):
    subject = "CAN Control Panel - Support Request"
    url = f"mailto:{SUPPORT_EMAIL}?subject={subject.replace(' ', '%20')}"
    log(f"Opening default email client to {SUPPORT_EMAIL}...")
    webbrowser.open(url)
    return True


def rounded_rect_points(x1, y1, x2, y2, r):
    r = min(r, (x2 - x1) / 2, (y2 - y1) / 2)
    return [
        x1 + r, y1,
        x2 - r, y1,
        x2, y1,
        x2, y1 + r,
        x2, y2 - r,
        x2, y2,
        x2 - r, y2,
        x1 + r, y2,
        x1, y2,
        x1, y2 - r,
        x1, y1 + r,
        x1, y1,
    ]


def leaf_tip_points(cx, cy, length, angle_deg):
    """The two pointed ends of a leaf_polygon at this angle, plus the axis
    perpendicular to the tip-to-tip line -- used to place the vein and stem
    consistently at any rotation."""
    rad = math.radians(angle_deg)
    sin_a, cos_a = math.sin(rad), math.cos(rad)
    dir_x, dir_y = -sin_a, cos_a
    bottom = (cx + dir_x * length / 2, cy + dir_y * length / 2)
    top = (cx - dir_x * length / 2, cy - dir_y * length / 2)
    perp = (cos_a, sin_a)
    return top, bottom, dir_x, dir_y, perp


def leaf_polygon(cx, cy, length, width, angle_deg):
    pts = []
    steps = 14
    for i in range(steps + 1):
        t = i / steps
        x = -width / 2 * math.sin(t * math.pi)
        y = length * (t - 0.5)
        pts.append((x, y))
    for i in range(steps + 1):
        t = i / steps
        x = width / 2 * math.sin(t * math.pi)
        y = length * (0.5 - t)
        pts.append((x, y))
    rad = math.radians(angle_deg)
    cos_a, sin_a = math.cos(rad), math.sin(rad)
    flat = []
    for x, y in pts:
        flat.append(cx + x * cos_a - y * sin_a)
        flat.append(cy + x * sin_a + y * cos_a)
    return flat


WAVE_AMPLITUDE_DEG = 14.0
WAVE_SPEED = 6.0  # radians/sec of oscillation while hovered


class LeafButton:
    """A button shaped like a leaf with a stem, filled in the green sampled
    from the background photo, oriented at angle_deg (90 = pointed ends on
    the left/right instead of top/bottom). Rocks side to side like it's
    being brushed by the cursor while hovered, and highlights/greys out for
    hover and enabled/disabled state."""

    def __init__(self, canvas, cx, cy, length, width, stem_len, text, command,
                 angle_deg=0, font=("sans-serif", 12, "bold")):
        self.canvas = canvas
        self.command = command
        self.enabled = True
        self.cx, self.cy = cx, cy
        self.length, self.width, self.stem_len = length, width, stem_len
        self.base_angle = angle_deg
        self._hovering = False
        self._wave_job = None
        self._wave_start = 0.0

        self.leaf_id = canvas.create_polygon(
            leaf_polygon(cx, cy, length, width, angle_deg), smooth=True,
            fill=LEAF_GREEN, outline=LEAF_GREEN_DARK, width=2)
        self.vein_id = canvas.create_line(0, 0, 0, 0, fill=LEAF_GREEN_DARK, width=1)
        self.stem_id = canvas.create_line(0, 0, 0, 0, 0, 0, smooth=True, fill=STEM_COLOR, width=3)
        self._redraw(angle_deg)
        self.text_id = canvas.create_text(cx, cy, text=text, fill=TEXT_CREAM, font=font)

        for item in (self.leaf_id, self.text_id):
            canvas.tag_bind(item, "<Enter>", self._on_enter)
            canvas.tag_bind(item, "<Leave>", self._on_leave)
            canvas.tag_bind(item, "<Button-1>", self._on_click)

    def _redraw(self, angle_deg):
        cx, cy, length, width = self.cx, self.cy, self.length, self.width
        self.canvas.coords(self.leaf_id, *leaf_polygon(cx, cy, length, width, angle_deg))
        top, bottom, dir_x, dir_y, perp = leaf_tip_points(cx, cy, length, angle_deg)
        self.canvas.coords(self.vein_id, *top, *bottom)
        mid = (bottom[0] + dir_x * self.stem_len * 0.6 + perp[0] * self.stem_len * 0.5,
               bottom[1] + dir_y * self.stem_len * 0.6 + perp[1] * self.stem_len * 0.5)
        end = (bottom[0] + dir_x * self.stem_len, bottom[1] + dir_y * self.stem_len)
        self.canvas.coords(self.stem_id, *bottom, *mid, *end)

    def _on_enter(self, _event):
        if not self.enabled:
            return
        self.canvas.itemconfigure(self.leaf_id, fill=LEAF_GREEN_LIGHT)
        self._hovering = True
        self._wave_start = time.time()
        if self._wave_job is None:
            self._wave_tick()

    def _on_leave(self, _event):
        self._hovering = False
        if self.enabled:
            self.canvas.itemconfigure(self.leaf_id, fill=LEAF_GREEN)
        if self._wave_job is not None:
            self.canvas.after_cancel(self._wave_job)
            self._wave_job = None
        self._redraw(self.base_angle)

    def _wave_tick(self):
        if not self._hovering:
            self._wave_job = None
            return
        t = time.time() - self._wave_start
        angle = self.base_angle + WAVE_AMPLITUDE_DEG * math.sin(t * WAVE_SPEED)
        self._redraw(angle)
        self._wave_job = self.canvas.after(30, self._wave_tick)

    def _on_click(self, _event):
        if self.enabled and self.command:
            self.command()

    def set_enabled(self, enabled):
        self.enabled = enabled
        self.canvas.itemconfigure(
            self.leaf_id, fill=LEAF_GREEN if enabled else LEAF_GREEN_DISABLED)

    def reposition(self, cx, cy):
        """Move this button to a new center, e.g. to track a window resize."""
        self.cx, self.cy = cx, cy
        self._redraw(self.base_angle)
        self.canvas.coords(self.text_id, cx, cy)


BUTTON_FONT = ("sans-serif", 12, "bold")
BUTTON_MAX_SPACING = 102  # leaf is 72 tall, so this keeps a 30px gap at most sizes
SMALL_BUTTON_FONT = ("sans-serif", 10, "bold")
LOG_FONT = ("sans-serif", 10, "bold")


class App:
    def __init__(self, root):
        self.root = root
        root.title("CAN Control Panel")
        root.resizable(True, True)
        root.minsize(CANVAS_WIDTH + 320, CANVAS_HEIGHT + 40)
        root.geometry(f"{CANVAS_WIDTH + 320}x{CANVAS_HEIGHT + 40}")

        # One canvas spans the whole window -- the background photo is
        # rescaled to fill it on every resize, rather than staying confined
        # to a fixed-size island next to a separately-colored log area.
        self.canvas = tk.Canvas(root, highlightthickness=0, bg=BEIGE_BG)
        self.canvas.pack(fill="both", expand=True)

        self._bg_photo = None  # keep a reference so Tk doesn't garbage-collect it
        self._bg_image_id = None
        self._last_size = (0, 0)
        self._resize_job = None
        if Image is not None and os.path.exists(BACKGROUND_PATH):
            self._bg_source = Image.open(BACKGROUND_PATH)
        else:
            self._bg_source = None

        self.canvas.create_text(20, 22, anchor="w", text="CAN Control Panel",
                                 fill=TITLE_BROWN, font=("sans-serif", 15, "bold"), tags="chrome")
        self.canvas.create_text(20, 46, anchor="w",
                                 text=f"Interface: {IFACE}    slcand: -{BITRATE_SLCAND_CODE} (500 kbit/s)",
                                 fill=TITLE_BROWN, font=("sans-serif", 9), tags="chrome")

        # Leaves rotated 90 degrees from the earlier design -- pointed ends
        # left/right instead of top/bottom -- so length/width are swapped
        # from before to keep the same footprint, just turned sideways.
        self.support_btn = LeafButton(self.canvas, CANVAS_WIDTH - 70, 32, length=86, width=40,
                                       stem_len=8, text="Support", command=self.on_support,
                                       angle_deg=90, font=SMALL_BUTTON_FONT)

        # Column sits over the clear left side of the photo and stays put on
        # resize -- only the right side (log panel) grows into new space.
        col_cx = 145
        self.setup_btn = LeafButton(self.canvas, col_cx, 106, length=190, width=72, stem_len=14,
                                     text="Set Up CAN", command=self.on_setup,
                                     angle_deg=90, font=BUTTON_FONT)
        self.enable_btn = LeafButton(self.canvas, col_cx, 174, length=190, width=72, stem_len=14,
                                      text="Enable Motor", command=self.on_enable,
                                      angle_deg=90, font=BUTTON_FONT)
        self.disable_btn = LeafButton(self.canvas, col_cx, 242, length=190, width=72, stem_len=14,
                                       text="Disable Motor", command=self.on_disable,
                                       angle_deg=90, font=BUTTON_FONT)
        self.shutdown_btn = LeafButton(self.canvas, col_cx, 310, length=190, width=72, stem_len=14,
                                        text="Shut Down CAN", command=self.on_shutdown,
                                        angle_deg=90, font=BUTTON_FONT)
        self.action_buttons = [self.setup_btn, self.enable_btn,
                               self.disable_btn, self.shutdown_btn]

        self.attribution_id = self.canvas.create_text(
            CANVAS_WIDTH - 12, CANVAS_HEIGHT - 12, anchor="se",
            text="Photo by Annie Spratt on Unsplash", fill=TEXT_CREAM, font=("sans-serif", 8))

        self.status_id = self.canvas.create_text(
            0, 54, anchor="nw", text="Ready.",
            fill=LEAF_GREEN_DARK, font=SMALL_BUTTON_FONT)

        self.log_text = tk.Text(self.canvas, bg="#000000", fg=LEAF_GREEN,
                                 font=LOG_FONT, wrap="word", bd=0,
                                 highlightthickness=0, insertbackground=LEAF_GREEN)
        self.log_text.configure(state="disabled")
        self.log_text_window = None

        self.canvas.bind("<Configure>", self._on_resize)
        self._layout(CANVAS_WIDTH + 320, CANVAS_HEIGHT + 40)

        if can is None:
            self.log("WARNING: python-can is not importable in this Python environment -- "
                      "the disable button's CAN scan will fail until that is fixed.")
        if self._bg_source is None:
            self.log("NOTE: Pillow is not importable -- showing a plain background instead "
                      "of the painted one.")

    def _on_resize(self, event):
        if self._resize_job is not None:
            self.root.after_cancel(self._resize_job)
        self._resize_job = self.root.after(60, lambda: self._layout(event.width, event.height))

    def _layout(self, w, h):
        self._resize_job = None
        w, h = max(w, 200), max(h, 150)
        if (w, h) == self._last_size:
            return
        self._last_size = (w, h)

        # The left (photo/buttons) side is always 3/8 of the window.
        side_x1 = round(w * 3 / 8)

        # Photo fills only the left side (up to where the beige log side
        # begins), not the whole window. Uses ImageOps.fit (crop-to-cover)
        # rather than a plain resize, which would stretch width and height
        # independently and visibly squish the photo whenever the box's
        # aspect ratio doesn't match the source image's.
        if self._bg_source is not None:
            photo_w = max(1, side_x1)
            if ImageOps is not None:
                resized = ImageOps.fit(self._bg_source, (photo_w, h),
                                        method=Image.LANCZOS, centering=(0.65, 0.6))
            else:
                resized = self._bg_source.resize((photo_w, h))
            self._bg_photo = ImageTk.PhotoImage(resized)
            if self._bg_image_id is None:
                self._bg_image_id = self.canvas.create_image(0, 0, anchor="nw", image=self._bg_photo)
                self.canvas.tag_lower(self._bg_image_id)
            else:
                self.canvas.itemconfigure(self._bg_image_id, image=self._bg_photo)

        # Spread the column between the header and the photo attribution
        # instead of using fixed y's -- with four buttons the old fixed
        # spacing ran off the bottom of the shorter window sizes.
        col_cx = side_x1 / 2
        col_top, col_bottom = 106, max(180, h - 76)
        n = len(self.action_buttons)
        step = min(BUTTON_MAX_SPACING, (col_bottom - col_top) / (n - 1))
        for i, btn in enumerate(self.action_buttons):
            btn.reposition(col_cx, col_top + i * step)

        self.support_btn.reposition(w - 70, 32)
        self.canvas.coords(self.attribution_id, side_x1 - 12, h - 12)

        margin = 6
        x1 = side_x1 + 20
        y1, x2, y2 = 70, max(x1 + 40, w - margin), max(140, h - margin)

        # A beige panel behind the log side (not just the photo showing
        # through) -- the whole output side gets its own readable surface,
        # like the earlier two-pane design had.
        self.canvas.delete("log_side_bg")
        self.canvas.create_rectangle(side_x1, 0, w, h, fill=BEIGE_BG, width=0, tags="log_side_bg")

        self.canvas.delete("log_panel")
        self.canvas.create_polygon(
            rounded_rect_points(x1, y1, x2, y2, 20), smooth=True,
            fill="#000000", outline=LEAF_GREEN_DARK, width=2, tags="log_panel")

        # Stacking order, bottom to top: background photo, beige side panel,
        # black log panel, then everything else (buttons/text, created
        # earlier and left alone).
        if self._bg_image_id is not None:
            self.canvas.tag_lower(self._bg_image_id)
        self.canvas.tag_lower("log_side_bg")
        if self._bg_image_id is not None:
            self.canvas.tag_raise("log_side_bg", self._bg_image_id)
        self.canvas.tag_lower("log_panel")
        self.canvas.tag_raise("log_panel", "log_side_bg")

        self.canvas.coords(self.status_id, x1 + 14, 48)
        inset = 20  # >= the polygon's own corner radius, so the widget's square
                    # corners stay inside the rounded shape instead of poking out
        text_x1, text_y1 = x1 + inset, y1 + inset
        text_w = max(10, (x2 - inset) - text_x1)
        text_h = max(10, (y2 - inset) - text_y1)
        if self.log_text_window is None:
            self.log_text_window = self.canvas.create_window(
                text_x1, text_y1, anchor="nw", window=self.log_text,
                width=text_w, height=text_h)
        else:
            self.canvas.coords(self.log_text_window, text_x1, text_y1)
            self.canvas.itemconfigure(self.log_text_window, width=text_w, height=text_h)

    def log(self, message):
        def _append():
            self.log_text.configure(state="normal")
            self.log_text.insert("end", message + "\n")
            self.log_text.see("end")
            self.log_text.configure(state="disabled")
        self.root.after(0, _append)

    def set_buttons_enabled(self, enabled):
        for btn in self.action_buttons:
            btn.set_enabled(enabled)

    def set_status(self, text):
        self.root.after(0, lambda: self.canvas.itemconfigure(self.status_id, text=text))

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

    def on_enable(self):
        self.run_in_background(enable_motor, "Enabling motor...")

    def on_disable(self):
        self.run_in_background(disable_motor, "Disabling motor...")

    def on_shutdown(self):
        self.run_in_background(shutdown_can, "Shutting down CAN (a password prompt may appear)...")

    def on_support(self):
        self.run_in_background(open_support_email, "Opening your email client...")


def main():
    root = tk.Tk()
    App(root)
    root.mainloop()


if __name__ == "__main__":
    main()
