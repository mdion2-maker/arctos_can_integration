#!/usr/bin/env python3
"""
joint_state_publisher_node.py

Read-only ROS 2 telemetry node for a single Arctos joint.

Publishes /joint_states from a joint's real encoder over CAN so it can be
watched with `ros2 topic echo /joint_states` or plotted in rqt while the
joint is still uncalibrated. Deliberately has NO command subscriber -- until
a joint has been through the supervised tiny-move test and a real
calibration pass, nothing in this node can make the arm move.

Generic across joints via ROS parameters -- set joint_name/can_id/gear_ratio
for whichever joint is physically wired up right now, e.g.:

    ros2 run arctos_can_control joint_state_publisher --ros-args \\
        -p joint_name:=C_joint -p can_id:=6 -p gear_ratio:=67.82
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import can

READ_ENCODER = 0x31
ENCODER_CPR = 16384


class JointStatePublisherNode(Node):
    def __init__(self):
        super().__init__('joint_state_publisher_node')

        self.declare_parameter('joint_name', 'X_joint')
        self.declare_parameter('can_channel', 'can0')
        self.declare_parameter('can_id', 0x01)
        self.declare_parameter('gear_ratio', 13.5)
        self.declare_parameter('inverted', True)
        self.declare_parameter('publish_rate_hz', 10.0)

        self.joint_name = self.get_parameter('joint_name').value
        self.can_id = self.get_parameter('can_id').value
        self.gear_ratio = self.get_parameter('gear_ratio').value
        self.inverted = self.get_parameter('inverted').value
        channel = self.get_parameter('can_channel').value

        self.bus = can.interface.Bus(channel=channel, interface='socketcan')
        self.state_publisher = self.create_publisher(JointState, '/joint_states', 10)

        rate = self.get_parameter('publish_rate_hz').value
        self.timer = self.create_timer(1.0 / rate, self.publish_joint_state)

        self.get_logger().info(
            f"{self.joint_name} telemetry-only node started. CAN ID=0x{self.can_id:02X} "
            f"on {channel}, gear_ratio={self.gear_ratio}, inverted={self.inverted}. "
            "No motion commands are accepted by this node."
        )

    def checksum(self, motor_id, data_bytes):
        return (motor_id + sum(data_bytes)) & 0xFF

    def read_encoder(self, timeout=0.3):
        crc = self.checksum(self.can_id, [READ_ENCODER])
        msg = can.Message(arbitration_id=self.can_id, data=[READ_ENCODER, crc], is_extended_id=False)
        self.bus.send(msg)

        end = self.get_clock().now().nanoseconds / 1e9 + timeout
        while True:
            remaining = end - self.get_clock().now().nanoseconds / 1e9
            if remaining <= 0:
                return None
            resp = self.bus.recv(timeout=remaining)
            if resp is None:
                return None
            if resp.arbitration_id == self.can_id and len(resp.data) >= 8 and resp.data[0] == READ_ENCODER:
                raw48 = int.from_bytes(resp.data[1:7], byteorder='big', signed=True)
                return raw48

    def publish_joint_state(self):
        try:
            raw48 = self.read_encoder()
            if raw48 is None:
                self.get_logger().warn(f'{self.joint_name}: no encoder response this cycle',
                                        throttle_duration_sec=2.0)
                return

            motor_shaft_deg = raw48 * 360.0 / ENCODER_CPR
            joint_deg = motor_shaft_deg / self.gear_ratio
            if self.inverted:
                joint_deg = -joint_deg
            joint_rad = joint_deg * 3.14159265358979 / 180.0

            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = [self.joint_name]
            msg.position = [float(joint_rad)]
            self.state_publisher.publish(msg)
        except Exception as e:
            self.get_logger().warn(f'{self.joint_name} telemetry frame dropped: {e}')

    def destroy_node(self):
        self.bus.shutdown()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = JointStatePublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
