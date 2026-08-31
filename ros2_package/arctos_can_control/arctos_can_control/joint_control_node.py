#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from .mks_driver import MKSServoDriver

class ArctosJointControlNode(Node):
    def __init__(self):
        super().__init__('arctos_joint_control_node')
        
        # 1. Initialize our calibrated CAN library
        self.JOINT_B_ID = 0x05
        self.arm = MKSServoDriver(channel="can0")
        
        # Enable the motor coils on startup
        self.get_logger().info("Powering up motor coils for Joint B...")
        self.arm.set_motor_state(self.JOINT_B_ID, enable=True)
        
        # 2. ROS 2 Subscriber: Listens for raw target degrees to execute
        self.command_subscription = self.create_subscription(
            Float64,
            '/joint_b_command',
            self.joint_command_callback,
            10
        )
        
        # 3. ROS 2 Publisher: Continuously broadcasts the joint state to the network
        self.state_publisher = self.create_publisher(JointState, '/joint_states', 10)
        
        # 4. Timer loop: Query the physical encoder and publish at 20Hz (every 0.05s)
        self.timer = self.create_timer(0.05, self.publish_joint_state)
        self.get_logger().info("🤖 Arctos CAN/ROS2 Control Node fully initialized.")

    def joint_command_callback(self, msg):
        """Triggers when a new target degree command is published to the topic."""
        target_degrees = msg.data
        self.get_logger().info(f"Received movement command: {target_degrees} degrees")
        
        try:
            # Call your verified, safe relative motion function
            self.arm.move_relative_degrees(self.JOINT_B_ID, degrees=target_degrees, speed=40, accel=10)
        except Exception as e:
            self.get_logger().error(f"Failed to transmit CAN motion frame: {e}")

    def publish_joint_state(self):
        """Queries telemetry directly from the MKS and exposes it as a standard JointState message."""
        try:
            raw_encoder = self.arm.read_absolute_encoder(self.JOINT_B_ID)
            
            # Map raw counts back to standard physics measurements
            # 810.8 counts/degree * (pi / 180) = ~14.15 counts per radian
            counts_per_radian = (810.8 * 180.0) / 3.14159265
            radians_position = raw_encoder / counts_per_radian
            
            # Construct standard ROS 2 JointState message
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = ['joint_b']
            msg.position = [float(radians_position)]
            
            self.state_publisher.publish(msg)
        except Exception as e:
            # Non-blocking log warning if a single CAN frame drops
            self.get_logger().warn(f"Telemetry frame dropped: {e}")

    def destroy_node(self):
        """Safety cleanup sequence when shutting down the node."""
        self.get_logger().info("Shutting down node safely: Disabling motor power coils.")
        self.arm.set_motor_state(self.JOINT_B_ID, enable=False)
        self.arm.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ArctosJointControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
