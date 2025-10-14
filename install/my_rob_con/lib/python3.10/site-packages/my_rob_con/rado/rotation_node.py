import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int16
from geometry_msgs.msg import Twist
import time
import math

class RotationNode(Node):
    def __init__(self):
        super().__init__('rotation_node')

        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.rotation_done_pub = self.create_publisher(Bool, 'rotation_done', 10)
        self.rotation_command_sub = self.create_subscription(Int16, 'rotate_angle_cmd', self.rotate_callback, 10)

        self.rotation_speed = 0.5  # rad/s, positive direction CCW
        self.is_rotating = False

        self.get_logger().info('Rotation Node started')

    def rotate_callback(self, msg):
        if self.is_rotating:
            self.get_logger().warn('Already rotating, ignoring new command')
            return

        angle_deg = msg.data
        if angle_deg == 0:
            self.get_logger().info('Received zero rotation command, skipping.')
            # Immediately publish done if needed
            done_msg = Bool()
            done_msg.data = True
            self.rotation_done_pub.publish(done_msg)
            return

        self.get_logger().info(f'Received rotation command: {angle_deg} degrees')
        self.is_rotating = True
        self.perform_rotation(angle_deg)
        self.is_rotating = False

    def perform_rotation(self, angle_deg):
        angle_rad = math.radians(abs(angle_deg))
        duration = angle_rad / self.rotation_speed
        twist = Twist()

        # Set angular direction (positive for CCW, negative for CW)
        twist.angular.z = self.rotation_speed if angle_deg > 0 else -self.rotation_speed
        self.get_logger().info(f'Starting rotation: {"CCW" if angle_deg > 0 else "CW"} at {abs(math.degrees(twist.angular.z)):.2f} deg/s for {duration:.2f} seconds')

        start_time = time.time()
        while time.time() - start_time < duration and rclpy.ok():
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.05)

        self.stop_rotation()
        self.get_logger().info('Rotation complete')

        # Publish rotation done
        done_msg = Bool()
        done_msg.data = True
        self.rotation_done_pub.publish(done_msg)

    def stop_rotation(self):
        self.cmd_vel_pub.publish(Twist())
        time.sleep(0.05)

def main(args=None):
    rclpy.init(args=args)
    node = RotationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
