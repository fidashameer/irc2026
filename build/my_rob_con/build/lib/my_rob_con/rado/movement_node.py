import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, Int16
from geometry_msgs.msg import Twist
import time

class MovementNode(Node):
    def __init__(self):
        super().__init__('movement_node')

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.rotation_cmd_pub = self.create_publisher(Int16, 'rotate_angle_cmd', 10)

        # Subscribers
        self.cone_detected_sub = self.create_subscription(Bool, 'cone_detected', self.cone_detected_callback, 10)
        self.cone_position_sub = self.create_subscription(Float32, 'cone_position', self.cone_position_callback, 10)
        self.cone_coverage_sub = self.create_subscription(Float32, 'cone_coverage', self.cone_coverage_callback, 10)
        self.rotation_done_sub = self.create_subscription(Bool, 'rotation_done', self.rotation_done_callback, 10)

        # State variables
        self.cone_detected = False
        self.cone_x = 0.5
        self.cone_coverage = 0.0
        self.target_reached = False
        self.waiting_for_rotation = False

        # Movement parameters
        self.forward_speed = 0.25
        self.frame_center_tolerance = 0.05
        self.target_coverage = 65.0
        self.exploration_distance = 3.0  # seconds per direction

        # Exploration directions
        self.directions = ['north', 'east', 'west', 'south']

        self.get_logger().info("Movement Node started — scanning for cones...")
        time.sleep(1)

        # Start with rotating 360° at beginning while scanning
        self.get_logger().info("Starting 360° rotation scan at start")
        if self.rotate_and_scan():
            self.approach_cone()
        else:
            self.main_loop()

    # -------------------------
    # Subscriber callbacks
    # -------------------------
    def cone_detected_callback(self, msg: Bool):
        self.cone_detected = msg.data

    def cone_position_callback(self, msg: Float32):
        self.cone_x = msg.data

    def cone_coverage_callback(self, msg: Float32):
        self.cone_coverage = msg.data

    def rotation_done_callback(self, msg: Bool):
        self.waiting_for_rotation = False

    # -------------------------
    # Main Exploration Logic
    # -------------------------
    def main_loop(self):
        for direction in self.directions:
            if self.target_reached:
                break

            self.get_logger().info(f"Moving {direction.upper()} 3 meters.")

            self.move_direction(direction, self.exploration_distance)

            self.get_logger().info(f"Rotating 360° at {direction.upper()} location while scanning for cones.")
            if self.rotate_and_scan():
                self.approach_cone()
                break

            self.get_logger().info(f"No cone detected after scanning at {direction.upper()}, returning to start.")
            self.return_to_start(self.exploration_distance)

        if not self.cone_detected and not self.target_reached:
            self.get_logger().warn("Cone not found in any direction, stopping search.")
            self.stop_robot()

    # -------------------------
    # Rotate while scanning for cones
    # -------------------------
    def rotate_and_scan(self, total_degrees=360, step_degrees=10):
        angular_speed = 0.5  # radians per second, tune as necessary
        step_radians = step_degrees * 3.14159 / 180
        step_time = step_radians / angular_speed

        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = angular_speed  # positive for CCW rotation

        turned_degrees = 0.0
        self.get_logger().info(f"Starting rotation scan of {total_degrees}°")

        while turned_degrees < total_degrees and rclpy.ok():
            if self.cone_detected:
                self.get_logger().info(f"Cone detected during rotation at {turned_degrees:.1f}°, stopping rotation.")
                self.stop_robot()
                return True

            self.cmd_vel_pub.publish(twist)
            time.sleep(step_time)
            turned_degrees += step_degrees

        self.stop_robot()
        self.get_logger().info(f"Completed full rotation of {total_degrees}°, no cone detected.")
        return False

    # -------------------------
    # Move forward in direction
    # -------------------------
    def move_direction(self, direction, forward_time):
        # Rotate robot to face direction first
        if direction == 'east':
            self.rotate_robot(90)
        elif direction == 'west':
            self.rotate_robot(-90)
        elif direction == 'south':
            self.rotate_robot(180)
        # north: no rotation needed

        twist = Twist()
        twist.linear.x = self.forward_speed

        start_time = time.time()
        while time.time() - start_time < forward_time and rclpy.ok():
            if self.cone_detected:
                self.stop_robot()
                return
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.05)

        self.stop_robot()

    # -------------------------
    # Return to start
    # -------------------------
    def return_to_start(self, duration):
        twist = Twist()
        twist.linear.x = -self.forward_speed
        self.get_logger().info("Returning to start...")
        start_time = time.time()
        while time.time() - start_time < duration and rclpy.ok():
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.05)
        self.stop_robot()

    # -------------------------
    # Approach cone logic
    # -------------------------
    def approach_cone(self):
        self.get_logger().info("Approaching cone and centering...")
        while rclpy.ok() and self.cone_detected and self.cone_coverage < self.target_coverage:
            err = self.cone_x - 0.5
            twist = Twist()

            if abs(err) > self.frame_center_tolerance:
                twist.angular.z = -0.5 * err
            else:
                twist.linear.x = self.forward_speed * 0.5

            self.cmd_vel_pub.publish(twist)
            time.sleep(0.05)

        self.stop_robot()
        if self.cone_coverage >= self.target_coverage:
            self.get_logger().info("✅ Cone reached — stopping movement.")
            self.target_reached = True

    # -------------------------
    # Rotate robot exact angle (blocking on rotation_done)
    # -------------------------
    def rotate_robot(self, angle_deg):
        if self.waiting_for_rotation:
            return

        self.get_logger().info(f"Rotating {angle_deg}°")
        rotate_msg = Int16()
        rotate_msg.data = angle_deg
        self.rotation_cmd_pub.publish(rotate_msg)
        self.waiting_for_rotation = True

        # Wait until rotation is done
        while self.waiting_for_rotation and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)

    # -------------------------
    # Stop all motion
    # -------------------------
    def stop_robot(self):
        self.cmd_vel_pub.publish(Twist())
        time.sleep(0.05)


# -------------------------
# Main
# -------------------------
def main(args=None):
    rclpy.init(args=args)
    node = MovementNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
