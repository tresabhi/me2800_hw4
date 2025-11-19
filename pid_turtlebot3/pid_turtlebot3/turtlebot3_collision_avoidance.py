#!/usr/bin/env python3

# Importing required modules
import rclpy  # ROS 2 Python client library
from rclpy.node import Node  # Base class for ROS 2 nodes
from geometry_msgs.msg import Twist  # Message type for velocity commands
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan  # import sensor message package


# Define the main controller node class
class CA_Node(Node):
    def __init__(self):
        super().__init__(
            "turtlebot3_collision_avoidance"
        )  # Initialize the node with a name
        self.get_logger().info("Node Started")  # Log node initialization

        # Publisher for sending velocity commands
        self.my_vel_command = self.create_publisher(Twist, "/cmd_vel", 10)
        self.subscriber_ = self.create_subscription(
            LaserScan, "/scan", self.timer_callback, 10
        )

    # Callback to process turtle's laser scan and compute control commands
    def timer_callback(self, msg):  #: LaserScan):
        import math

        # Function to get the range for a specific angle
        def get_range_for_angle(scan, angle_deg):
            n = len(scan)

            # Shouldn't ever happen
            if n == 0:
                return float("inf")

            # MATLAB-like extraction pattern
            idx = int(round(angle_deg)) % n

            # This can fail sometimes so assume we're out of range
            try:
                r = scan[idx]
            except Exception:
                return float("inf")

            if r == 0.0 or r is None or math.isinf(r) or math.isnan(r):
                return float("inf")

            return float(r)

        front = get_range_for_angle(msg.ranges, 0)
        front_15 = get_range_for_angle(msg.ranges, 15)
        left = get_range_for_angle(msg.ranges, 90)
        right = get_range_for_angle(msg.ranges, 270)
        front_345 = get_range_for_angle(msg.ranges, 345)

        print("Front-direction laser scan:", front)
        print("15 deg laser scan:", front_15)
        print("Left-direction laser scan:", left)
        print("Right-direction laser scan:", right)
        print("345 deg laser scan:", front_345)

        SAFE_DIST = 0.8
        CAUTION_DIST = 0.5
        TOO_CLOSE = 0.35

        l_v = 0.0
        a_v = 0.0

        # Completely clear ahead and around: go forward
        if front > SAFE_DIST and front_15 > SAFE_DIST and front_345 > SAFE_DIST:
            l_v = 0.20  # forward speed (m/s) - small safe value
            a_v = 0.0

        # Immediate danger anywhere in front arc: stop and turn away
        elif front < TOO_CLOSE or front_15 < TOO_CLOSE or front_345 < TOO_CLOSE:
            l_v = 0.0
            left_space = min(left, 10.0)
            right_space = min(right, 10.0)

            # Turn in place
            if left_space > right_space:
                a_v = 0.8
            else:
                a_v = -0.8

        # Something is approaching but not dangerously close: slow and steer around it
        else:
            if front <= CAUTION_DIST:
                l_v = 0.05
            else:
                l_v = 0.12

            # Simple steering command using front-left vs front-right
            left_feel = min(front_15, 10.0)
            right_feel = min(front_345, 10.0)
            diff = left_feel - right_feel

            K_ang = 0.8
            a_v = K_ang * math.tanh(2 * diff)  # I love hyperbolic tangent :)

            if front < SAFE_DIST:
                if front_15 < front_345:
                    a_v = -abs(a_v) - 0.2
                else:
                    a_v = abs(a_v) + 0.2

        self.my_velocity_cont(l_v, a_v)

    # Publish velocity commands to the topic
    def my_velocity_cont(self, l_v, a_v):
        my_msg = Twist()
        my_msg.linear.x = l_v  # Linear velocity
        my_msg.angular.z = a_v  # Angular velocity
        self.my_vel_command.publish(my_msg)  # Publish the message


# Entry point of the script
def main(args=None):
    rclpy.init(args=args)  # Initialize ROS 2
    node = CA_Node()  # Create an instance of the node
    rclpy.spin(node)  # Keep the node running
    node.destroy_node()  # Destroy the node when done


# Run the script
if __name__ == "__main__":
    # if wait_for_gazebo_service():
    # Proceed with the rest of your robot control logic
    main()
