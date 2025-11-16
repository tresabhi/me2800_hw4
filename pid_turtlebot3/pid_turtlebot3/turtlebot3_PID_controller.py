#!/usr/bin/env python3

# Importing required modules
import rclpy  # ROS 2 Python client library
import math  # For mathematical calculations
from rclpy.node import Node  # Base class for ROS 2 nodes
from geometry_msgs.msg import Twist  # Message type for velocity commands
from nav_msgs.msg import Odometry
from gazebo_msgs.srv import SpawnEntity  # GetModelList #GetModelState
from cpp_node.action import GoToPose  # Custom action for GoToPose
from rclpy.action import ActionServer, GoalResponse  # ROS 2 Action Server utilities
from rclpy.action.server import ServerGoalHandle  # Goal handle for the action server
from rclpy.callback_groups import (
    ReentrantCallbackGroup,
)  # To allow concurrent callbacks

# from rclpy.wait_for_message import wait_for_message  # asdf


# Define the main controller node class
class Controller_Node(Node):
    def __init__(self):
        super().__init__("turtlebot3_controller")  # Initialize the node with a name

        self.service_client = self.create_client(
            # srv_type=gazebo_msgs/srv/GetModelList,
            # srv_type=GetModelList,
            # srv_name='/get_model_list'
            srv_type=SpawnEntity,
            srv_name="/spawn_entity",
        )

        while not self.service_client.wait_for_service(timeout_sec=50.0):
            self.get_logger().info(
                f"service {self.service_client.srv_name} not available, waiting..."
            )

        self.get_logger().info(
            f"service {self.service_client.srv_name} is now available!"
        )

        # self.future: Future = None

        # while not rclpy.wait_for_message.wait_for_message(Odometry, turtlebot3_diff_drive, '/odom', time_to_wait=50.0):
        #    self.get_logger().info('Waiting for turtlebot3_diff_drive')

        self.get_logger().info("Node Started")  # Log node initialization

        # Desired position for the turtle
        self.desired_x = 5.544
        self.desired_y = 5.544

        # Initialize error variables
        self.err_dist = 0
        self.err_theta = 0

        # Initialize controller gains
        self.declare_parameter("kp_dist_parm", 0.0)
        self.declare_parameter("ki_dist_parm", 0.0)
        self.declare_parameter("kd_dist_parm", 0.0)
        self.declare_parameter("kp_ang_parm", 0.0)
        self.declare_parameter("ki_ang_parm", 0.0)
        self.declare_parameter("kd_ang_parm", 0.0)

        # Initialize terminal constraints
        self.declare_parameter("eps_dist_tol", 0.0)
        self.declare_parameter("eps_ang_tol", 0.0)

        # Use a ReentrantCallbackGroup to allow concurrent callback execution
        self.callback_group = ReentrantCallbackGroup()
        self.goal_completed = False  # Flag to track goal completion

        # Publisher for sending velocity commands
        self.my_vel_command = self.create_publisher(Twist, "/cmd_vel", 10)

        # Action server setup for the GoToPose action
        self._action_server = ActionServer(
            self,
            GoToPose,  # Action type
            "GoToPose",  # Action name
            callback_group=self.callback_group,
            goal_callback=self.goal_callback,  # Callback when a goal is received
            execute_callback=self.execute_callback,  # Callback to execute the goal
        )

        self.dist_tol = self.get_parameter("eps_dist_tol").value
        self.ang_tol = self.get_parameter("eps_dist_tol").value

        # self.declare_parameter('use_sim_time', True)
        # self.get_logger().info(f"Using simulation time: {self.get_parameter('use_sim_time').value}")
        self.set_parameters(
            [rclpy.Parameter("use_sim_time", rclpy.Parameter.Type.BOOL, True)]
        )
        # self.current_sim_time = 0.0

    # Callback to handle incoming goals
    def goal_callback(self, goal_request: GoToPose.Goal):
        self.get_logger().info("Received a goal...")
        # Check if the absolute values of x and y are less than or equal to 11
        if (
            abs(goal_request.desired_x_pos) <= 11
            and abs(goal_request.desired_y_pos) <= 11
        ):
            self.get_logger().info(
                f"Goal accepted with x: {goal_request.desired_x_pos}, y: {goal_request.desired_y_pos}"
            )
            return GoalResponse.ACCEPT  # Accept the goal
        else:
            self.get_logger().info(
                f"Goal rejected with x: {goal_request.desired_x_pos}, y: {goal_request.desired_y_pos}"
            )
            return GoalResponse.REJECT  # Reject the goal

    # Callback to execute the goal
    def execute_callback(self, goal_handle: ServerGoalHandle):
        self.get_logger().info("Executing goal...")
        self.goal_handle = goal_handle  # Store the goal handle

        # Update desired position based on goal
        self.desired_x = goal_handle.request.desired_x_pos
        self.desired_y = goal_handle.request.desired_y_pos

        # Subscribe to the pose topic to get feedback on turtle's position
        self.my_pose_sub = self.create_subscription(
            Odometry,
            "/odom",
            self.pose_callback,
            10,
            callback_group=self.callback_group,
        )

        # Initialize result object for the action
        self.result = GoToPose.Result()

        try:
            # Create a periodic timer to check if the goal is reached
            self.goal_timer = self.create_timer(0.1, self.check_goal_reached)

            # Spin the node to handle other callbacks while the goal is active
            while not self.goal_handle.is_cancel_requested:
                rclpy.spin_once(self, timeout_sec=0.1)
                if self.goal_completed:  # Exit if goal is completed
                    break

        except Exception as e:
            # Handle exceptions during goal execution
            self.get_logger().error(f"Error during goal execution: {e}")
            self.goal_handle.abort()  # Abort the goal
            self.result.sucess = False

        # Return the result object
        return self.result

    # Periodic callback to check if the goal is reached
    def check_goal_reached(self):
        # Check if the position and heading errors are within tolerances
        if self.err_dist < self.dist_tol and abs(self.err_theta) < self.ang_tol:
            # save time results

            # self.current_sim_time += self.get_clock().now()

            current_sim_time = self.get_clock().now()
            time_msg = current_sim_time.to_msg()
            total_seconds = time_msg.sec + time_msg.nanosec * 1e-9
            #
            # write information to file
            fname = (
                "time/"
                + str(total_seconds)
                + "sec"
                + "_kpl"
                + str(self.get_parameter("kp_dist_parm").value)
                + "_kil"
                + str(self.get_parameter("ki_dist_parm").value)
                + "_kdl"
                + str(self.get_parameter("kd_dist_parm").value)
                + "_kpa"
                + str(self.get_parameter("kp_ang_parm").value)
                + "_kia"
                + str(self.get_parameter("ki_ang_parm").value)
                + "_kda"
                + str(self.get_parameter("kd_ang_parm").value)
                + ".txt"
            )
            f = open(fname, "w")
            f.write(f"Total time to goal: {total_seconds:.2f} seconds \n")
            f.write(f"Location of goal, (x,y)=({self.desired_x},{self.desired_y}) \n")
            f.write(f"Linear gains: Kp={self.get_parameter('kp_dist_parm').value}, ")
            f.write(f"Ki={self.get_parameter('ki_dist_parm').value}, ")
            f.write(f"Kd={self.get_parameter('kd_dist_parm').value} \n")
            f.write(f"Angular gains: Kp={self.get_parameter('kp_ang_parm').value}, ")
            f.write(f"Ki={self.get_parameter('ki_ang_parm').value}, ")
            f.write(f"Kd={self.get_parameter('kd_ang_parm').value} \n")
            f.close()

            # now regular ROS stuff
            self.get_logger().info("Goal reached successfully!")
            self.get_logger().info(
                "Total time to goal: " + str(total_seconds) + " seconds"
            )
            self.result.sucess = True
            self.goal_handle.succeed()  # Mark the goal as succeeded
            self.goal_timer.cancel()  # Stop the periodic checks
            self.goal_completed = True
            # self.goal_timer.cancel()  # Stop the periodic checks
            # self.goal_completed = True
        else:
            self.get_logger().info("Still moving towards the goal...")

    # Callback to process turtle's pose and compute control commands
    def pose_callback(self, msg):  #: Odometry):
        # Publish feedback for the action
        Feedback = GoToPose.Feedback()
        Feedback.current_x_pos = msg.pose.pose.position.x
        Feedback.current_y_pos = msg.pose.pose.position.y
        self.goal_handle.publish_feedback(Feedback)

        # Calculate positional errors
        err_x = self.desired_x - msg.pose.pose.position.x
        err_y = self.desired_y - msg.pose.pose.position.y
        self.err_dist = math.sqrt(err_x**2 + err_y**2)

        # Calculate heading error
        desired_theta = math.atan2(err_y, err_x)
        self.err_theta = desired_theta - msg.pose.pose.orientation.z

        # Wrap heading error within [-pi, pi]
        while self.err_theta > math.pi:
            self.err_theta -= 2.0 * math.pi
        while self.err_theta < -math.pi:
            self.err_theta += 2.0 * math.pi

        # PID gains for distance and heading control
        # self.declare_parameter('kp_dist_parm', 2.0)
        Kp_dist = self.get_parameter("kp_dist_parm").value
        Ki_dist = self.get_parameter("ki_dist_parm").value
        Kd_dist = self.get_parameter("kd_dist_parm").value
        Kp_theta = self.get_parameter("kp_ang_parm").value
        Ki_theta = self.get_parameter("ki_ang_parm").value
        Kd_theta = self.get_parameter("kd_ang_parm").value

        # Initialize integral and derivative terms
        integral_dist = 0.0
        previous_err_dist = 0.0
        integral_theta = 0.0
        previous_err_theta = 0.0

        # PID control for linear velocity
        if self.err_dist >= self.dist_tol:
            l_v = (
                Kp_dist * abs(self.err_dist)
                + Ki_dist * integral_dist
                + Kd_dist * (self.err_dist - previous_err_dist)
            )
            previous_err_dist = self.err_dist
        else:
            # self.get_logger().info(f"Turtlebot3 stopping as goal distance is within tolerance")
            l_v = 0.0

        # PID control for angular velocity
        if abs(self.err_theta) >= self.ang_tol:
            a_v = (
                Kp_theta * self.err_theta
                + Ki_theta * integral_theta
                + Kd_theta * (self.err_theta - previous_err_theta)
            )
            previous_err_theta = self.err_theta
        else:
            # self.get_logger().info(f"Turtlebot3 stopping as goal heading is within tolerance")
            a_v = 0.0

        # Send computed velocity commands
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
    node = Controller_Node()  # Create an instance of the node
    # while not wait_for_message(Odometry, node, '/odom', time_to_wait=50.0):
    #    self.get_logger().info('Waiting for turtlebot3_diff_drive')
    rclpy.spin(node)  # Keep the node running
    node.destroy_node()  # Destroy the node when done
    # rclpy.shutdown()  # Shutdown ROS 2


# Run the script
if __name__ == "__main__":
    # if wait_for_gazebo_service():
    # Proceed with the rest of your robot control logic
    main()
