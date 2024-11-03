#!/usr/bin/env python3
import rclpy
import time
import math
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion, quaternion_from_euler

QOS_PROFILE_DEFAULT = 10
PI = math.pi

# Control parameters remain unchanged
LINEAR_P_GAIN = 0.5
ANGULAR_P_GAIN = 1.0
MAX_LINEAR_SPEED = 0.22
MAX_ANGULAR_SPEED = 0.22
MIN_LINEAR_SPEED = 0.05
MIN_ANGULAR_SPEED = 0.02

# Distance thresholds remain unchanged
GOAL_DISTANCE_THRESHOLD = 0.05
SLOWDOWN_DISTANCE = 0.2
HEADING_THRESHOLD = 0.05

class WaypointNode(Node):
    def __init__(self):
        super().__init__('waypoint_navigation_node')

        self.pub_twist = self.create_publisher(
            Twist,
            '/cmd_vel',
            QOS_PROFILE_DEFAULT)
        
        self.sub_odom = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            QOS_PROFILE_DEFAULT)
        
        # Waypoints list - add your waypoints here
        self.waypoints = [
            (3.0, 1.0),    # First waypoint
            (4.0, 2.0),    # Second waypoint
            (2.0, 3.0),    # Third waypoint
            # Add more waypoints as needed
        ]
        
        # Waypoint tracking
        self.current_waypoint_index = 0
        self.goal_x, self.goal_y = self.waypoints[0]
        
        # Current state
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        
        # Control state
        self.goal_reached = False
        self.final_approach = False
        self.all_waypoints_reached = False
        
        # Create timer for control loop
        self.control_timer = self.create_timer(0.05, self.control_loop)
        
        self.get_logger().info('Waypoint Navigation Node initialized')
        self.get_logger().info(f'Total waypoints: {len(self.waypoints)}')

    def move_to_next_waypoint(self):
        """Update goal to next waypoint"""
        self.current_waypoint_index += 1
        
        if self.current_waypoint_index < len(self.waypoints):
            self.goal_x, self.goal_y = self.waypoints[self.current_waypoint_index]
            self.goal_reached = False
            self.final_approach = False
            self.get_logger().info(
                f'Moving to waypoint {self.current_waypoint_index + 1}: '
                f'({self.goal_x}, {self.goal_y})'
            )
        else:
            self.all_waypoints_reached = True
            self.get_logger().info('All waypoints reached!')
            self.stop_robot()
            rclpy.shutdown()

    def calculate_control_commands(self):
        """Calculate linear and angular velocity commands with improved control"""
        # Calculate distance and heading to goal
        distance_to_goal = math.sqrt(
            (self.goal_x - self.current_x) ** 2 + 
            (self.goal_y - self.current_y) ** 2
        )
        
        desired_heading = math.atan2(
            self.goal_y - self.current_y,
            self.goal_x - self.current_x
        )
        
        # Calculate heading error (-π to π)
        heading_error = desired_heading - self.current_yaw
        heading_error = math.atan2(math.sin(heading_error), math.cos(heading_error))
        
        # Improved angular velocity control
        if abs(heading_error) > HEADING_THRESHOLD:
            angular_velocity = ANGULAR_P_GAIN * heading_error
        else:
            angular_velocity = (ANGULAR_P_GAIN * 0.5) * heading_error
            
        if abs(angular_velocity) < MIN_ANGULAR_SPEED and abs(heading_error) > 0.01:
            angular_velocity = math.copysign(MIN_ANGULAR_SPEED, angular_velocity)
        angular_velocity = max(min(angular_velocity, MAX_ANGULAR_SPEED), -MAX_ANGULAR_SPEED)
        
        # Improved linear velocity control
        if distance_to_goal > SLOWDOWN_DISTANCE:
            base_linear_velocity = LINEAR_P_GAIN * distance_to_goal
        else:
            slowdown_factor = distance_to_goal / SLOWDOWN_DISTANCE
            base_linear_velocity = max(
                MIN_LINEAR_SPEED,
                LINEAR_P_GAIN * distance_to_goal * slowdown_factor
            )
        
        heading_factor = max(0.0, math.cos(heading_error))
        linear_velocity = base_linear_velocity * heading_factor
        
        if heading_factor > 0.7 and distance_to_goal > GOAL_DISTANCE_THRESHOLD:
            linear_velocity = max(linear_velocity, MIN_LINEAR_SPEED)
            
        if distance_to_goal < GOAL_DISTANCE_THRESHOLD * 2:
            self.final_approach = True
            linear_velocity *= 0.5
            
        linear_velocity = max(min(linear_velocity, MAX_LINEAR_SPEED), 0.0)
        
        return linear_velocity, angular_velocity

    def control_loop(self):
        """Main control loop with waypoint handling"""
        if self.all_waypoints_reached:
            return
            
        distance_to_goal = math.sqrt(
            (self.goal_x - self.current_x) ** 2 + 
            (self.goal_y - self.current_y) ** 2
        )
        
        # Check if current waypoint is reached
        if (distance_to_goal <= GOAL_DISTANCE_THRESHOLD and 
            abs(self.get_heading_error()) <= HEADING_THRESHOLD):
            self.get_logger().info(
                f'Waypoint {self.current_waypoint_index + 1} reached! '
                f'Position: ({self.current_x:.3f}, {self.current_y:.3f})'
            )
            self.move_to_next_waypoint()
            return
        elif distance_to_goal > GOAL_DISTANCE_THRESHOLD * 2 and self.final_approach:
            # Reset final approach if robot moves too far from goal
            self.final_approach = False
            
        linear_velocity, angular_velocity = self.calculate_control_commands()
        self.publish_velocity(linear_velocity, angular_velocity)
        
        if self.final_approach:
            self.get_logger().debug(
                f'Final approach to waypoint {self.current_waypoint_index + 1} - '
                f'Distance: {distance_to_goal:.3f}, '
                f'Linear vel: {linear_velocity:.3f}, '
                f'Angular vel: {angular_velocity:.3f}'
            )

    def get_heading_error(self):
        """Calculate current heading error"""
        desired_heading = math.atan2(
            self.goal_y - self.current_y,
            self.goal_x - self.current_x
        )
        heading_error = desired_heading - self.current_yaw
        return math.atan2(math.sin(heading_error), math.cos(heading_error))

    def odom_callback(self, msg):
        """Update current pose from odometry"""
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        
        quat = msg.pose.pose.orientation
        quat_list = [quat.x, quat.y, quat.z, quat.w]
        _, _, self.current_yaw = euler_from_quaternion(quat_list)

    def publish_velocity(self, linear_vel, angular_vel):
        """Publish velocity commands"""
        vel_msg = Twist()
        vel_msg.linear.x = linear_vel
        vel_msg.angular.z = angular_vel
        self.pub_twist.publish(vel_msg)

    def stop_robot(self):
        """Stop the robot"""
        self.publish_velocity(0.0, 0.0)

def main(args=None):
    rclpy.init(args=args)
    node = WaypointNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()