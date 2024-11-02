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

# Improved control parameters
LINEAR_KP = 0.5        # Increas for better convergence
ANGULAR_KP = 1.0       # Increas for faster heading correction
MAX_LINEAR_SPEED = 0.22
MAX_ANGULAR_SPEED = 0.22
MIN_LINEAR_SPEED = 0.05    # Small non-zero minimum speed to prevent stalling
MIN_ANGULAR_SPEED = 0.02   # Small minimum angular speed for fine adjustments

# Distance thresholds
GOAL_DISTANCE_THRESHOLD = 0.05  # Tighter threshold for goal reaching
SLOWDOWN_DISTANCE = 0.2        # Distance at which to start slowing down
HEADING_THRESHOLD = 0.05       # Radians (~3 degrees) for heading alignment

class GoalNode(Node):
    def __init__(self):
        super().__init__('goal_navigation_node')

        self.pub_twist = self.create_publisher(
            Twist,
            '/cmd_vel',
            QOS_PROFILE_DEFAULT)
        
        self.sub_odom = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            QOS_PROFILE_DEFAULT)
        
        # Goal position
        self.goal_x = 3.0
        self.goal_y = 1.0
        
        # Current state
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        
        # Control state
        self.goal_reached = False
        self.final_approach = False
        
        # Create timer for control loop (20 Hz for more responsive control)
        self.control_timer = self.create_timer(0.05, self.control_loop)
        
        self.get_logger().info('Initialized goal navigation node')

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
            # Use full angular control when heading error is significant
            angular_velocity = ANGULAR_KP * heading_error
        else:
            # Fine adjustments when nearly aligned
            angular_velocity = (ANGULAR_KP * 0.5) * heading_error
            
        # Limit angular velocity with minimum threshold for small corrections
        if abs(angular_velocity) < MIN_ANGULAR_SPEED and abs(heading_error) > 0.01:
            angular_velocity = math.copysign(MIN_ANGULAR_SPEED, angular_velocity)
        angular_velocity = max(min(angular_velocity, MAX_ANGULAR_SPEED), -MAX_ANGULAR_SPEED)
        
        # Improved linear velocity control
        if distance_to_goal > SLOWDOWN_DISTANCE:
            # Full speed when far from goal
            base_linear_velocity = LINEAR_KP * distance_to_goal
        else:
            # Gradual slowdown approaching goal, but maintain minimum speed
            slowdown_factor = distance_to_goal / SLOWDOWN_DISTANCE
            base_linear_velocity = max(
                MIN_LINEAR_SPEED,
                LINEAR_KP * distance_to_goal * slowdown_factor
            )
        
        """
        -> Linear velocity (that robot should achieve) depends on both distance and heading alignment
          of the robot wrt goal.
        -> The heading factor creates smooth speed transition during turns.
        -> math.cos(heading_error) ranges from:
            -> 1.0 when heading_error = 0° (perfect alignment)
            -> 0.0 when heading_error = 90° or -90°
            -> -1.0 when heading_error = 180° (facing opposite direction)
        -> Squaring it (** 2) ensures:
            -> Always positive value between 0 and 1
            -> More aggressive speed reduction during turns
        """

        heading_factor = max(0.0, math.cos(heading_error))
        linear_velocity = base_linear_velocity * heading_factor
        
        # Ensure minimum forward velocity when aligned and not at goal
        if heading_factor > 0.7 and distance_to_goal > GOAL_DISTANCE_THRESHOLD:
            linear_velocity = max(linear_velocity, MIN_LINEAR_SPEED)
            
        # Final approach behavior
        if distance_to_goal < GOAL_DISTANCE_THRESHOLD * 2:
            self.final_approach = True
            linear_velocity *= 0.5  # Slower final approach
            
        # Limit final linear velocity
        linear_velocity = max(min(linear_velocity, MAX_LINEAR_SPEED), 0.0)
        
        return linear_velocity, angular_velocity

    def control_loop(self):
        """Main control loop with improved goal detection"""
        if self.goal_reached:
            return
            
        distance_to_goal = math.sqrt(
            (self.goal_x - self.current_x) ** 2 + 
            (self.goal_y - self.current_y) ** 2
        )
        
        # More precise goal reaching condition
        if (distance_to_goal <= GOAL_DISTANCE_THRESHOLD and 
            abs(self.get_heading_error()) <= HEADING_THRESHOLD):
            self.goal_reached = True
            self.stop_robot()
            self.get_logger().info(
                f'Goal reached! Final position: ({self.current_x:.3f}, {self.current_y:.3f})'
            )
            rclpy.shutdown()
            return
            
        linear_velocity, angular_velocity = self.calculate_control_commands()
        self.publish_velocity(linear_velocity, angular_velocity)
        
        # Debug logging
        if self.final_approach:
            self.get_logger().debug(
                f'Final approach - Distance: {distance_to_goal:.3f}, '
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
    node = GoalNode()

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