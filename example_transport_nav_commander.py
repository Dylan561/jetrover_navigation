#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.duration import Duration
import time
import math
import threading

class SmartCommander(Node):
    def __init__(self):
        super().__init__('smart_commander_node')
        
        # --- Use the Nav2 Simple Commander ---
        self.navigator = BasicNavigator()
        
        # --- DEFINE YOUR MISSION POSES HERE ---
        # This is a simple list of patrol points [x, y, yaw_degrees]
        self.patrol_points = [
            [0.336929641, 1.257132343, -4.123539359],
            [1.384044811029843, 1.188265382155733, 73.21390939],
            [0.158265500753959, -0.100743450929931, -71.07576787]
        ]

        # Start the main mission logic in a separate thread
        self.mission_thread = threading.Thread(target=self.run_mission)
        self.mission_thread.start()

    def run_mission(self):
        """The main function that orchestrates the patrol mission."""
        # Wait for Nav2 to be active
        self.navigator.waitUntilNav2Active()
        
        self.get_logger().info('Nav2 is active. Starting patrol mission.')
        
        for pose in self.patrol_points:
            # 1. Navigate to the goal using Nav2 Goal
            success = self.navigate_to_goal(pose[0], pose[1], pose[2])

            if not success:
                self.get_logger().error("Navigation failed. Aborting the rest of the mission.")
                break

        self.get_logger().info("--- Patrol mission complete! ---")
        rclpy.shutdown()

    def navigate_to_goal(self, x, y, yaw_degrees):
        """Sends a navigation goal using the Nav2 Simple Commander and waits for the result."""
        self.get_logger().info(f"--- Navigating to: x={x}, y={y} ---")
        
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        yaw_rad = math.radians(yaw_degrees)
        goal_pose.pose.orientation.z = math.sin(yaw_rad / 2.0)
        goal_pose.pose.orientation.w = math.cos(yaw_rad / 2.0)
        
        # Send the goal
        self.navigator.goToPose(goal_pose)
        
        # Wait until the task is complete
        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            if feedback:
                # Correctly print the estimated time remaining
                time_remaining = Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9
                self.get_logger().info(f'Estimated time remaining: {time_remaining:.0f} seconds.',
                                       throttle_duration_sec=5)

        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info("Navigation goal reached successfully!")
            return True
        else:
            self.get_logger().error(f"Navigation failed with status: {result.name}")
            return False

def main(args=None):
    rclpy.init(args=args)
    # The BasicNavigator needs a node to be spun for its callbacks to work.
    # We create our SmartCommander node and then let its thread handle the mission.
    commander = SmartCommander()
    
    # We wait for the mission thread to complete its work before shutting down.
    commander.mission_thread.join()
    
    commander.destroy_node()

if __name__ == '__main__':
    main()

