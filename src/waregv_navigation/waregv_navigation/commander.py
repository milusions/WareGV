#!/usr/bin/env python3

import sys
import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.duration import Duration
from rclpy.node import Node
from std_msgs.msg._empty import Empty
from std_msgs.msg._string import String
from rclpy.executors import SingleThreadedExecutor
from geometry_msgs.msg._pose_array import PoseArray

class CommanderNode(Node):
    def __init__(self,node_name):
        super().__init__(node_name)
        self.navigator = BasicNavigator()
        
        self.create_subscription(PoseStamped, '/nav_to_pose', self.handle_nav_to_pose, 10)
        self.create_subscription(PoseArray, '/follow_waypoints', self.handle_follow_waypoints, 10)
        self.create_subscription(PoseStamped, '/set_initial_pose', self.set_initial_pose, 10)
        self.create_subscription(Empty, '/abort_mission', self.handle_abort, 10)
        self.status_pub = self.create_publisher(String, '/nav_mission_status', 10)

        self.get_logger().info('Nav2 Manager Node is running and listening for commands...')
        
            

    def set_initial_pose(self, msg: PoseStamped):
        """Sets the initial pose for AMCL localization and waits for Nav2 active state."""
        self.navigator.setInitialPose(msg)


    def monitor_task(self, max_duration_sec: float = 120.0) -> TaskResult:
        """Monitors active tasks, prints progress, and handles timeout/cancellation."""
        
        cycle = 0
        while not self.navigator.isTaskComplete():
            cycle += 1
            feedback = self.navigator.getFeedback()

            if feedback and cycle % 5 == 0:
                eta = Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9
                dist = feedback.distance_remaining
                print(f'[Nav2] Distance remaining: {dist:.2f} m | ETA: {eta:.0f} s')

                elapsed = Duration.from_msg(feedback.navigation_time).nanoseconds / 1e9
                if elapsed > max_duration_sec:
                    print(f'[Nav2] Task exceeded limit of {max_duration_sec}s. Aborting...')
                    self.abort_mission()

        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.publish_status("SUCCEEDED")
        elif result == TaskResult.CANCELED:
            self.publish_status("CANCELED")
        else:
            self.publish_status("FAILED")

        


    def handle_nav_to_pose(self, msg: PoseStamped):
        """Commands the robot to navigate to a single goal pose."""
        self.navigator.waitUntilNav2Active()
        self.navigator.goToPose(msg)
        self.monitor_task()


    def handle_follow_waypoints(self, msg: PoseArray):
        """Commands the robot to follow a series of waypoints."""
        self.navigator.waitUntilNav2Active()
        
        # Convert individual Pose objects in PoseArray to PoseStamped objects
        waypoints = []
        for pose in msg.poses:
            pose_stamped = PoseStamped()
            pose_stamped.header = msg.header
            pose_stamped.pose = pose
            waypoints.append(pose_stamped)
            
        self.navigator.followWaypoints(waypoints)
        self.monitor_task()
    
    def handle_abort(self, msg: Empty):
        self.abort_mission()

    def abort_mission(self):
        """Immediately cancels any active navigation task."""
        print('[Nav2] Force aborting current navigation mission...')
        self.navigator.cancelTask()
        
    def publish_status(self, status_msg: str):
        msg = String()
        msg.data = status_msg
        self.status_pub.publish(msg)


def main():
    rclpy.init()
    commander = CommanderNode("commande_node")
    executor = SingleThreadedExecutor()
    executor.add_node(commander)

    try:
        executor.spin() 
    except KeyboardInterrupt:
        commander.abort_mission()

    finally:
        executor.shutdown()
        commander.destroy_node()
        commander.navigator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()