"""The WareGVBridgeNode: combines map and nav mixins with ROS setup."""
import os
from datetime import datetime, timezone
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav2_msgs.action import NavigateToPose, FollowWaypoints
from std_msgs.msg import String
from std_srvs.srv import Empty

from tf2_ros import Buffer, TransformListener

from .ros_node_maps import MapHelpersMixin
from .ros_node_nav import NavHelpersMixin


class WareGVBridgeNode(MapHelpersMixin, NavHelpersMixin, Node):
    """
    REST <-> ROS 2 bridge for the WareGV rover.

    AMCL is NOT used as the robot pose source. Pose comes from
    TF: map -> base_link. Nav2 actions are called directly.
    """

    def __init__(self):
        super().__init__("beargv_rest_bridge")

        self.current_mode = "auto_nav"
        self.current_map = "small_warehouse"
        self.started_at = datetime.now(timezone.utc)

        self.active_nav_goal = None
        self.active_waypoint_goal = None

        self.map_directory = self._resolve_map_directory()
        self.get_logger().info(f"Using map directory: {self.map_directory}")

        self.webrtc_signaler = os.environ.get(
            "WAREGV_WEBRTC_SIGNAL_URL", ""
        ).rstrip("/")

        # Publishers
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", 10,
        )
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.profile_pub = self.create_publisher(String, "profile_setting", 10)
        self.tts_pub = self.create_publisher(
            String, "/robot_operator/speak_device", 10,
        )

        # TF (no AMCL subscription)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer, self, spin_thread=False,
        )

        # Nav2 action clients
        self.nav_to_pose_client = ActionClient(
            self, NavigateToPose, "navigate_to_pose",
        )
        self.follow_waypoints_client = ActionClient(
            self, FollowWaypoints, "follow_waypoints",
        )

        # SLAM Toolbox reset clients
        self.slam_reset_client = self.create_client(
            Empty, "/slam_toolbox/reset",
        )
        self.slam_clear_client = self.create_client(
            Empty, "/slam_toolbox/clear_changes",
        )

        self.get_logger().info("BearGV ROS 2 REST Bridge Node Initialized.")
        self.get_logger().info(
            "Pose source: TF map -> base_link (AMCL disabled)"
        )