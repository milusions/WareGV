#!/usr/bin/env python3

import os
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rcl_interfaces.msg import SetParametersResult, Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from nav2_msgs.srv import LoadMap
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf2_ros import Buffer, TransformListener, TransformException
from slam_toolbox.srv import SaveMap

class AutoModeSwitcher(Node):
    def __init__(self):
        super().__init__('auto_mode_switcher')

        self.cb_group = ReentrantCallbackGroup()
        self.declare_parameter('mode', 'none')
        self.declare_parameter('map_save_dir', '/tmp/waregv_maps')
        self.map_dir = self.get_parameter('map_save_dir').value
        os.makedirs(self.map_dir, exist_ok=True)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Clients configured with ReentrantCallbackGroup
        self.mode_param_client = self.create_client(
            SetParameters, '/mode_manager/set_parameters', callback_group=self.cb_group
        )
        self.save_map_client = self.create_client(
            SaveMap, '/slam_toolbox/save_map', callback_group=self.cb_group
        )
        self.load_map_client = self.create_client(
            LoadMap, '/map_server/load_map', callback_group=self.cb_group
        )

        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, 
            '/initialpose', 
            10
        )

        self.add_on_set_parameters_callback(self.on_param_change)
        self.get_logger().info("Auto Mode Switcher ready. Switch mode with: ros2 param set /auto_mode_switcher mode <mode_name>")

    def on_param_change(self, params):
        for param in params:
            if param.name == 'mode':
                target_mode = str(param.value).strip().lower()
                self.get_logger().info(f"Scheduling auto switch task to '{target_mode}'")
                # Asynchronously dispatch task to prevent blocking the parameter callback
                self.executor.create_task(self.execute_auto_switch, target_mode)
                return SetParametersResult(successful=True, reason=f"Auto switch to '{target_mode}' scheduled")
        return SetParametersResult(successful=True)

    def get_current_pose(self):
        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(
                'map', 
                'base_footprint', 
                now, 
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            return trans
        except TransformException as ex:
            self.get_logger().warn(f"Could not lookup robot pose: {ex}")
            return None

    async def save_slam_map(self, map_path_prefix):
        if not self.save_map_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error("SLAM map saving service not available")
            return False

        req = SaveMap.Request()
        req.name.data = map_path_prefix
        try:
            await self.save_map_client.call_async(req)
            return True
        except Exception as e:
            self.get_logger().error(f"Failed to save map: {e}")
            return False

    async def load_map_to_server(self, yaml_filepath):
        if not self.load_map_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error("Map server load_map service not available")
            return False

        req = LoadMap.Request()
        req.map_url = yaml_filepath
        try:
            await self.load_map_client.call_async(req)
            return True
        except Exception as e:
            self.get_logger().error(f"Failed to load map: {e}")
            return False

    def seed_amcl_initial_pose(self, transform):
        if transform is None:
            return

        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.frame_id = 'map'
        pose_msg.header.stamp = self.get_clock().now().to_msg()

        pose_msg.pose.pose.position.x = transform.transform.translation.x
        pose_msg.pose.pose.position.y = transform.transform.translation.y
        pose_msg.pose.pose.position.z = transform.transform.translation.z
        pose_msg.pose.pose.orientation = transform.transform.rotation

        pose_msg.pose.covariance[0] = 0.25
        pose_msg.pose.covariance[7] = 0.25
        pose_msg.pose.covariance[35] = 0.068

        self.initial_pose_pub.publish(pose_msg)
        self.get_logger().info("Seeded AMCL initial pose from last SLAM transform.")

    async def execute_auto_switch(self, target_mode: str):
        self.get_logger().info(f"Executing auto switch to mode: {target_mode}")

        # Step 1: Capture transform before disabling SLAM
        last_pose_tf = self.get_current_pose()

        # Step 2: Save SLAM map if transitioning to nav2_amcl
        saved_map_yaml = os.path.join(self.map_dir, "auto_saved_map.yaml")
        if target_mode == "nav2_amcl":
            map_prefix = os.path.join(self.map_dir, "auto_saved_map")
            self.get_logger().info("Saving current SLAM map...")
            await self.save_slam_map(map_prefix)

        # Step 3: Trigger lifecycle transition via mode_manager parameter
        if not self.mode_param_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error("mode_manager parameter service unavailable")
            return

        req = SetParameters.Request()
        param = Parameter()
        param.name = "mode"
        param.value = ParameterValue(type=ParameterType.PARAMETER_STRING, string_value=target_mode)
        req.parameters = [param]

        try:
            res = await self.mode_param_client.call_async(req)
            if not res or not res.results or not res.results[0].successful:
                self.get_logger().error("Failed to update mode_manager node states")
                return
        except Exception as e:
            self.get_logger().error(f"Failed parameter call to mode_manager: {e}")
            return

        # Step 4: Post-switch map loading and pose seeding
        if target_mode == "nav2_amcl":
            await self.load_map_to_server(saved_map_yaml)
            self.seed_amcl_initial_pose(last_pose_tf)

        self.get_logger().info(f"Auto switch to '{target_mode}' completed successfully.")

def main(args=None):
    rclpy.init(args=args)
    node = AutoModeSwitcher()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()