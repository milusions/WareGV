#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.task import Future
from rcl_interfaces.msg import SetParametersResult
from lifecycle_msgs.srv import ChangeState, GetState
from lifecycle_msgs.msg import Transition, State

class ModeManager(Node):
    def __init__(self):
        super().__init__('mode_manager')

        self.cb_group = ReentrantCallbackGroup()
        self.declare_parameter('mode', 'none')

        # Define node groups
        self.slam_nodes = ['slam_toolbox']
        self.amcl_nodes = ['map_server', 'amcl']
        self.nav2_nodes = ['controller_server', 'planner_server', 'behavior_server', 'bt_navigator']

        self.all_nodes = list(set(self.slam_nodes + self.amcl_nodes + self.nav2_nodes))

        # Mode Map
        self.mode_map = {
            'slam_only': self.slam_nodes,
            'nav2_amcl': self.amcl_nodes + self.nav2_nodes,
            'slam_nav2': self.slam_nodes + self.nav2_nodes,
        }

        self.current_mode = 'none'

        # Parameter change callback
        self.add_on_set_parameters_callback(self.on_param_change)

        # Trigger initial mode transition after executor starts spinning
        initial_mode = self.get_parameter('mode').value
        if initial_mode in self.mode_map:
            self.startup_timer = self.create_timer(
                0.5, self._start_initial_mode, callback_group=self.cb_group
            )

        self.get_logger().info("Mode Manager Initialized. Change mode via: ros2 param set /mode_manager mode <mode_name>")

    async def ros_sleep(self, seconds: float):
        """ROS2-native async sleep compatible with rclpy MultiThreadedExecutor."""
        future = Future()
        timer = self.create_timer(
            seconds, 
            lambda: (future.set_result(True), timer.destroy()), 
            callback_group=self.cb_group
        )
        await future

    async def _start_initial_mode(self):
        self.startup_timer.cancel()
        initial_mode = self.get_parameter('mode').value
        await self.switch_to_mode(initial_mode)

    def on_param_change(self, params):
        for param in params:
            if param.name == 'mode':
                target_mode = str(param.value).strip().lower()
                if target_mode == self.current_mode:
                    return SetParametersResult(successful=True, reason=f"Already in mode '{target_mode}'")
                
                if target_mode in self.mode_map:
                    # Schedule async transition task without blocking parameter server
                    self.get_logger().info(f"Scheduling transition to mode '{target_mode}'")
                    self.executor.create_task(self.switch_to_mode, target_mode)
                    return SetParametersResult(successful=True, reason=f"Transition to '{target_mode}' requested")
                else:
                    return SetParametersResult(
                        successful=False, 
                        reason=f"Unknown mode '{target_mode}'. Valid options: {list(self.mode_map.keys())}"
                    )
        return SetParametersResult(successful=True)

    async def get_node_state(self, node_name: str) -> int:
        client = self.create_client(
            GetState, f'/{node_name}/get_state', callback_group=self.cb_group
        )
        if not client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error(f"Service /{node_name}/get_state not available.")
            return State.PRIMARY_STATE_UNKNOWN

        request = GetState.Request()
        try:
            response = await client.call_async(request)
            return response.current_state.id
        except Exception as e:
            self.get_logger().error(f"Failed to get state for {node_name}: {e}")
            return State.PRIMARY_STATE_UNKNOWN

    async def change_node_state(self, node_name: str, transition_id: int) -> bool:
        client = self.create_client(
            ChangeState, f'/{node_name}/change_state', callback_group=self.cb_group
        )
        if not client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error(f"Service /{node_name}/change_state not available.")
            return False

        request = ChangeState.Request()
        request.transition.id = transition_id
        try:
            response = await client.call_async(request)
            return response.success
        except Exception as e:
            self.get_logger().error(f"Failed to transition state for {node_name}: {e}")
            return False

    async def transition_node_to(self, node_name: str, target_state: str) -> bool:
        current_state_id = await self.get_node_state(node_name)

        if target_state == 'active':
            if current_state_id == State.PRIMARY_STATE_ACTIVE:
                return True
            if current_state_id == State.PRIMARY_STATE_UNCONFIGURED:
                self.get_logger().info(f"Configuring node: {node_name}")
                await self.change_node_state(node_name, Transition.TRANSITION_CONFIGURE)
                current_state_id = State.PRIMARY_STATE_INACTIVE
            if current_state_id == State.PRIMARY_STATE_INACTIVE:
                self.get_logger().info(f"Activating node: {node_name}")
                return await self.change_node_state(node_name, Transition.TRANSITION_ACTIVATE)

        elif target_state == 'unconfigured':
            if current_state_id == State.PRIMARY_STATE_UNCONFIGURED:
                return True
            if current_state_id == State.PRIMARY_STATE_ACTIVE:
                self.get_logger().info(f"Deactivating node: {node_name}")
                await self.change_node_state(node_name, Transition.TRANSITION_DEACTIVATE)
                current_state_id = State.PRIMARY_STATE_INACTIVE
            if current_state_id == State.PRIMARY_STATE_INACTIVE:
                self.get_logger().info(f"Cleaning up node: {node_name}")
                return await self.change_node_state(node_name, Transition.TRANSITION_CLEANUP)

        return False

    async def switch_to_mode(self, mode_name: str) -> bool:
        if mode_name not in self.mode_map:
            self.get_logger().error(f"Unknown mode '{mode_name}'. Valid options: {list(self.mode_map.keys())}")
            return False

        self.get_logger().info(f"--- Transitioning from '{self.current_mode}' to '{mode_name}' ---")
        target_active_nodes = self.mode_map[mode_name]

        # Step 1: Deactivate and unconfigure all nodes NOT needed in the new mode
        for node in self.all_nodes:
            if node not in target_active_nodes:
                await self.transition_node_to(node, 'unconfigured')

        # Step 2: Activate localization/mapping provider first (slam_toolbox or amcl/map_server)
        map_providers = [n for n in ['slam_toolbox', 'map_server', 'amcl'] if n in target_active_nodes]
        for node in map_providers:
            await self.transition_node_to(node, 'active')

        # Step 3: Wait briefly via ROS2 timer so TF frame 'map' becomes active
        if map_providers:
            self.get_logger().info("Waiting 2.0s for TF map frame to settle...")
            await self.ros_sleep(2.0)

        # Step 4: Configure and activate remaining Nav2 nodes
        for node in target_active_nodes:
            if node not in map_providers:
                await self.transition_node_to(node, 'active')

        self.current_mode = mode_name
        self.get_logger().info(f"Successfully switched to mode: '{self.current_mode}'")
        return True

def main(args=None):
    rclpy.init(args=args)
    node = ModeManager()
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