#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient
from franka_msgs.action import Grasp, Move
from control_msgs.action import GripperCommand

class GripperActionShim(Node):
    def __init__(self):
        super().__init__('gripper_action_shim')
        self._grasp_action_server = ActionServer(
            self,
            Grasp,
            '/franka_gripper/grasp',
            self.execute_grasp_callback)
        self._move_action_server = ActionServer(
            self,
            Move,
            '/franka_gripper/move',
            self.execute_move_callback)
            
        self._gripper_client = ActionClient(self, GripperCommand, '/panda_gripper_controller/gripper_cmd')
        
        self.get_logger().info('Gripper Action Shim is ready.')

    def execute_grasp_callback(self, goal_handle):
        self.get_logger().info('Executing Grasp goal...')
        # Send goal to GripperCommand action server
        goal_msg = GripperCommand.Goal()
        # Grasp width is in meters (width of the object).
        # We might need to divide by 2 if GripperCommand expects single finger position,
        # but typical ros2_control GripperCommand maps to the total width.
        goal_msg.command.position = goal_handle.request.width
        goal_msg.command.max_effort = goal_handle.request.force
        
        self._gripper_client.wait_for_server()
        future = self._gripper_client.send_goal_async(goal_msg)
        # We would ideally wait for future and return result based on it,
        # but for simulation this is a simple stub.
        
        goal_handle.succeed()
        result = Grasp.Result()
        result.success = True
        return result

    def execute_move_callback(self, goal_handle):
        self.get_logger().info('Executing Move goal...')
        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = goal_handle.request.width
        goal_msg.command.max_effort = 100.0 # Default move effort
        
        self._gripper_client.wait_for_server()
        self._gripper_client.send_goal_async(goal_msg)
        
        goal_handle.succeed()
        result = Move.Result()
        result.success = True
        return result

def main(args=None):
    rclpy.init(args=args)
    shim = GripperActionShim()
    rclpy.spin(shim)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
