#!/usr/bin/env python3
"""Gripper simulation shim.

Bridges franka_gripper action servers (Move, Grasp) and the Stop service
to the panda_gripper_controller's GripperCommand action in MuJoCo sim.

Coordinate conversion
---------------------
Franka gripper width  = total distance between both fingers  (0 .. 0.08 m)
GripperActionController controls a single finger (finger_joint1, 0 .. 0.04 m)
    target_position = width / 2.0
    reported_width  = position * 2.0
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from action_msgs.msg import GoalStatus

from franka_msgs.action import Grasp, Move
from control_msgs.action import GripperCommand
from std_srvs.srv import Trigger


class GripperSimShim(Node):
    """Shim node that translates franka_gripper goals into GripperCommand goals."""

    def __init__(self):
        super().__init__('gripper_sim_shim')

        # Reentrant group so action callbacks can await the downstream client
        self._cb_group = ReentrantCallbackGroup()

        # --- Action client to panda_gripper_controller ---
        self._gripper_client = ActionClient(
            self,
            GripperCommand,
            '/panda_gripper_controller/gripper_cmd',
            callback_group=self._cb_group,
        )

        # Track the latest goal handle sent to panda_gripper_controller
        self._active_goal_handle = None

        # --- franka_gripper action servers ---
        self._move_server = ActionServer(
            self,
            Move,
            '/franka_gripper/move',
            self._execute_move,
            callback_group=self._cb_group,
        )

        self._grasp_server = ActionServer(
            self,
            Grasp,
            '/franka_gripper/grasp',
            self._execute_grasp,
            callback_group=self._cb_group,
        )

        # --- Stop service ---
        self._stop_srv = self.create_service(
            Trigger,
            '/franka_gripper/stop',
            self._handle_stop,
            callback_group=self._cb_group,
        )

        self.get_logger().info('Gripper Sim Shim is ready.')

    # ------------------------------------------------------------------
    # Move action
    # ------------------------------------------------------------------
    def _execute_move(self, goal_handle):
        """Execute a franka_gripper/Move goal."""
        request = goal_handle.request
        self.get_logger().info(
            f'Move goal received: width={request.width:.4f}, speed={request.speed:.4f}'
        )

        # Build GripperCommand goal
        cmd = GripperCommand.Goal()
        cmd.command.position = request.width / 2.0
        cmd.command.max_effort = 50.0  # default effort for moves

        # Wait for downstream server
        if not self._gripper_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('panda_gripper_controller action server not available')
            goal_handle.abort()
            return Move.Result(success=False)

        # Send goal with feedback callback
        send_future = self._gripper_client.send_goal_async(
            cmd, feedback_callback=lambda fb: self._forward_move_feedback(goal_handle, fb)
        )
        rclpy.spin_until_future_complete(self, send_future)
        downstream_handle = send_future.result()

        if not downstream_handle.accepted:
            self.get_logger().warn('GripperCommand goal rejected by controller')
            goal_handle.abort()
            return Move.Result(success=False)

        self._active_goal_handle = downstream_handle

        # Wait for result
        result_future = downstream_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        self._active_goal_handle = None

        goal_handle.succeed()
        return Move.Result(success=True)

    def _forward_move_feedback(self, move_handle, feedback_msg):
        """Forward GripperCommand feedback as Move feedback."""
        fb = Move.Feedback()
        fb.current_width = feedback_msg.feedback.position * 2.0
        move_handle.publish_feedback(fb)

    # ------------------------------------------------------------------
    # Grasp action
    # ------------------------------------------------------------------
    def _execute_grasp(self, goal_handle):
        """Execute a franka_gripper/Grasp goal."""
        request = goal_handle.request
        self.get_logger().info(
            f'Grasp goal received: width={request.width:.4f}, '
            f'speed={request.speed:.4f}, force={request.force:.2f}, '
            f'epsilon=[{request.epsilon.inner:.4f}, {request.epsilon.outer:.4f}]'
        )

        # Build GripperCommand goal
        cmd = GripperCommand.Goal()
        cmd.command.position = request.width / 2.0
        cmd.command.max_effort = request.force

        if not self._gripper_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('panda_gripper_controller action server not available')
            goal_handle.abort()
            return Grasp.Result(success=False)

        send_future = self._gripper_client.send_goal_async(
            cmd, feedback_callback=lambda fb: self._forward_grasp_feedback(goal_handle, fb)
        )
        rclpy.spin_until_future_complete(self, send_future)
        downstream_handle = send_future.result()

        if not downstream_handle.accepted:
            self.get_logger().warn('GripperCommand goal rejected by controller')
            goal_handle.abort()
            return Grasp.Result(success=False)

        self._active_goal_handle = downstream_handle

        # Wait for result
        result_future = downstream_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        self._active_goal_handle = None

        downstream_result = result_future.result()
        final_position = downstream_result.result.position  # single-finger position
        actual_width = final_position * 2.0

        # Check tolerance
        lower = request.width - request.epsilon.inner
        upper = request.width + request.epsilon.outer
        within_tolerance = lower <= actual_width <= upper

        self.get_logger().info(
            f'Grasp finished: actual_width={actual_width:.4f}, '
            f'tolerance=[{lower:.4f}, {upper:.4f}], success={within_tolerance}'
        )

        goal_handle.succeed()
        return Grasp.Result(success=within_tolerance)

    def _forward_grasp_feedback(self, grasp_handle, feedback_msg):
        """Forward GripperCommand feedback as Grasp feedback."""
        fb = Grasp.Feedback()
        fb.current_width = feedback_msg.feedback.position * 2.0
        grasp_handle.publish_feedback(fb)

    # ------------------------------------------------------------------
    # Stop service
    # ------------------------------------------------------------------
    def _handle_stop(self, _request, response):
        """Cancel any active goal on panda_gripper_controller."""
        self.get_logger().info('Stop requested')
        if self._active_goal_handle is not None:
            self.get_logger().info('Cancelling active gripper goal')
            cancel_future = self._active_goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, cancel_future)
        response.success = True
        response.message = 'Stop executed'
        return response


def main(args=None):
    rclpy.init(args=args)
    shim = GripperSimShim()
    rclpy.spin(shim)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
