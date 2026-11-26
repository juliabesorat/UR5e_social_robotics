#!/usr/bin/env python3
"""Plan a YAML pose sequence with MoveIt, independently of the sequence server."""
import copy
import math
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Pose, PoseStamped
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from moveit_msgs.msg import Constraints, DisplayTrajectory, OrientationConstraint, PositionConstraint
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs
import yaml

from ur5e_robot_controller.ur5e_pose_sequence import quat_from_rpy_zyx, UR5E_JOINTS


class PlannedSequence(Node):
    def __init__(self):
        super().__init__('ur5e_pose_sequence_moveit')
        # Execution is controlled by the launch parameter, independently of YAML.
        self.declare_parameter('sequence_file', '')
        self.declare_parameter('execute', False)
        path = self.get_parameter('sequence_file').value
        with open(path, encoding='utf-8') as stream:
            data = yaml.safe_load(stream)
        self.common = data.get('common', {})
        self.steps = data.get('steps', [])
        if not self.steps:
            raise ValueError('The sequence must contain at least one step')
        self.execute = self.get_parameter('execute').value
        self.group = self.common.get('group_name', 'ur_manipulator')
        self.link = self.common.get('ik_link', 'tool0')
        self.target_frame = self.common.get('target_frame', 'table')
        self.planning_frame = self.common.get('planning_frame', 'base_link')
        self.velocity = float(self.common.get('max_velocity', 0.1))
        self.acceleration = float(self.common.get('max_acceleration', 0.1))
        if not (0.0 < self.velocity <= 1.0 and 0.0 < self.acceleration <= 1.0):
            raise ValueError('Velocity and acceleration factors must be in (0, 1]')
        self.joints = None
        # Get the robot state and TF; MoveIt provides planning and execution.
        self.create_subscription(JointState, '/joint_states', self._joints_cb, qos_profile_sensor_data)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.move = ActionClient(self, MoveGroup, '/move_action')
        self.execute_client = ActionClient(self, ExecuteTrajectory, '/execute_trajectory')
        self.display = self.create_publisher(DisplayTrajectory, '/display_planned_path', 10)

    def _joints_cb(self, msg):
        if all(name in msg.name for name in UR5E_JOINTS):
            self.joints = msg

    def wait_future(self, future):
        rclpy.spin_until_future_complete(self, future)
        if not future.done() or future.exception():
            raise RuntimeError('ROS request failed: ' + str(future.exception()))
        return future.result()

    def send(self, client, goal):
        # Check both goal acceptance and the final action result.
        handle = self.wait_future(client.send_goal_async(goal))
        if not handle.accepted:
            raise RuntimeError('MoveIt rejected the action goal')
        result = self.wait_future(handle.get_result_async())
        if result.status != 4 or result.result.error_code.val != 1:
            raise RuntimeError(f'MoveIt action failed: status={result.status}, error={result.result.error_code.val}')
        return result.result

    def target(self, step):
        # Convert YAML millimetres and degrees to a ROS pose in the planning frame.
        xyz, rpy = step['target_xyz'], step['target_rpy']
        if len(xyz) != 3 or len(rpy) != 3:
            raise ValueError('Each step needs three target_xyz and target_rpy values')
        pose = PoseStamped()
        pose.header.frame_id = self.target_frame
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = [float(v) / 1000.0 for v in xyz]
        q = quat_from_rpy_zyx(*[math.radians(float(v)) for v in rpy])
        pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = q
        # Process TF callbacks while waiting; a blocking lookup on this thread
        # would prevent the listener from receiving the transform it needs.
        deadline = time.monotonic() + 10.0
        while rclpy.ok() and time.monotonic() < deadline:
            if self.tf_buffer.can_transform(
                self.planning_frame, self.target_frame, rclpy.time.Time()
            ):
                break
            rclpy.spin_once(self, timeout_sec=0.1)
        else:
            raise RuntimeError(
                f'TF unavailable: {self.planning_frame} <- {self.target_frame}'
            )
        transform = self.tf_buffer.lookup_transform(
            self.planning_frame, self.target_frame, rclpy.time.Time()
        )
        pose.pose = tf2_geometry_msgs.do_transform_pose(pose.pose, transform)
        pose.header.frame_id = self.planning_frame
        return pose

    def goal(self, pose, start):
        # Define a pose goal using position and orientation tolerances.
        goal = MoveGroup.Goal()
        request = goal.request
        request.group_name = self.group
        request.num_planning_attempts = 5
        request.allowed_planning_time = 5.0
        request.max_velocity_scaling_factor = self.velocity
        request.max_acceleration_scaling_factor = self.acceleration
        request.start_state = start
        position = PositionConstraint()
        position.header.frame_id = self.planning_frame
        position.link_name = self.link
        sphere = SolidPrimitive()
        # Radius is the allowed position error in metres.
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [0.001]
        region_pose = Pose()
        region_pose.position = pose.pose.position
        region_pose.orientation.w = 1.0
        position.constraint_region.primitives.append(sphere)
        position.constraint_region.primitive_poses.append(region_pose)
        position.weight = 1.0
        orientation = OrientationConstraint()
        orientation.header.frame_id = self.planning_frame
        orientation.link_name = self.link
        orientation.orientation = pose.pose.orientation
        # Orientation tolerances are in radians around each axis.
        orientation.absolute_x_axis_tolerance = 0.01
        orientation.absolute_y_axis_tolerance = 0.01
        orientation.absolute_z_axis_tolerance = 0.01
        orientation.weight = 1.0
        constraints = Constraints()
        constraints.position_constraints.append(position)
        constraints.orientation_constraints.append(orientation)
        request.goal_constraints.append(constraints)
        goal.planning_options.plan_only = True
        # Planning and execution are separate, so preview does not move the robot.
        return goal

    def run(self):
        from moveit_msgs.msg import RobotState

        self.get_logger().info('Waiting for MoveIt and joint states...')
        if not self.move.wait_for_server(timeout_sec=30.0):
            raise RuntimeError('MoveIt /move_action is unavailable')
        if self.execute and not self.execute_client.wait_for_server(timeout_sec=30.0):
            raise RuntimeError('MoveIt /execute_trajectory is unavailable')
        deadline = time.monotonic() + 30.0
        while rclpy.ok() and self.joints is None and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.joints is None:
            raise RuntimeError('No complete UR5e joint state received')
        start = RobotState()
        start.joint_state = copy.deepcopy(self.joints)
        display = DisplayTrajectory()
        display.trajectory_start = copy.deepcopy(start)
        for index, step in enumerate(self.steps):
            name = step.get('name', str(index + 1))
            self.get_logger().info(f'Planning step {index + 1}: {name}')
            if self.execute:
                # Use the measured joint state for each plan during execution.
                rclpy.spin_once(self, timeout_sec=0.1)
                start.joint_state = copy.deepcopy(self.joints)
            result = self.send(self.move, self.goal(self.target(step), start))
            trajectory = result.planned_trajectory
            jt = trajectory.joint_trajectory
            if not jt.points:
                raise RuntimeError(f'Empty trajectory for {name}')
            display.trajectory.append(trajectory)
            # Publish the accumulated sequence for RViz before optional execution.
            self.display.publish(display)
            if self.execute:
                execute_goal = ExecuteTrajectory.Goal()
                execute_goal.trajectory = trajectory
                self.send(self.execute_client, execute_goal)
                delay = float(step.get('sleep_after', 0.0))
                if delay > 0:
                    time.sleep(delay)
            else:
                # Preview the next step from the previous planned end state.
                positions = dict(zip(start.joint_state.name, start.joint_state.position))
                positions.update(zip(jt.joint_names, jt.points[-1].positions))
                start.joint_state.position = [positions[n] for n in start.joint_state.name]
            self.get_logger().info(f'Completed step: {name}')
        self.get_logger().info('Sequence finished.')


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = PlannedSequence()
        node.run()
    except Exception as exc:
        if node:
            node.get_logger().error(str(exc))
        else:
            print(f'Sequence failed: {exc}')
        raise SystemExit(1)
    finally:
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
