#!/usr/bin/env python3
"""Add one temporary test table to the shared MoveIt planning scene."""
import signal

import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene, PlanningSceneComponents
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from shape_msgs.msg import SolidPrimitive


TABLE_ID = 'ur5e_test_scene_table'


class TestScene(Node):
    def __init__(self):
        super().__init__('ur5e_test_scene')
        self.declare_parameter('remove_only', False)
        self.apply_client = self.create_client(ApplyPlanningScene, '/apply_planning_scene')
        self.get_client = self.create_client(GetPlanningScene, '/get_planning_scene')

    def call(self, client, request):
        if not client.wait_for_service(timeout_sec=10.0):
            raise RuntimeError(f'Service unavailable: {client.srv_name}')
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        if not future.done():
            raise RuntimeError(f'Service timed out: {client.srv_name}')
        if future.exception() is not None:
            raise RuntimeError(f'Service failed: {future.exception()}')
        return future.result()

    def table_exists(self):
        request = GetPlanningScene.Request()
        request.components.components = PlanningSceneComponents.WORLD_OBJECT_NAMES
        response = self.call(self.get_client, request)
        return any(obj.id == TABLE_ID for obj in response.scene.world.collision_objects)

    def apply_object(self, collision_object):
        # Apply a scene diff so existing objects and robot settings stay intact.
        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects.append(collision_object)
        request = ApplyPlanningScene.Request()
        request.scene = scene
        if not self.call(self.apply_client, request).success:
            raise RuntimeError('MoveIt did not accept the planning scene update')

    def add_table(self):
        # base_link z=0 is assumed to be the upper surface of the table.
        table = CollisionObject()
        table.header.frame_id = 'base_link'
        table.id = TABLE_ID
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [1.0, 1.0, 0.05]
        pose = Pose()
        pose.position.z = -0.025
        pose.orientation.w = 1.0
        table.primitives.append(box)
        table.primitive_poses.append(pose)
        table.operation = CollisionObject.ADD
        self.apply_object(table)

    def remove_table(self):
        table = CollisionObject()
        table.id = TABLE_ID
        table.header.frame_id = 'base_link'
        table.operation = CollisionObject.REMOVE
        self.apply_object(table)


def main(args=None):
    # Keep ROS alive on Ctrl+C so the removal request can complete.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    stopped = False

    def stop(_signum, _frame):
        nonlocal stopped
        stopped = True

    previous_int = signal.signal(signal.SIGINT, stop)
    previous_term = signal.signal(signal.SIGTERM, stop)
    node = None
    owns_table = False
    failed = False
    try:
        node = TestScene()
        if node.get_parameter('remove_only').value:
            node.remove_table()
            node.get_logger().info(f'Removed {TABLE_ID}, if present.')
        elif node.table_exists():
            raise RuntimeError(
                f'{TABLE_ID} already exists. Stop the other scene node or remove the stale object.'
            )
        else:
            # Mark ownership before the service call: even a timeout can mean the
            # request was applied, so cleanup must still be attempted.
            owns_table = True
            node.add_table()
            node.get_logger().info(f'Added {TABLE_ID}. Press Ctrl+C to remove it.')
            while rclpy.ok() and not stopped:
                rclpy.spin_once(node, timeout_sec=0.2)
    except Exception as exc:
        failed = True
        if node:
            node.get_logger().error(str(exc))
        else:
            print(f'Test scene failed: {exc}')
    finally:
        if node and owns_table:
            try:
                node.remove_table()
                node.get_logger().info(f'Removed {TABLE_ID}.')
            except Exception as exc:
                failed = True
                node.get_logger().error(f'Could not remove {TABLE_ID}: {exc}')
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        signal.signal(signal.SIGINT, previous_int)
        signal.signal(signal.SIGTERM, previous_term)
    if failed:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
