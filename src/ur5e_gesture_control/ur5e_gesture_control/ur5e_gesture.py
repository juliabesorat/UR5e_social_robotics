"""Run YAML steps in order, or run one selected step for testing."""
import time
import yaml
import rclpy
from ur5e_gesture_control.motion_common import MotionContext
from ur5e_gesture_control import ur5e_movej, ur5e_movel, ur5e_planned

MOVES = {'moveJ': ur5e_movej.move, 'moveL': ur5e_movel.move, 'planned': ur5e_planned.move}


def main_for_mode(required_mode=None):
    rclpy.init()
    # A small parameter node loads the YAML before creating the motion context.
    params = rclpy.create_node('ur5e_gesture_parameters')
    params.declare_parameter('sequence_file', '')
    params.declare_parameter('step_name', '')
    params.declare_parameter('execute', False)
    path = params.get_parameter('sequence_file').value
    selected = params.get_parameter('step_name').value
    execute = params.get_parameter('execute').value
    context = None
    try:
        if not path:
            raise ValueError('sequence_file must be an absolute YAML path')
        with open(path, encoding='utf-8') as stream:
            data = yaml.safe_load(stream)
        steps = data.get('steps', [])
        if selected:
            steps = [s for s in steps if s.get('name') == selected]
        if not steps:
            raise ValueError('No matching steps in sequence')
        if selected and len(steps) != 1:
            raise ValueError('step_name must identify exactly one step')
        if required_mode and (not selected or len(steps) != 1):
            raise ValueError('Individual motion nodes require a step_name')
        for step in steps:
            mode = step.get('motion_type')
            if mode not in MOVES:
                raise ValueError(f'Unknown motion_type: {mode}')
            if required_mode and mode != required_mode:
                raise ValueError(f'Selected step must have motion_type: {required_mode}')
        context = MotionContext(data.get('common', {}))
        start = context.state()
        for step in steps:
            name = step.get('name', '<unnamed>')
            context.get_logger().info(f"Step {name}: {step['motion_type']}")
            try:
                start = MOVES[step['motion_type']](context, step, start, execute)
                if execute:
                    version = context.joint_version
                    start = context.state(newer_than=version)
                    delay = float(step.get('sleep_after', 0.0))
                    if delay > 0:
                        time.sleep(delay)
            except Exception as exc:
                raise RuntimeError(f'Step {name} failed: {exc}') from exc
        context.get_logger().info('Gesture finished.')
    except Exception as exc:
        (context or params).get_logger().error(str(exc))
        raise SystemExit(1)
    finally:
        if context:
            context.destroy_node()
        params.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main():
    main_for_mode()
