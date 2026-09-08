"""Exercise physical wrist constraints with real navigation/Joy and an ideal robot."""

import copy
import math
import os
from pathlib import Path
import signal
import subprocess
import time

import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from catchrobo2026_msgs.action import ExecuteSequence, FollowRoute
from catchrobo2026_msgs.srv import PlanRotationGroup, WristControl
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray
import yaml

from test_route_handoff import Harness, waypoint_pose, xyz


@pytest.fixture
def rotation_rig(tmp_path, monkeypatch):
    monkeypatch.setenv('ROS_DOMAIN_ID', '231')
    monkeypatch.setenv('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST')
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    context = rclpy.context.Context()
    rclpy.init(context=context)
    node = rclpy.create_node('observer', namespace=f'/rotation_nav_{os.getpid()}', context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    children = []
    log_path = tmp_path / 'nodes.log'
    log = log_path.open('w')
    try:
        rig = Harness(node, children, log, executor)
        rig.joints, rig.commands = [], []
        rig.subscriptions.extend([
            node.create_subscription(Float32MultiArray, 'current_joints', rig.joints.append, 100),
            node.create_subscription(Float32MultiArray, 'target_joint_angles', rig.commands.append, 100),
        ])
        rig.pose_pub = node.create_publisher(PoseStamped, 'target_pose', 10)
        rig.plan = node.create_client(PlanRotationGroup, 'plan_rotation_group')
        rig.wrist = node.create_client(WristControl, 'wrist_control')
        rig.launch('nav_director', 'path_generator_3d')
        rig.launch('nav_director', 'path_follower_node', remaps=('route:=follower_route',))
        rig.dummy = rig.launch('nav_director', 'dummy_robot_node')
        rig.launch('catchrobo2026_hand_operated', 'joy_controller_node')
        assert rig.plan.wait_for_service(timeout_sec=15)
        assert rig.wrist.wait_for_service(timeout_sec=15)
        assert rig.follow.wait_for_server(timeout_sec=15)
        rig.until(lambda: len(rig.joints) >= 10 and len(rig.poses) >= 10 and
                  rig.pose_pub.get_subscription_count() > 0)
        rig.observe()
        yield rig
    finally:
        for child in children:
            if child.poll() is None:
                os.killpg(child.pid, signal.SIGCONT)
                os.killpg(child.pid, signal.SIGINT)
        for child in children:
            try:
                child.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid, signal.SIGKILL)
                child.wait()
        executor.shutdown()
        node.destroy_node()
        context.shutdown()
        log.close()
        print(log_path.read_text())


def wrist_pose(wrist, position=(675, 200, 200)):
    base = math.atan2(position[0] - 675, position[1] + 190)
    return waypoint_pose(position, base + wrist)


def set_start(rig, wrist):
    pose = PoseStamped(pose=wrist_pose(wrist))
    pose.header.frame_id = 'map'
    for _ in range(3):
        rig.pose_pub.publish(pose)
        rig.observe(0.1)
    rig.until(lambda: abs(rig.joints[-1].data[3] - wrist) < 1e-5 and
              math.dist(xyz(rig.poses[-1]), (675, 200, 200)) < 0.1)
    rig.observe(0.1)


def plan(rig, targets, ends):
    return rig.resolve(rig.plan.call_async(PlanRotationGroup.Request(
        targets=targets, route_ends=ends)))


def begin(rig, response, group_id):
    first = response.routes[0]
    result = rig.resolve(rig.wrist.call_async(WristControl.Request(
        operation=WristControl.Request.BEGIN, group_id=group_id,
        direction=response.direction, wrist_angle=first.wrist_angles[0],
        target=first.path.poses[0])))
    assert result.success, result.message


def end(rig, group_id):
    result = rig.resolve(rig.wrist.call_async(WristControl.Request(
        operation=WristControl.Request.END, group_id=group_id)))
    assert result.success, result.message


def follow(rig, response, route_index, group_id):
    route = response.routes[route_index]
    result = rig.resolve(rig.follow.send_goal_async(FollowRoute.Goal(
        start=True, path=route.path, rotation_group_id=group_id,
        wrist_direction=response.direction, wrist_angles=route.wrist_angles)))
    assert result.accepted
    return result


@pytest.mark.parametrize('direction', [1, -1])
def test_auto_direction_across_routes_and_yaw_wrap(rotation_rig, direction):
    rig = rotation_rig
    set_start(rig, -3.0)
    targets = [wrist_pose(-3.0 + direction * 0.3, (800, 200, 240)),
               wrist_pose(-3.0 + direction * 0.7, (900, 250, 260))]
    response = plan(rig, targets, [1, 2])
    assert response.success, response.message
    assert response.direction == direction and len(response.routes) == 2
    assert all(len(route.path.poses) == len(route.wrist_angles) for route in response.routes)
    all_angles = [angle for route in response.routes for angle in route.wrist_angles]
    assert min(direction * (b - a) for a, b in zip(all_angles, all_angles[1:])) >= -1e-6
    begin(rig, response, 1)
    command_start, topic_start = len(rig.commands), len(rig.targets)
    for index in range(2):
        handle = follow(rig, response, index, 1)
        assert rig.resolve(handle.get_result_async()).result.success
    rig.observe(0.1)
    commands = [message.data[3] for message in rig.commands[command_start:]]
    assert len(commands) > 10
    changes = [direction * (b - a) for a, b in zip(commands, commands[1:])]
    assert min(changes) >= -1e-5
    assert max(changes) <= 0.051
    assert abs(rig.joints[-1].data[3] - (-3.0 + direction * 0.7)) < 0.05
    assert math.dist(xyz(rig.poses[-1]), (900, 250, 260)) < 20.1
    assert len(rig.targets) == topic_start
    end(rig, 1)


def test_pure_full_turn_requires_nonwrapped_arrival_and_holds(rotation_rig):
    rig = rotation_rig
    set_start(rig, 0.0)
    response = plan(rig, [wrist_pose(-math.pi), wrist_pose(0.0)], [2])
    assert response.success, response.message
    assert response.direction == -1
    assert abs(response.routes[0].wrist_angles[-1] + 2 * math.pi) < 1e-6
    begin(rig, response, 1)
    handle = follow(rig, response, 0, 1)
    result = handle.get_result_async()
    rig.observe(0.3)
    assert not result.done()
    assert -0.5 < rig.joints[-1].data[3] < -0.05
    assert rig.resolve(result).result.success
    rig.observe(0.1)
    assert abs(rig.joints[-1].data[3] + 2 * math.pi) < 0.05
    end(rig, 1)
    command_start = len(rig.commands)
    rig.observe(0.4)
    assert all(abs(command.data[3] + 2 * math.pi) < 1e-5
               for command in rig.commands[command_start:])
    late = rig.resolve(rig.wrist.call_async(WristControl.Request(
        operation=WristControl.Request.TARGET, group_id=1, direction=-1,
        wrist_angle=0.0, target=PoseStamped(pose=wrist_pose(0.0)))))
    assert not late.success


def test_cancel_releases_and_rejects_late_targets(rotation_rig):
    rig = rotation_rig
    set_start(rig, -0.5)
    response = plan(rig, [wrist_pose(-5.5)], [1])
    assert response.success and response.direction == -1
    begin(rig, response, 1)
    handle = follow(rig, response, 0, 1)
    result = handle.get_result_async()
    rig.observe(0.25)
    assert not result.done()
    canceled = rig.resolve(handle.cancel_goal_async())
    assert canceled.goals_canceling
    assert not rig.resolve(result).result.success
    end(rig, 1)
    rig.observe(0.1)
    held = rig.commands[-1].data[3]
    assert -1.1 < held < -0.5
    command_start = len(rig.commands)
    rig.observe(0.4)
    assert all(abs(command.data[3] - held) < 1e-6 for command in rig.commands[command_start:])
    late = rig.resolve(rig.wrist.call_async(WristControl.Request(
        operation=WristControl.Request.TARGET, group_id=1, direction=-1,
        wrist_angle=-5.5, target=PoseStamped(pose=wrist_pose(-5.5)))))
    assert not late.success
    next_plan = plan(rig, [wrist_pose(-0.1)], [1])
    assert next_plan.success and next_plan.direction == 1
    begin(rig, next_plan, 2)
    next_handle = follow(rig, next_plan, 0, 2)
    assert rig.resolve(next_handle.get_result_async()).result.success
    end(rig, 2)


def test_invalid_and_stale_plans_publish_nothing(rotation_rig):
    rig = rotation_rig
    set_start(rig, -3.0)
    command_start, route_start = len(rig.commands), len(rig.routes)
    invalid_targets = [
        ([wrist_pose(-2.0), wrist_pose(-4.0)], [2]),
        ([wrist_pose(-2.0)], [0]),
        ([wrist_pose(-2.0)], [2]),
        ([wrist_pose(-2.0, (5000, 200, 200))], [1]),
    ]
    nonfinite = wrist_pose(-2.0)
    nonfinite.position.x = float('nan')
    invalid_targets.append(([nonfinite], [1]))
    for targets, ends in invalid_targets:
        response = plan(rig, targets, ends)
        assert not response.success and not response.routes
    rig.observe(0.1)
    assert len(rig.routes) == route_start
    assert all(abs(command.data[3] + 3.0) < 1e-5
               for command in rig.commands[command_start:])
    os.killpg(rig.dummy.pid, signal.SIGSTOP)
    try:
        rig.observe(1.2)
        response = plan(rig, [wrist_pose(-2.0)], [1])
        assert not response.success and not response.routes
        assert 'Fresh' in response.message
    finally:
        os.killpg(rig.dummy.pid, signal.SIGCONT)


def test_follow_rejects_incomplete_or_reversing_constraints(rotation_rig):
    rig = rotation_rig
    set_start(rig, -3.0)
    response = plan(rig, [wrist_pose(-2.0)], [1])
    assert response.success
    route = response.routes[0]
    valid = FollowRoute.Goal(start=True, path=route.path, rotation_group_id=1,
                             wrist_direction=1, wrist_angles=route.wrist_angles)
    invalid = []
    for field, value in [('rotation_group_id', 0), ('wrist_direction', 0),
                         ('wrist_angles', []), ('path', type(route.path)())]:
        goal = copy.deepcopy(valid)
        setattr(goal, field, value)
        invalid.append(goal)
    for angle in [float('nan'), 0.1, -7.0, -4.0]:
        goal = copy.deepcopy(valid)
        goal.wrist_angles[-1] = angle
        invalid.append(goal)
    for goal in invalid:
        handle = rig.resolve(rig.follow.send_goal_async(goal))
        assert not handle.accepted


def test_sequence_yaml_group_through_real_navigation(rotation_rig, tmp_path):
    rig = rotation_rig
    set_start(rig, -3.0)

    def target(wrist, position):
        base = math.atan2(position[0] - 675, position[1] + 190)
        return [*position, base + wrist]

    waypoint = target(-2.9, (725, 200, 220))
    inline = target(-2.8, (750, 200, 230))
    first_move = target(-2.7, (775, 200, 240))
    final_move = target(-2.2, (900, 250, 260))
    config = yaml.load((Path(__file__).parents[1] / 'config/sequences.yaml').read_text(),
                       Loader=yaml.BaseLoader)
    config['sequences']['ending'] = {'steps': [
        {'rotation_group': 'start'},
        {'waypoint': {'absolute': waypoint}},
        {'move': {'absolute': first_move, 'waypoints': [inline]}},
        {'wait': 0.2},
        {'move': {'absolute': final_move}},
        {'rotation_group': 'end'},
    ]}
    config['end_sequence'] = 'ending'
    config['route_timeout_sec'] = 12.0
    config_file = tmp_path / 'sequence_rotation_group.yaml'
    config_file.write_text(yaml.safe_dump(config))
    rig.launch('catchrobo2026_sequence', 'sequence_node', {
        'team': 'red', 'sequence_file': config_file,
    })
    assert rig.sequence.wait_for_server(timeout_sec=15)
    command_start, topic_start = len(rig.commands), len(rig.targets)
    rig.execute(ExecuteSequence.Goal.END)

    commands = [command.data[3] for command in rig.commands[command_start:]]
    assert len(commands) > 20
    assert min(b - a for a, b in zip(commands, commands[1:])) >= -1e-5
    assert max(b - a for a, b in zip(commands, commands[1:])) <= 0.051
    assert abs(rig.joints[-1].data[3] + 2.2) < 0.05
    assert math.dist(xyz(rig.poses[-1]), final_move[:3]) < 20.1
    assert len(rig.targets) == topic_start
    command_start = len(rig.commands)
    rig.observe(0.4)
    assert all(abs(command.data[3] + 2.2) < 1e-5
               for command in rig.commands[command_start:])

    # A new BEGIN proves SequenceNode released its group before reporting success.
    next_id = time.time_ns() // 1000 + 1000
    released = rig.resolve(rig.wrist.call_async(WristControl.Request(
        operation=WristControl.Request.BEGIN, group_id=next_id, direction=1,
        wrist_angle=rig.joints[-1].data[3], target=rig.poses[-1])))
    assert released.success, released.message
    end(rig, next_id)
