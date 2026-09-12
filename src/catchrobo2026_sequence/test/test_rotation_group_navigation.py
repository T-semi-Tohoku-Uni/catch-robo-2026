"""Exercise physical wrist constraints with real navigation/Joy and an ideal robot."""

import copy
import math
import os
from pathlib import Path
import signal
import subprocess
import time

from catchrobo2026_msgs.action import ExecuteSequence, FollowRoute
from catchrobo2026_msgs.srv import PlanRotationGroup, WristControl
from geometry_msgs.msg import PoseStamped
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import Float32MultiArray
from test_route_handoff import Harness, waypoint_pose, xyz
import yaml


@pytest.fixture
def rotation_rig(tmp_path, monkeypatch):
    monkeypatch.setenv('ROS_DOMAIN_ID', '231')
    monkeypatch.setenv('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST')
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    context = rclpy.context.Context()
    rclpy.init(context=context)
    namespace = f'/rotation_nav_{os.getpid()}_{time.monotonic_ns()}'
    node = rclpy.create_node('observer', namespace=namespace, context=context)
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
            node.create_subscription(
                Float32MultiArray, 'target_joint_angles', rig.commands.append, 100),
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


def plan(rig, targets, ends, **constraints):
    return rig.resolve(rig.plan.call_async(PlanRotationGroup.Request(
        targets=targets, route_ends=ends, **constraints)))


def begin(rig, response, group_id, **constraints):
    first = response.routes[0]
    result = rig.resolve(rig.wrist.call_async(WristControl.Request(
        operation=WristControl.Request.BEGIN, group_id=group_id,
        direction=response.direction, wrist_angle=first.wrist_angles[0],
        target=first.path.poses[0], **constraints)))
    assert result.success, result.message


def end(rig, group_id):
    result = rig.resolve(rig.wrist.call_async(WristControl.Request(
        operation=WristControl.Request.END, group_id=group_id)))
    assert result.success, result.message


def follow(rig, response, route_index, group_id):
    route = response.routes[route_index]
    result = rig.resolve(rig.follow.send_goal_async(FollowRoute.Goal(
        start=True, path=route.path, rotation_group_id=group_id,
        wrist_direction=response.direction, wrist_angles=route.wrist_angles,
        phi_angles=route.phi_angles)))
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
    for field, value in [('rotation_group_id', 0), ('wrist_direction', 2),
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


@pytest.mark.parametrize('team', ['red', 'blue'])
@pytest.mark.parametrize('grouped', [False, True])
def test_first_pick_then_place_with_builtin_coordinates(rotation_rig, tmp_path, team, grouped):
    from catchrobo2026_msgs.srv import EndeffectorControl, PumpControl

    rig = rotation_rig

    def mechanism_done(request, response):
        response.success = True
        return response

    services = [
        rig.node.create_service(PumpControl, 'set_pump_state', mechanism_done),
        rig.node.create_service(EndeffectorControl, 'set_endeffector_state', mechanism_done),
    ]
    config_file = Path(__file__).parents[1] / 'config/sequences.yaml'
    config = yaml.load(config_file.read_text(), Loader=yaml.BaseLoader)
    if grouped:
        place_name = config['bindings'][team]['place']['0,0']
        steps = config['sequences'][place_name]['steps']
        assert steps[-1] == {'call': f'place_pre_{team}_waypoint'}
        steps.insert(0, {'rotation_group': 'start'})
        steps.insert(len(steps) - 1, {'rotation_group': 'end'})
        config_file = tmp_path / 'first_place_rotation_group.yaml'
        config_file.write_text(yaml.safe_dump(config))
    rig.launch('catchrobo2026_sequence', 'sequence_node', {
        'team': team, 'sequence_file': config_file,
    })
    assert rig.sequence.wait_for_server(timeout_sec=15)

    place_commands = []
    for index, kind in enumerate([ExecuteSequence.Goal.PICK, ExecuteSequence.Goal.PLACE], 1):
        if kind == ExecuteSequence.Goal.PLACE:
            place_commands.append(rig.commands[-1].data[3])
            command_start = len(rig.commands)
            place_route_start = len(rig.routes)
        handle = rig.resolve(rig.sequence.send_goal_async(ExecuteSequence.Goal(
            control_epoch=1, step_id=index, kind=kind, row=0, column=1,
            box=0, box_column=0, collector_mask=7)))
        assert handle.accepted
        result = rig.resolve(handle.get_result_async())
        assert result.result.success, result.result.message
        rig.observe(0.1)
    place_commands.extend(command.data[3] for command in rig.commands[command_start:])
    assert len(place_commands) > 5
    assert min(place_commands) >= -2 * math.pi - 1e-5
    assert max(place_commands) <= 1e-5
    assert max(abs(b-a) for a, b in zip(place_commands, place_commands[1:])) < math.pi
    if grouped:
        direction = 1 if team == 'red' else -1
        assert min(direction*(b-a) for a, b in zip(place_commands, place_commands[1:])) >= -1e-5
        assert max(abs(b-a) for a, b in zip(place_commands, place_commands[1:])) <= 0.051
    else:
        def numeric(value):
            return float(config['values'][value[1:]]) if value.startswith('$') else float(value)

        place_above = [numeric(value) for value in config['poses'][f'place_{team}_0_0_above']]
        place_retreat = place_above[:3]
        place_retreat[2] += float(config['values']['place_retreat_dz'])
        via = [float(value) for value in config['sequences'][
            f'place_pre_{team}_waypoint']['steps'][0]['waypoint']['absolute']][:3]
        pick_above = [numeric(value) for value in config['poses']['pick_0_1_above']]
        pick_retreat = pick_above[:3]
        pick_retreat[2] += float(config['values']['pick_retreat_dz'])

        # PLACE finishes at its retreat target; its trailing waypoint is still deferred.
        rig.observe(0.3)
        assert len(rig.routes) == place_route_start + 3
        assert math.dist(xyz(rig.poses[-1]), place_retreat) < 20.1
        assert math.dist(xyz(rig.poses[-1]), via) > 100.0
        placed_position = xyz(rig.poses[-1])
        route_start, pose_start = len(rig.routes), len(rig.poses)
        rig.observe(0.3)
        assert len(rig.routes) == route_start
        assert math.dist(xyz(rig.poses[-1]), placed_position) < 1.0

        handle = rig.resolve(rig.sequence.send_goal_async(ExecuteSequence.Goal(
            control_epoch=1, step_id=3, kind=ExecuteSequence.Goal.PICK,
            row=0, column=1, collector_mask=7)))
        assert handle.accepted
        result = rig.resolve(handle.get_result_async())
        assert result.result.success, result.result.message
        rig.observe(0.1)
        assert len(rig.routes) == route_start + 4
        first_route = rig.routes[route_start]
        assert math.dist(xyz(first_route.poses[0]), placed_position) < 1.0
        assert math.dist(xyz(first_route.poses[-1]), pick_above[:3]) < 0.1
        assert min(math.dist(xyz(pose), via) for pose in first_route.poses) < 3.0
        assert min(math.dist(xyz(pose), via) for pose in rig.poses[pose_start:]) < 90.0
        assert math.dist(xyz(rig.poses[-1]), pick_retreat) < 20.1
    for service in services:
        rig.node.destroy_service(service)


def commanded_phi_travel(commands):
    angles = [float(command.data[0]) + float(command.data[3]) for command in commands]
    return sum(abs(b - a) for a, b in zip(angles, angles[1:]))


@pytest.mark.parametrize('one_direction', [False, True])
def test_ending_ab_rejects_full_turn_and_accepts_zero_phi_travel(rotation_rig, one_direction):
    rig = rotation_rig
    set_start(rig, 0.0)
    final = waypoint_pose((670, -110, 220), 0.0)
    before = len(rig.commands)
    rejected = plan(rig, [final], [1], allow_wrist_reversal=not one_direction,
                    limit_phi_travel=True, max_phi_travel=0.1)
    assert not rejected.success and not rejected.routes
    assert 'phi' in rejected.message.lower()
    rig.observe(0.1)
    assert commanded_phi_travel(rig.commands[before:]) < 1e-5

    # Reach the equivalent lower endpoint without a full-turn setup motion.
    set_start(rig, -2 * math.pi + 0.01)
    setup = plan(rig, [wrist_pose(0.0, (675, 200, 300))], [1])
    assert setup.success and setup.direction == -1
    begin(rig, setup, 1)
    assert rig.resolve(follow(rig, setup, 0, 1).get_result_async()).result.success
    end(rig, 1)
    rig.observe(0.1)

    response = plan(rig, [final], [1], allow_wrist_reversal=not one_direction,
                    limit_phi_travel=True, max_phi_travel=0.0)
    assert response.success, response.message
    assert response.direction == (1 if one_direction else 0) and response.phi_travel < 1e-5
    assert response.routes[0].phi_angles
    begin(rig, response, 2, limit_phi_travel=True, max_phi_travel=0.0)
    before = len(rig.commands)
    result = rig.resolve(follow(rig, response, 0, 2).get_result_async())
    assert result.result.success
    rig.observe(0.1)
    assert commanded_phi_travel(rig.commands[before:]) < 1e-4
    assert abs(rig.joints[-1].data[3] - (-6.220766497)) < 0.05
    end(rig, 2)


def test_phi_limit_counts_reversals_and_combines_with_direction(rotation_rig):
    rig = rotation_rig
    set_start(rig, -3.0)
    targets = [wrist_pose(-2.6), wrist_pose(-3.0)]
    response = plan(rig, targets, [1, 2], allow_wrist_reversal=True,
                    limit_phi_travel=True, max_phi_travel=0.5)
    assert not response.success and not response.routes
    response = plan(rig, targets, [1, 2], allow_wrist_reversal=True,
                    limit_phi_travel=True, max_phi_travel=0.81)
    assert response.success, response.message
    assert response.phi_travel == pytest.approx(0.8, abs=1e-5)
    assert response.direction == 0
    begin(rig, response, 1, limit_phi_travel=True, max_phi_travel=0.81)
    before = len(rig.commands)
    for index in range(2):
        assert rig.resolve(follow(rig, response, index, 1).get_result_async()).result.success
    rig.observe(0.1)
    commands = rig.commands[before:]
    assert commanded_phi_travel(commands) == pytest.approx(0.8, abs=0.06)
    wrists = [message.data[3] for message in commands]
    assert max(abs(b - a) for a, b in zip(wrists, wrists[1:])) <= 0.051
    end(rig, 1)
    rejected = plan(rig, targets, [1, 2], limit_phi_travel=True, max_phi_travel=1.0)
    assert not rejected.success and not rejected.routes
    accepted = plan(rig, [wrist_pose(-2.7)], [1],
                    limit_phi_travel=True, max_phi_travel=0.4)
    assert accepted.success and accepted.direction == 1
    assert accepted.phi_travel == pytest.approx(0.3, abs=0.05)


@pytest.mark.parametrize('local_budget,start_xyz', [
    (False, (375, 238, 196.95)), (True, (375, 238, 196.95)),
    (True, (1199.13, -736, 304.35)),
])
def test_sequence_group_looks_ahead_across_ending_moves(
        rotation_rig, tmp_path, local_budget, start_xyz):
    rig = rotation_rig
    start = PoseStamped(pose=waypoint_pose(start_xyz, -math.pi))
    for _ in range(3):
        rig.pose_pub.publish(start)
        rig.observe(0.1)
    rig.until(lambda: math.dist(xyz(rig.poses[-1]), start_xyz) < 0.1)
    rig.observe(0.1)
    config = yaml.load((Path(__file__).parents[1] / 'config/sequences.yaml').read_text(),
                       Loader=yaml.BaseLoader)
    config['sequences']['ending'] = {'steps': [
        {'sequence_group': {'start': True, 'max_phi_travel': math.pi + 0.01}},
        {'move': {'absolute': [675, 200, 300, 0]}}, {'wait': 0.05},
        {'move': {'absolute': 'lifecycle_pose'}}, {'sequence_group': 'end'},
    ]}
    if local_budget:
        config['sequences']['ending']['steps'] = [
            {'sequence_group': 'start'},
            {'move': {'absolute': [675, 200, 300, 0]}},
            {'phi_travel': {'start': True, 'limit': math.pi / 6}}, {'wait': 0.05},
            {'move': {'absolute': 'lifecycle_pose'}}, {'phi_travel': 'end'},
            {'sequence_group': 'end'},
        ]
    config['end_sequence'] = 'ending'
    config['route_timeout_sec'] = 12.0
    config_file = tmp_path / 'sequence_phi_limit.yaml'
    config_file.write_text(yaml.safe_dump(config))
    rig.launch('catchrobo2026_sequence', 'sequence_node', {
        'team': 'red', 'sequence_file': config_file,
    })
    assert rig.sequence.wait_for_server(timeout_sec=15)
    before = len(rig.commands)
    rig.execute(ExecuteSequence.Goal.END)
    rig.observe(0.1)
    commands = rig.commands[before:]
    assert commanded_phi_travel(commands) <= math.pi + (0.1 if local_budget else 0.011)
    a_index = min(range(len(commands)), key=lambda i: abs(commands[i].data[3] + 2 * math.pi))
    assert abs(commands[a_index].data[3] + 2 * math.pi) < 1e-4
    assert commanded_phi_travel(commands[a_index:]) < (math.pi / 6 if local_budget else 1e-4)
    assert abs(rig.joints[-1].data[3] - (-6.220766497)) < 0.05


def test_place_red_1_1_waypoint_phi_interval_through_real_navigation(
        rotation_rig, tmp_path):
    from action_msgs.msg import GoalStatus, GoalStatusArray
    from catchrobo2026_msgs.srv import EndeffectorControl, PumpControl

    rig = rotation_rig
    start_xyz = (375, 238, 196.95)
    start = PoseStamped(pose=waypoint_pose(start_xyz, -math.pi))
    for _ in range(3):
        rig.pose_pub.publish(start)
        rig.observe(0.1)
    rig.until(lambda: math.dist(xyz(rig.poses[-1]), start_xyz) < 0.1)
    rig.observe(0.1)

    def mechanism_done(request, response):
        response.success = True
        return response

    services = [
        rig.node.create_service(PumpControl, 'set_pump_state', mechanism_done),
        rig.node.create_service(EndeffectorControl, 'set_endeffector_state', mechanism_done),
    ]
    follow_status = []
    status_subscription = rig.node.create_subscription(
        GoalStatusArray, 'follow_route/_action/status', follow_status.append, 10)
    config = yaml.load(
        (Path(__file__).parents[1] / 'config/sequences.yaml').read_text(),
        Loader=yaml.BaseLoader)
    config['sequences']['place_common']['steps'] = [
        step for step in config['sequences']['place_common']['steps']
        if 'manual' not in step]
    config['route_timeout_sec'] = 20.0
    config_file = tmp_path / 'place_red_1_1_phi_interval.yaml'
    config_file.write_text(yaml.safe_dump(config))
    rig.launch('catchrobo2026_sequence', 'sequence_node', {
        'team': 'red', 'sequence_file': config_file,
    })
    assert rig.sequence.wait_for_server(timeout_sec=15)

    before_commands = len(rig.commands)
    handle = rig.resolve(rig.sequence.send_goal_async(ExecuteSequence.Goal(
        control_epoch=1, step_id=1, kind=ExecuteSequence.Goal.PLACE,
        box=1, box_column=1, collector_mask=7)))
    assert handle.accepted
    result = rig.resolve(handle.get_result_async())
    assert result.result.success, result.result.message
    rig.observe(0.3)

    commands = rig.commands[before_commands:]
    assert len(commands) > 20
    waypoint = (150, 0, 360)
    destination = (150.87, -436, 390)
    waypoint_base = math.atan2(-(waypoint[0] - 675), waypoint[1] + 130)
    destination_base = math.atan2(-(destination[0] - 675), destination[1] + 130)

    def boundary_error(command, base):
        q0, q4 = float(command.data[0]), float(command.data[3])
        return abs(q0 - base) + abs(q0 + q4)

    waypoint_index = min(
        range(len(commands)), key=lambda i: boundary_error(commands[i], waypoint_base))
    destination_index = min(
        range(waypoint_index, len(commands)),
        key=lambda i: boundary_error(commands[i], destination_base))
    assert destination_index > waypoint_index
    assert boundary_error(commands[waypoint_index], waypoint_base) < 1e-4
    assert boundary_error(commands[destination_index], destination_base) < 1e-4
    assert commanded_phi_travel(commands[:waypoint_index + 1]) > 0.5
    assert commanded_phi_travel(commands[waypoint_index:destination_index + 1]) <= \
        math.pi / 6 + 1e-4
    assert min(math.dist(xyz(pose), waypoint) for pose in rig.poses) < 90.0
    assert math.dist(xyz(rig.poses[-1]), destination) < 20.1

    # One grouped route plus the two normal PLACE moves must finish.
    succeeded = {
        bytes(status.goal_info.goal_id.uuid)
        for message in follow_status for status in message.status_list
        if status.status == GoalStatus.STATUS_SUCCEEDED
    }
    assert len(succeeded) == 3
    assert result.status == GoalStatus.STATUS_SUCCEEDED
    rig.node.destroy_subscription(status_subscription)
    for service in services:
        rig.node.destroy_service(service)


def test_unrestricted_wrist_interpolation_limits_reversal_speed(rotation_rig):
    rig = rotation_rig
    set_start(rig, -2.0)
    response = plan(rig, [wrist_pose(-1.5)], [1])
    assert response.success
    response.direction = 0
    route = response.routes[0]
    route.path.poses = [PoseStamped(pose=wrist_pose(wrist))
                        for wrist in [-2.0, -1.5, -2.0]]
    route.wrist_angles = [-2.0, -1.5, -2.0]
    route.phi_angles = []
    begin(rig, response, 1)
    before = len(rig.commands)
    handle = follow(rig, response, 0, 1)
    pending = handle.get_result_async()
    rig.observe(0.2)
    assert not pending.done()
    assert rig.resolve(pending).result.success
    rig.observe(0.1)
    wrists = [command.data[3] for command in rig.commands[before:]]
    assert max(wrists) > -1.56
    assert abs(wrists[-1] + 2.0) < 1e-4
    assert max(abs(b - a) for a, b in zip(wrists, wrists[1:])) <= 0.051
    end(rig, 1)
