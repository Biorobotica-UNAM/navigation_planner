"""Real ROS action server and simple_move with a kinematic base and fault-injected services.

Run in a dedicated ROS domain; no hardware topics leave loopback. Planner, locations,
head feedback and obstacle freshness are controlled by the fixture.
"""
import copy
import math
import os
from pathlib import Path
import subprocess
import time
import unittest

# This must be set before the first DDS participant is created.
os.environ['ROS_LOCALHOST_ONLY'] = '1'
os.environ['ROS_DOMAIN_ID'] = os.environ.get('NAVIGATION_TEST_DOMAIN_ID', '173')
os.environ.setdefault('ROS_LOG_DIR', '/tmp/navigation_action_ros_logs')
os.environ['CYCLONEDDS_URI'] = (
    '<CycloneDDS><Domain><General><AllowMulticast>false</AllowMulticast></General>'
    '<Discovery><ParticipantIndex>auto</ParticipantIndex><Peers>'
    '<Peer Address="127.0.0.1"/></Peers></Discovery></Domain></CycloneDDS>')
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.action import ActionClient
from rclpy.task import Future
from rclpy.qos import QoSProfile, DurabilityPolicy
from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist, Vector3
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Bool, Float32, Float64MultiArray
from std_srvs.srv import Trigger
from actionlib_msgs.msg import GoalStatus
from action_msgs.msg import GoalStatus as ActionStatus
from tf2_ros import TransformBroadcaster
from navigation_manager.action import Navigate
from navigation_manager.msg import NavigationStatus
from path_planner.srv import PlanPath, GetPlanningSnapshot
from known_locations_interfaces.srv import LocationsGet
from rcl_interfaces.srv import GetParameters


class NavigationActionIntegration(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node('action_navigation_fixture')
        cls.processes, cls.logs = [], []
        cls.real_planner = os.environ.get('NAVIGATION_TEST_REAL_PLANNER') == '1'
        cls.grid = None
        cls.addClassCleanup(cls.cleanup)
        cls.x = cls.y = cls.yaw = 0.0
        cls.command = Twist()
        cls.integrate = True
        cls.sensor_ready = True
        cls.collision_risk = False
        cls.freeze_tf = False
        cls.blocked = False
        cls.plan_failure = ''
        cls.plan_delay = 0.0
        cls.relocate = False
        cls.start_escape = False
        cls.plan_count = 0
        cls.requests = []
        cls.deferred = []
        cls.statuses, cls.head_goals, cls.motion_commands = [], [], []
        cls.navigation_statuses, cls.feedback = [], []
        cls.head_pose = [0.0, -0.5]
        cls.head_auto = True
        cls.last_tick = time.monotonic()
        cls.tf = TransformBroadcaster(cls.node)
        retained = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        cls.pose_pub = cls.node.create_publisher(PoseStamped, '/navigation/goal', 10)
        cls.reset = cls.node.create_client(Trigger, '/action_test/reset_fault')
        cls.map_pub = cls.node.create_publisher(OccupancyGrid, '/action_test/augmented_map', retained)
        cls.clearance_pub = cls.node.create_publisher(OccupancyGrid, '/action_test/clearance_map', retained)
        cls.ready_pub = cls.node.create_publisher(Bool, '/action_test/potential_fields/ready', 10)
        cls.risk_pub = cls.node.create_publisher(Bool, '/action_test/potential_fields/collision_risk', 10)
        cls.speed_pub = cls.node.create_publisher(Float32, '/action_test/potential_fields/speed_limit', 10)
        cls.force_pub = cls.node.create_publisher(Vector3, '/action_test/potential_fields/pf_rejection_force', 10)
        cls.head_pub = cls.node.create_publisher(Float64MultiArray, '/hardware/head/current_pose', 10)
        cls.result_pub = cls.node.create_publisher(GoalStatus, '/action_test/simple_move/goal_reached', 10)
        cls.subs = [
            cls.node.create_subscription(NavigationStatus, '/action_test/navigation_status',
                                         cls.navigation_statuses.append, retained),
            cls.node.create_subscription(Twist, '/cmd_vel', lambda m: setattr(cls, 'command', m), retained),
            cls.node.create_subscription(GoalStatus, '/action_test/status', lambda m: cls.statuses.append(m), 10),
            cls.node.create_subscription(Float64MultiArray, '/hardware/head/goal_pose', cls.head_command, 10),
        ]
        from path_planner.msg import NavigationPath
        cls.path_pub = cls.node.create_publisher(NavigationPath, '/action_test/simple_move/navigation_path', 10)
        cls.subs.append(cls.node.create_subscription(
            NavigationPath, '/action_test/simple_move/navigation_path', lambda m: cls.motion_commands.append(m), 10))
        if cls.real_planner:
            cls.plan_service = cls.node.create_service(
                GetPlanningSnapshot, '/action_test/map_augmenter/get_planning_snapshot', cls.snapshot)
        else:
            cls.plan_service = cls.node.create_service(
                PlanPath, '/action_test/path_planner/plan_navigation', cls.plan, callback_group=ReentrantCallbackGroup())
        cls.location_service = cls.node.create_service(LocationsGet, '/test_locations/get', cls.location)
        cls.timer = cls.node.create_timer(0.02, cls.tick)
        nodes = [
            ('simple_move', 'simple_move_node', 'simple_move',
             ['use_pot_fields:=true', 'move_head:=true', 'max_linear_speed:=0.4']),
            ('navigation_manager', 'navigation_manager_node', 'navigation_manager',
             ['motion_node:=simple_move', 'locations_service:=/test_locations/get', 'legacy_move_head:=false',
              'plan_request_timeout_sec:=1.0', 'replan_period_sec:=0.15',
              'sensor_wait_timeout_sec:=2.0', 'recovery.replan.attempts:=3',
              'recovery.observation.attempts:=1', 'recovery.sensors.attempts:=2',
              'execution_timeout_sec:=40.0']),
        ]
        if cls.real_planner:
            nodes.append(('path_planner', 'path_planner_node', 'path_planner', ['diagonal_paths:=true']))
        for package, executable, name, parameters in nodes:
            binary = Path(get_package_prefix(package)) / 'lib' / package / executable
            log = open(f'/tmp/navigation_action_{os.environ["ROS_DOMAIN_ID"]}_{name}.log', 'w')
            cls.logs.append(log)
            args = [str(binary), '--ros-args', '-r', '__ns:=/action_test', '-r', f'__node:={name}',
                    '-p', 'use_namespace:=true']
            for parameter in parameters:
                args += ['-p', parameter]
            cls.processes.append(subprocess.Popen(args, stdout=log, stderr=subprocess.STDOUT))
        cls.client = ActionClient(cls.node, Navigate, '/action_test/navigate')
        cls.params = cls.node.create_client(GetParameters, '/action_test/simple_move/get_parameters')
        cls.wait_for(lambda: cls.client.server_is_ready() and cls.params.service_is_ready(), 12)
        cls.spin_for(0.7)
        for process in cls.processes:
            assert process.poll() is None, 'Navigation process exited; inspect /tmp/navigation_action_*.log'

    @classmethod
    def cleanup(cls):
        for p in cls.processes:
            p.terminate()
        for p in cls.processes:
            try:
                p.wait(5)
            except subprocess.TimeoutExpired:
                p.kill()
                p.wait(5)
        for log in cls.logs:
            log.close()
        for _, future in cls.deferred:
            if not future.done():
                future.set_result(False)
        cls.spin_for(0.1)
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def head_command(cls, message):
        cls.head_goals.append(message)
        if cls.head_auto:
            cls.head_pose = list(message.data)

    @classmethod
    async def plan(cls, request, response):
        cls.plan_count += 1
        cls.requests.append(request)
        failure, relocate, escape = cls.plan_failure, cls.relocate, cls.start_escape
        if cls.plan_delay:
            future = Future()
            cls.deferred.append((time.monotonic() + cls.plan_delay, future))
            await future
        response.map_stamp = cls.node.get_clock().now().to_msg()
        if failure:
            response.reason = failure
            response.has_observation_target = True
            response.observation_target = request.goal.pose.position
            return response
        response.path.header.frame_id = 'map'
        endpoint = copy.deepcopy(request.goal)
        if relocate:
            endpoint.pose.position.x -= 0.2
            response.goal_relocated = True
        if escape:
            endpoint = copy.deepcopy(request.start)
            endpoint.pose.position.x += 0.35
            response.start_recovery = True
            cls.start_escape = False
        for i in range(21):
            pose = copy.deepcopy(endpoint)
            t = i / 20
            pose.pose.position.x = request.start.pose.position.x * (1-t) + endpoint.pose.position.x * t
            pose.pose.position.y = request.start.pose.position.y * (1-t) + endpoint.pose.position.y * t
            response.path.poses.append(pose)
        return response

    @classmethod
    def snapshot(cls, request, response):
        if cls.grid is None:
            return response
        response.ready = True
        response.revision = 1
        response.map = copy.deepcopy(cls.grid)
        response.clearance = copy.deepcopy(cls.grid)
        response.costs = copy.deepcopy(cls.grid)
        response.costs.data = [0] * len(cls.grid.data)
        response.uncertain_obstacles.header = cls.grid.header
        return response

    @classmethod
    def location(cls, request, response):
        response.success = request.location_name == 'desk'
        response.pose.position.x = 0.5
        response.pose.orientation.w = 1.0
        return response

    @classmethod
    def tick(cls):
        now = time.monotonic()
        dt = min(0.05, now - cls.last_tick)
        cls.last_tick = now
        for deadline, future in cls.deferred[:]:
            if now >= deadline:
                future.set_result(True)
                cls.deferred.remove((deadline, future))
        if cls.integrate:
            cls.x += (math.cos(cls.yaw)*cls.command.linear.x - math.sin(cls.yaw)*cls.command.linear.y)*dt
            cls.y += (math.sin(cls.yaw)*cls.command.linear.x + math.cos(cls.yaw)*cls.command.linear.y)*dt
            cls.yaw += cls.command.angular.z*dt
        stamp = cls.node.get_clock().now().to_msg()
        if not cls.freeze_tf:
            transforms = []
            for parent, child in [('map', 'odom'), ('odom', 'base_link')]:
                tf = TransformStamped()
                tf.header.stamp, tf.header.frame_id, tf.child_frame_id = stamp, parent, child
                tf.transform.rotation.w = 1.0
                if child == 'base_link':
                    tf.transform.translation.x, tf.transform.translation.y = cls.x, cls.y
                    tf.transform.rotation.z, tf.transform.rotation.w = math.sin(cls.yaw/2), math.cos(cls.yaw/2)
                transforms.append(tf)
            cls.tf.sendTransform(transforms)
        grid = OccupancyGrid()
        grid.header.stamp, grid.header.frame_id = stamp, 'map'
        grid.info.width = grid.info.height = 80
        grid.info.resolution = 0.1
        grid.info.origin.position.x = grid.info.origin.position.y = -4.0
        grid.info.origin.orientation.w = 1.0
        data = [0] * 6400
        if cls.relocate:
            data[39*80+45] = data[39*80+46] = data[40*80+45] = data[40*80+46] = 100
        if cls.blocked:
            for row in range(38, 43):
                for col in range(43, 46):
                    data[row*80+col] = 100
        grid.data = data
        cls.grid = grid
        cls.map_pub.publish(grid)
        cls.clearance_pub.publish(grid)
        cls.ready_pub.publish(Bool(data=cls.sensor_ready))
        cls.risk_pub.publish(Bool(data=cls.collision_risk))
        cls.speed_pub.publish(Float32(data=1.0 if cls.sensor_ready else 0.0))
        cls.force_pub.publish(Vector3())
        cls.head_pub.publish(Float64MultiArray(data=cls.head_pose))

    @classmethod
    def wait_for(cls, predicate, timeout=10):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if predicate():
                return
            rclpy.spin_once(cls.node, timeout_sec=0.02)
        raise AssertionError('Condition timed out; inspect /tmp/navigation_action_*.log')

    @classmethod
    def spin_for(cls, seconds):
        deadline = time.monotonic()+seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(cls.node, timeout_sec=0.02)

    def setUp(self):
        cls = type(self)
        cls.x = cls.y = cls.yaw = 0.0
        cls.integrate = cls.sensor_ready = cls.head_auto = True
        cls.head_pose = [0.0, -0.5]
        cls.freeze_tf = cls.blocked = cls.relocate = cls.start_escape = False
        cls.plan_failure = ''
        cls.plan_delay = 0.0
        cls.collision_risk = False
        cls.statuses.clear()
        cls.navigation_statuses.clear()
        cls.feedback.clear()
        cls.head_goals.clear()
        cls.motion_commands.clear()
        cls.requests.clear()
        self.spin_for(0.6)

    def send(self, x=0.6, y=0.0, yaw=0.0, head=False, location=None):
        goal = Navigate.Goal()
        goal.target_type = Navigate.Goal.LOCATION if location else Navigate.Goal.POSE
        goal.location_name = location or ''
        goal.pose.header.frame_id = 'map'
        goal.pose.pose.position.x, goal.pose.pose.position.y = float(x), float(y)
        goal.pose.pose.orientation.z, goal.pose.pose.orientation.w = math.sin(yaw/2), math.cos(yaw/2)
        goal.move_head = head
        future = self.client.send_goal_async(goal, feedback_callback=lambda m: self.feedback.append(m.feedback))
        self.wait_for(future.done)
        handle = future.result()
        self.assertTrue(handle.accepted)
        return handle

    def result(self, handle, timeout=20):
        future = handle.get_result_async()
        self.wait_for(future.done, timeout)
        return future.result()

    def cancel(self, handle):
        future = handle.cancel_goal_async()
        self.wait_for(future.done)
        self.assertTrue(future.result().goals_canceling)
        result = self.result(handle)
        self.assertEqual(result.status, ActionStatus.STATUS_CANCELED)
        self.assertEqual(result.result.result_code, Navigate.Result.CANCELLED)
        self.assertLess(math.hypot(self.command.linear.x, self.command.linear.y), 0.001)
        return result

    def test_01_pose_no_head_and_orientation(self):
        result = self.result(self.send(yaw=0.3))
        self.assertEqual(result.result.result_code, Navigate.Result.SUCCESS)
        self.assertLessEqual(result.result.position_error_m, 0.03)
        self.assertLessEqual(result.result.orientation_error_rad, 0.05)
        self.assertEqual(len(self.head_goals), 0)
        moving = [m for m in self.feedback if m.status_code == Navigate.Feedback.MOVING]
        self.assertGreaterEqual(len(moving), 2)
        self.assertGreater(moving[0].distance_remaining_m, moving[-1].distance_remaining_m)
        self.assertTrue(any(m.status_message == 'Approaching goal' for m in moving))
        self.wait_for(lambda: any(m.source == NavigationStatus.SOURCE_ACTION and m.result_valid and
                                  m.result_code == Navigate.Result.SUCCESS for m in self.navigation_statuses))
        request = GetParameters.Request(names=['move_head'])
        future = self.params.call_async(request)
        self.wait_for(future.done)
        self.assertFalse(future.result().values[0].bool_value)

    def test_02_location_with_head(self):
        result = self.result(self.send(head=True, location='desk'))
        self.assertEqual(result.result.result_code, Navigate.Result.SUCCESS)
        self.assertGreater(len(self.head_goals), 0)
        unknown = self.result(self.send(location='missing'))
        self.assertEqual(unknown.result.result_code, Navigate.Result.INVALID_GOAL)
        self.assertEqual(unknown.result.reason, 'unknown_location')

    def test_03_approximate_arrival_errors_original_goal(self):
        type(self).relocate = True
        result = self.result(self.send())
        self.assertEqual(result.result.result_code, Navigate.Result.APPROXIMATE)
        self.assertGreater(result.result.position_error_m, 0.03 if self.real_planner else 0.15)
        self.assertLess(result.result.position_error_m, 0.33)

    def test_04_cancel_during_planning_and_ignore_late_response(self):
        type(self).plan_delay = 1.5
        before = len(self.requests)
        handle = self.send()
        self.wait_for(lambda: len(self.requests) > before)
        self.cancel(handle)
        self.spin_for(1.5)
        self.assertEqual(len(self.motion_commands), 0)

    def test_05_cancel_during_motion(self):
        handle = self.send(x=2.0)
        self.wait_for(lambda: math.hypot(self.command.linear.x, self.command.linear.y) > 0.03)
        self.cancel(handle)
        at_stop = self.x
        self.spin_for(0.4)
        self.assertAlmostEqual(self.x, at_stop, delta=0.01)

    def test_06_preempt_stops_old_goal_before_new_motion(self):
        old = self.send(x=2.0)
        self.wait_for(lambda: self.command.linear.x > 0.03)
        new = self.send(x=0.2, y=0.3)
        previous = self.result(old)
        self.assertEqual(previous.result.result_code, Navigate.Result.PREEMPTED)
        result = self.result(new)
        self.assertEqual(result.result.result_code, Navigate.Result.SUCCESS)

    def test_07_sensor_recovery_and_cancel_recovery(self):
        handle = self.send(x=1.0)
        self.wait_for(lambda: self.command.linear.x > 0.03)
        type(self).sensor_ready = False
        self.wait_for(lambda: any(s.text == 'obstacle_data_stale' for s in self.statuses))
        self.wait_for(lambda: any(m.status_code == NavigationStatus.RECOVERING and
                                  m.total_recoveries > 0 for m in self.navigation_statuses))
        self.spin_for(0.7)
        self.assertLess(abs(self.command.linear.x), 0.001)
        type(self).sensor_ready = True
        result = self.result(handle)
        self.assertEqual(result.result.result_code, Navigate.Result.SUCCESS)
        type(self).plan_failure = 'no_path'
        handle = self.send()
        self.wait_for(lambda: any(s.text == 'no_path' for s in self.statuses))
        self.cancel(handle)

    def test_08_blocked_route_is_not_followed_and_recovery_is_bounded(self):
        type(self).blocked = True
        result = self.result(self.send(x=1.0), 18)
        self.assertEqual(result.result.result_code, Navigate.Result.FAILED)
        self.assertEqual(result.result.reason, 'plan_invalid_on_current_map')
        self.assertAlmostEqual(self.x, 0.0, delta=0.02)
        self.assertEqual(len(self.head_goals), 0)

    def test_09_planner_timeout_and_late_response(self):
        type(self).plan_delay = 1.6
        result = self.result(self.send(), 18)
        self.assertEqual(result.result.result_code, Navigate.Result.FAILED)
        self.assertIn('plan_request_timeout', result.result.reason)
        self.assertEqual(len(self.motion_commands), 0)
        self.spin_for(2.0)
        self.assertEqual(len(self.motion_commands), 0)

    def test_10_observation_respects_head_flag(self):
        type(self).plan_failure = 'no_path'
        result = self.result(self.send(head=True), 18)
        self.assertEqual(result.result.result_code, Navigate.Result.FAILED)
        from path_planner.msg import NavigationPath
        self.assertTrue(any(m.mode == NavigationPath.OBSERVE for m in self.motion_commands))
        self.assertGreater(len(self.head_goals), 0)

    def test_11_success_from_wrong_motion_is_ignored(self):
        type(self).integrate = False
        handle = self.send()
        self.wait_for(lambda: bool(self.motion_commands))
        self.result_pub.publish(GoalStatus(status=GoalStatus.SUCCEEDED, text='done'))
        self.spin_for(0.4)
        future = handle.get_result_async()
        self.assertFalse(future.done())
        self.cancel(handle)

    def test_12_localization_loss_stops_then_recovers(self):
        handle = self.send(x=1.0)
        self.wait_for(lambda: self.command.linear.x > 0.03)
        type(self).freeze_tf = True
        self.spin_for(0.8)
        self.assertLess(abs(self.command.linear.x), 0.001)
        type(self).freeze_tf = False
        result = self.result(handle)
        self.assertEqual(result.result.result_code, Navigate.Result.SUCCESS)

    def test_13_sensor_timeout_is_bounded_and_never_moves(self):
        type(self).sensor_ready = False
        result = self.result(self.send(), 22)
        self.assertEqual(result.result.result_code, Navigate.Result.FAILED)
        self.assertEqual(result.result.reason, 'obstacle_data_unavailable')
        self.assertEqual(len(self.motion_commands), 0)
        self.assertAlmostEqual(self.x, 0.0, delta=0.001)
        type(self).sensor_ready = True

    def test_14_cancel_during_active_observation(self):
        type(self).plan_failure = 'no_path'
        type(self).head_auto = False
        handle = self.send(x=0.6, y=0.6, head=True)
        from path_planner.msg import NavigationPath
        self.wait_for(lambda: any(m.mode == NavigationPath.OBSERVE for m in self.motion_commands), 10)
        self.spin_for(0.2)
        self.cancel(handle)
        self.assertLess(math.hypot(self.x, self.y), 0.001)

    def test_15_delayed_command_cannot_restart_after_cancel(self):
        handle = self.send(x=1.0)
        self.wait_for(lambda: self.command.linear.x > 0.03)
        stale = copy.deepcopy(self.motion_commands[-1])
        self.cancel(handle)
        stopped_at = self.x
        self.path_pub.publish(stale)
        self.spin_for(0.6)
        self.assertAlmostEqual(self.x, stopped_at, delta=0.001)
        self.assertLess(abs(self.command.linear.x), 0.001)

    def test_16_topic_goal_and_late_status_subscriber(self):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position.x = 0.4
        pose.pose.orientation.w = 1.0
        self.pose_pub.publish(pose)
        self.wait_for(lambda: any(m.source == NavigationStatus.SOURCE_TOPIC and m.result_valid
                                  for m in self.navigation_statuses))
        result = self.navigation_statuses[-1]
        self.assertEqual(result.result_code, Navigate.Result.SUCCESS)
        self.assertFalse(self.feedback)
        self.assertTrue(any(m.source == NavigationStatus.SOURCE_TOPIC and
                            m.status_code == NavigationStatus.FOLLOWING and not m.result_valid
                            for m in self.navigation_statuses))
        received = []
        retained = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        subscription = self.node.create_subscription(
            NavigationStatus, '/action_test/navigation_status', received.append, retained)
        try:
            self.wait_for(lambda: bool(received))
            self.assertEqual(received[0], result)
            self.spin_for(0.45)
            self.assertEqual(len(received), 1)
        finally:
            self.node.destroy_subscription(subscription)

    def test_17_cancel_while_pending(self):
        previous = self.send(x=2.0)
        self.wait_for(lambda: self.command.linear.x > 0.03)
        pending = self.send(x=1.0)
        request = pending.cancel_goal_async()
        self.wait_for(request.done)
        self.assertTrue(request.result().goals_canceling)
        canceled = self.result(pending)
        self.assertEqual(canceled.status, ActionStatus.STATUS_CANCELED)
        self.assertEqual(canceled.result.result_code, Navigate.Result.CANCELLED)
        self.assertEqual(self.result(previous).result.result_code, Navigate.Result.PREEMPTED)
        self.assertEqual(len(self.motion_commands), 1)

    def test_18_fault_reset_requires_fresh_odometry(self):
        handle = self.send(x=2.0)
        self.wait_for(lambda: self.command.linear.x > 0.03)
        type(self).freeze_tf = True
        self.spin_for(0.65)
        cancel = handle.cancel_goal_async()
        self.wait_for(cancel.done)
        self.assertTrue(cancel.result().goals_canceling)
        self.assertEqual(self.result(handle).result.reason, 'stop_unverified')
        self.wait_for(lambda: bool(self.navigation_statuses) and self.navigation_statuses[-1].faulted)
        commands = len(self.motion_commands)
        self.assertEqual(self.result(self.send()).result.reason, 'stop_unverified')
        self.assertEqual(len(self.motion_commands), commands)

        self.wait_for(self.reset.service_is_ready)
        attempt = self.reset.call_async(Trigger.Request())
        self.wait_for(attempt.done)
        self.assertTrue(attempt.result().success)
        self.wait_for(lambda: self.navigation_statuses[-1].status_message.startswith('Fault reset timed out'))
        self.assertTrue(self.navigation_statuses[-1].faulted)

        type(self).freeze_tf = False
        attempt = self.reset.call_async(Trigger.Request())
        self.wait_for(attempt.done)
        self.assertTrue(attempt.result().success)
        self.wait_for(lambda: not self.navigation_statuses[-1].faulted)
        self.assertEqual(self.navigation_statuses[-1].status_message, 'Navigation fault cleared')
        self.spin_for(0.45)
        self.assertEqual(len(self.motion_commands), commands)
        self.assertEqual(self.result(self.send()).result.result_code, Navigate.Result.SUCCESS)

    def test_19_head_tracking_resumes_after_collision_replan(self):
        cls = type(self)
        handle = self.send(x=0.8, y=0.2, head=True)
        self.wait_for(lambda: self.command.linear.x > 0.03 and bool(self.head_goals))

        cls.collision_risk = True
        self.wait_for(lambda: any(m.status_code == NavigationStatus.RECOVERING and
                                  m.total_recoveries > 0 for m in self.navigation_statuses))
        head_commands = len(self.head_goals)
        cls.collision_risk = False
        self.wait_for(lambda: len(self.motion_commands) >= 2)
        self.wait_for(lambda: len(self.head_goals) > head_commands)

        future = self.params.call_async(GetParameters.Request(names=['move_head']))
        self.wait_for(future.done)
        self.assertTrue(future.result().values[0].bool_value)
        self.assertEqual(self.result(handle).result.result_code, Navigate.Result.SUCCESS)


if __name__ == '__main__':
    unittest.main()
