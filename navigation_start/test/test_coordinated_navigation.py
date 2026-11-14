#!/usr/bin/env python3
"""ROS integration tests in a separate DDS domain, with simulated base/head/map.

Run after sourcing the built workspace:
  ROS_DOMAIN_ID=173 ROS_LOCALHOST_ONLY=1 python3 test_coordinated_navigation.py
No hardware nodes are started. The supplied domain must be isolated from robots.
Run the unknown-policy cases with NAVIGATION_TEST_ALLOW_UNKNOWN=0 and then 1.
Set NAVIGATION_TEST_CMD_VEL_RELAY=1 to route simulated motion through topic_tools.
"""
import array
import copy
import math
import os
from pathlib import Path as FilePath
import subprocess
import struct
import time
import unittest

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.srv import SetParametersAtomically
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from ament_index_python.packages import get_package_prefix
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist, Vector3
from nav_msgs.msg import OccupancyGrid, Path
from nav_msgs.srv import GetMap
from path_planner.srv import PlanPath
from path_planner.msg import NavigationPath
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from std_msgs.msg import Bool, Empty, Float64MultiArray, Float32MultiArray, Float32
from tf2_ros import TransformBroadcaster

# Ignore deployment peers in the developer shell: every test process stays on loopback.
os.environ['CYCLONEDDS_URI'] = (
    '<CycloneDDS><Domain><General>'
    '<AllowMulticast>false</AllowMulticast></General><Discovery><ParticipantIndex>auto</ParticipantIndex><Peers>'
    '<Peer Address="127.0.0.1"/></Peers></Discovery></Domain></CycloneDDS>')


class NavigationIntegration(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if os.environ.get('ROS_LOCALHOST_ONLY') != '1' or not os.environ.get('ROS_DOMAIN_ID'):
            raise RuntimeError('Set an isolated ROS_DOMAIN_ID and ROS_LOCALHOST_ONLY=1')
        rclpy.init()
        cls.node = Node('navigation_test_fixture')
        cls.processes = []
        cls.logs = []
        cls.addClassCleanup(cls.cleanup)
        cls.allow_unknown = os.environ.get('NAVIGATION_TEST_ALLOW_UNKNOWN') == '1'
        cls.use_cmd_vel_relay = os.environ.get('NAVIGATION_TEST_CMD_VEL_RELAY') == '1'
        if cls.use_cmd_vel_relay:
            # Discover the bumper's retained publisher before navigation starts.
            cls.bumper_cmd_pub = cls.node.create_publisher(
                Twist, '/cmd_vel',
                QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL))
            binary = FilePath(get_package_prefix('topic_tools'))/'lib/topic_tools/relay'
            log = open(f"{os.environ.get('NAVIGATION_LOG_PREFIX', '/tmp/navigation_test_')}relay.log", 'w')
            cls.logs.append(log)
            cls.processes.append(subprocess.Popen(
                [str(binary), '/cmd_vel', '/test_base_cmd_vel',
                 '--ros-args', '-r', '__node:=test_cmd_vel_relay'],
                stdout=log, stderr=subprocess.STDOUT))
            deadline = time.monotonic()+8
            while time.monotonic() < deadline:
                endpoints = cls.node.get_subscriptions_info_by_topic('/cmd_vel')
                if any(e.node_name == 'test_cmd_vel_relay' and
                       e.qos_profile.durability == DurabilityPolicy.TRANSIENT_LOCAL for e in endpoints):
                    break
                rclpy.spin_once(cls.node, timeout_sec=0.02)
            else:
                raise RuntimeError('Relay did not discover the transient-local bumper publisher')
        for package, executable, name, extra in [
            ('simple_move', 'simple_move_node', 'simple_move',
             ['-p', 'move_head:=false', '-p', 'max_linear_speed:=0.4']),
            ('mvn_pln', 'mvn_pln_node', 'mvn_pln', []),
            ('potential_fields', 'potential_fields_node', 'potential_fields',
             ['-p', 'use_lidar:=true', '-p', 'use_point_cloud:=true',
              '-p', 'use_pot_fields:=true', '-p', 'laser_min_z:=-0.1',
              '-p', 'laser_min_x:=0.1', '-p', 'laser_max_x:=1.0',
              '-p', 'laser_pot_fields_d0:=0.9', '-p', 'lidar_points_threshold:=0']),
        ]:
            if package in ('simple_move', 'mvn_pln'):
                extra += ['-p', f'allow_unknown:={str(cls.allow_unknown).lower()}']
            binary = FilePath(get_package_prefix(package)) / 'lib' / package / executable
            log = open(f"{os.environ.get('NAVIGATION_LOG_PREFIX', '/tmp/navigation_test_')}{name}.log", 'w')
            cls.logs.append(log)
            cls.processes.append(subprocess.Popen(
                [str(binary), '--ros-args', '-r', '__ns:=/navigation',
                 '-r', f'__node:={name}', '-p', 'use_namespace:=true'] + extra,
                stdout=log, stderr=subprocess.STDOUT))
        cls.tf = TransformBroadcaster(cls.node)
        cls.map_pub = cls.node.create_publisher(
            OccupancyGrid, '/navigation/augmented_map',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        cls.clearance_pub = cls.node.create_publisher(
            OccupancyGrid, '/navigation/clearance_map',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        cls.path_pub = cls.node.create_publisher(Path, '/navigation/simple_move/goal_path', 10)
        cls.navigation_path_pub = cls.node.create_publisher(
            NavigationPath, '/navigation/simple_move/navigation_path', 10)
        cls.relative_pub = cls.node.create_publisher(Float32MultiArray, '/navigation/simple_move/goal_dist_angle', 10)
        cls.goal_pub = cls.node.create_publisher(PoseStamped, '/nav_control/goal', 10)
        cls.stop_pub = cls.node.create_publisher(Empty, '/navigation/stop', 10)
        cls.head_pub = cls.node.create_publisher(Float64MultiArray, '/hardware/head/current_pose', 10)
        cls.enable_pub = cls.node.create_publisher(Bool, '/navigation/potential_fields/enable', 10)
        cls.cloud_enable_pub = cls.node.create_publisher(Bool, '/navigation/potential_fields/enable_cloud', 10)
        cls.intent_pub = cls.node.create_publisher(Vector3, '/navigation/simple_move/motion_direction', 10)
        cls.scan_pub = cls.node.create_publisher(LaserScan, '/scan', 10)
        cls.cloud_pub = cls.node.create_publisher(PointCloud2, '/points', qos_profile_sensor_data)
        cls.result_pub = cls.node.create_publisher(GoalStatus, '/navigation/simple_move/goal_reached', 10)
        cls.subs = [
            cls.node.create_subscription(Twist, '/test_base_cmd_vel' if cls.use_cmd_vel_relay else '/cmd_vel',
                                         lambda m: setattr(cls, 'command', m), 10),
            cls.node.create_subscription(GoalStatus, '/navigation/status', lambda m: cls.statuses.append(m), 10),
            cls.node.create_subscription(GoalStatus, '/navigation/simple_move/goal_reached', lambda m: cls.results.append(m), 10),
            cls.node.create_subscription(Float64MultiArray, '/hardware/head/goal_pose',
                lambda m: (cls.head_goals.append(m), cls.head_records.append((time.monotonic(), m.data[0]))), 10),
            cls.node.create_subscription(Float32, '/navigation/potential_fields/speed_limit', lambda m: setattr(cls, 'obstacle_limit', m.data), 10),
            cls.node.create_subscription(Bool, '/navigation/potential_fields/ready', cls.record_readiness, 10),
            cls.node.create_subscription(Bool, '/navigation/potential_fields/collision_risk', lambda m: setattr(cls, 'risk', m.data), 10),
            cls.node.create_subscription(Vector3, '/navigation/potential_fields/pf_rejection_force', lambda m: setattr(cls, 'force', m), 10),
        ]
        cls.params = cls.node.create_client(SetParametersAtomically, '/navigation/simple_move/set_parameters_atomically')
        cls.manager_params = cls.node.create_client(SetParametersAtomically, '/navigation/mvn_pln/set_parameters_atomically')
        cls.planner = cls.node.create_service(PlanPath, '/navigation/path_planner/plan_navigation', cls.plan)
        cls.timer = cls.node.create_timer(0.02, cls.tick)
        cls.x = cls.y = cls.yaw = 0.0
        cls.command = Twist()
        cls.statuses, cls.results, cls.head_goals = [], [], []
        cls.head_records = []
        cls.integrate = False
        cls.head_auto = False
        cls.head_pan = 0.0
        cls.scan_angle, cls.scan_range = 0.0, float('inf')
        cls.publish_scan = True
        cls.cloud_points = 1
        cls.cloud_payload = array.array('B', struct.pack('<fff', 2.0, 0.0, 0.4))
        cls.cloud_period_sec = 0.02
        cls.last_cloud_sent = 0.0
        cls.monitor_times = []
        cls.monitor_ready = []
        cls.cloud_tf_lag = 0.10
        cls.cloud_stamp_offset = 0.0
        cls.risk, cls.ready, cls.force = None, None, Vector3()
        cls.rotation_wall = False
        cls.corner_obstacle = False
        cls.inflated_start = False
        cls.curved_escape = False
        cls.blocked = False
        cls.map_fill = 0
        cls.plan_calls = 0
        cls.plan_requests = []
        cls.planner_stale_response = False
        cls.planner_failure = ''
        cls.planner_goal_offset = 0.0
        cls.planner_local_recovery = False
        cls.planner_recovery_forever = False
        cls.blocked_goal_position = None
        cls.short_escape = False
        cls.odom_jitter = 0.0
        cls.map_jitter = 0.0
        cls.freeze_odom = False
        cls.odom_tick = 0
        deadline = time.monotonic() + 8
        while not cls.params.service_is_ready() and time.monotonic() < deadline:
            rclpy.spin_once(cls.node, timeout_sec=0.02)
        if not cls.params.service_is_ready():
            raise RuntimeError('Controller failed to start; see /tmp/navigation_test_*.log')
        cls.spin_for(0.6)
        for process in cls.processes:
            if process.poll() is not None:
                raise RuntimeError('A node exited; see /tmp/navigation_test_*.log')

    @classmethod
    def cleanup(cls):
        for process in cls.processes:
            process.terminate()
        for process in cls.processes:
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=5)
        for log in cls.logs:
            log.close()
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def plan(cls, request, response):
        cls.plan_calls += 1
        cls.plan_requests.append(request)
        response.map_stamp = request.minimum_map_stamp
        if cls.planner_failure:
            response.reason = cls.planner_failure
            return response
        if cls.planner_stale_response:
            response.map_stamp.sec -= 1
        response.path.header.frame_id = 'map'
        end_x = request.goal.pose.position.x + cls.planner_goal_offset
        end_y = request.goal.pose.position.y
        response.goal_relocated = cls.planner_goal_offset != 0.0
        if cls.planner_recovery_forever:
            response.start_recovery = True
            end_x, end_y = request.start.pose.position.x, request.start.pose.position.y
        elif cls.planner_local_recovery and cls.plan_calls == 1:
            response.start_recovery = True
            end_x, end_y = 0.25, 0.0
        for i in range(41):
            p = PoseStamped()
            t = i / 40
            p.pose.position.x = request.start.pose.position.x * (1-t) + end_x*t
            p.pose.position.y = request.start.pose.position.y * (1-t) + end_y*t
            p.pose.orientation = request.start.pose.orientation if response.start_recovery else request.goal.pose.orientation
            response.path.poses.append(p)
        return response

    @classmethod
    def record_readiness(cls, message):
        cls.ready = message.data
        cls.monitor_times.append(time.monotonic())
        cls.monitor_ready.append(message.data)

    @classmethod
    def tick(cls):
        if cls.integrate:
            cls.x += (math.cos(cls.yaw)*cls.command.linear.x - math.sin(cls.yaw)*cls.command.linear.y)*0.02
            cls.y += (math.sin(cls.yaw)*cls.command.linear.x + math.cos(cls.yaw)*cls.command.linear.y)*0.02
            cls.yaw += cls.command.angular.z*0.02
        stamp = cls.node.get_clock().now().to_msg()
        transforms = []
        cls.odom_tick += 1
        for parent, child in [('map', 'odom'), ('odom', 'base_link')]:
            if child == 'base_link' and cls.freeze_odom:
                continue
            tf = TransformStamped()
            tf.header.stamp, tf.header.frame_id, tf.child_frame_id = stamp, parent, child
            tf.transform.rotation.w = 1.0
            if child == 'odom':
                tf.transform.translation.y = cls.map_jitter * (1 if cls.odom_tick % 2 else -1)
            if child == 'base_link':
                tf.transform.translation.x, tf.transform.translation.y = cls.x, cls.y
                tf.transform.translation.x += cls.odom_jitter * (1 if cls.odom_tick % 2 else -1)
                tf.transform.rotation.z, tf.transform.rotation.w = math.sin(cls.yaw/2), math.cos(cls.yaw/2)
            transforms.append(tf)
        camera_tf = TransformStamped()
        camera_tf.header.frame_id, camera_tf.child_frame_id = 'base_link', 'test_camera'
        camera_tf.header.stamp = (cls.node.get_clock().now()-rclpy.duration.Duration(seconds=cls.cloud_tf_lag)).to_msg()
        camera_tf.transform.rotation.w = 1.0
        transforms.append(camera_tf)
        cls.tf.sendTransform(transforms)
        grid = OccupancyGrid()
        grid.header.stamp, grid.header.frame_id = stamp, 'map'
        grid.info.width = grid.info.height = 160
        grid.info.resolution = 0.05
        grid.info.origin.position.x = grid.info.origin.position.y = -4.0
        grid.info.origin.orientation.w = 1.0
        data = [cls.map_fill]*25600
        if cls.blocked:
            for iy in range(76, 85):
                for ix in range(87, 93):
                    data[iy*160+ix] = 100
        if cls.rotation_wall:
            for iy in range(60, 79):
                for ix in range(80, 130):
                    data[iy*160+ix] = 100
        if cls.corner_obstacle:
            for iy in range(84, 96):
                for ix in range(76, 92):
                    data[iy*160+ix] = 100
        grid.data = data
        if cls.blocked_goal_position is not None:
            gx, gy = cls.blocked_goal_position
            # OccupancyGrid serializes resolution as float32; use its wire value
            # so goals on cell boundaries address the same cell as the C++ nodes.
            resolution = struct.unpack('<f', struct.pack('<f', grid.info.resolution))[0]
            ix, iy = int((gx+4.0)/resolution), int((gy+4.0)/resolution)
            data[iy*160+ix] = 100
            grid.data = data
        physical = copy.deepcopy(grid)
        if cls.short_escape:
            data[80*160+80] = 100
            grid.data = data
        if cls.curved_escape:
            cells = list(physical.data)
            for iy in range(78,82): cells[iy*160+81] = 100
            physical.data = cells
            for iy in range(74,87):
                for ix in range(74,87): data[iy*160+ix] = 100
            for ix in range(84,160): data[80*160+ix] = 0
            grid.data = data
        if cls.inflated_start:
            for iy in range(74, 86):
                for ix in range(74, 86):
                    if math.hypot((ix+0.5)*0.05-4, (iy+0.5)*0.05-4) < 0.20:
                        data[iy*160+ix] = 100
            grid.data = data
        cls.clearance_pub.publish(physical)
        cls.map_pub.publish(grid)
        if cls.head_auto and cls.head_goals:
            delta = cls.head_goals[-1].data[0] - cls.head_pan
            cls.head_pan += max(-0.025, min(0.025, delta))
        head = Float64MultiArray()
        head.data = [cls.head_pan, -0.5]
        cls.head_pub.publish(head)
        if time.monotonic()-cls.last_cloud_sent >= cls.cloud_period_sec:
            cls.last_cloud_sent = time.monotonic()
            cloud = PointCloud2()
            cloud.header.frame_id = 'test_camera'
            cloud.header.stamp = (cls.node.get_clock().now()-rclpy.duration.Duration(seconds=cls.cloud_stamp_offset)).to_msg()
            cloud.width, cloud.height = cls.cloud_points, 1
            cloud.point_step, cloud.row_step = 12, 12*cls.cloud_points
            cloud.fields = [PointField(name=name, offset=i*4, datatype=PointField.FLOAT32, count=1)
                            for i, name in enumerate(('x', 'y', 'z'))]
            cloud.data = cls.cloud_payload
            cls.cloud_pub.publish(cloud)
        if cls.publish_scan:
            scan = LaserScan()
            scan.header.stamp, scan.header.frame_id = stamp, 'base_link'
            scan.angle_min, scan.angle_max, scan.angle_increment = -math.pi, math.pi, math.pi/180
            scan.range_min, scan.range_max = 0.1, 10.0
            scan.ranges = [float('inf')]*361
            if math.isfinite(cls.scan_range):
                scan.ranges[round((cls.scan_angle + math.pi)/scan.angle_increment)] = cls.scan_range
            cls.scan_pub.publish(scan)

    @classmethod
    def spin_for(cls, seconds):
        deadline = time.monotonic()+seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(cls.node, timeout_sec=0.01)

    def wait_for(self, predicate, timeout=4):
        deadline = time.monotonic()+timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.01)
            if predicate():
                return
        self.fail('Condition timed out; see /tmp/navigation_test_*.log')

    def set_params(self, **values):
        request = SetParametersAtomically.Request()
        request.parameters = [Parameter(k, value=v).to_parameter_msg() for k, v in values.items()]
        future = self.params.call_async(request)
        self.wait_for(future.done)
        self.assertTrue(future.result().result.successful, future.result().result.reason)

    def setUp(self):
        cls = type(self)
        self.stop_pub.publish(Empty())
        self.spin_for(0.15)
        self.set_params(move_head=False, motion_mode='omni', use_pot_fields=False, max_linear_speed=0.4,
                        head_pan_min=-3.14, head_pan_max=1.74,
                        debug_detection=os.environ.get('NAVIGATION_TEST_DEBUG') == '1',
                        fine_dist_tolerance=0.03, progress_timeout_sec=8.0,
                        head_alignment_timeout_sec=10.0)
        cls.rotation_wall = False
        cls.corner_obstacle = False
        cls.inflated_start = False
        cls.curved_escape = False
        cls.map_fill = 0
        cls.integrate = cls.head_auto = cls.blocked = False
        cls.publish_scan = True
        cls.cloud_points = 1
        cls.cloud_payload = array.array('B', struct.pack('<fff', 2.0, 0.0, 0.4))
        cls.cloud_period_sec = 0.02
        cls.last_cloud_sent = 0.0
        cls.monitor_times = []
        cls.monitor_ready = []
        cls.cloud_tf_lag = 0.10
        cls.cloud_stamp_offset = 0.0
        cls.scan_angle, cls.scan_range = 0.0, float('inf')
        cls.head_pan = 0.0
        cls.x = cls.y = cls.yaw = 0.0
        cls.command = Twist()
        self.cloud_enable_pub.publish(Bool(data=True))
        self.spin_for(0.15)
        cls.results.clear()
        cls.statuses.clear()
        cls.head_goals.clear()
        cls.head_records.clear()
        cls.plan_calls = 0
        cls.plan_requests.clear()
        cls.planner_stale_response = False
        cls.planner_failure = ''
        cls.planner_goal_offset = 0.0
        cls.planner_local_recovery = False
        cls.planner_recovery_forever = False
        cls.blocked_goal_position = None
        cls.short_escape = False
        cls.odom_jitter = 0.0
        cls.map_jitter = 0.0
        cls.freeze_odom = False

    @unittest.skipUnless(os.environ.get('NAVIGATION_TEST_CMD_VEL_RELAY') == '1',
                         'Requires the topic_tools relay fixture')
    def test_velocity_commands_reach_base_through_transient_relay(self):
        type(self).integrate = True
        self.send_path(0.3, 0.0)
        self.wait_for(lambda: self.x > 0.05, timeout=4)
        self.wait_for(lambda: any(r.status == 3 for r in self.results), timeout=8)
        self.assertAlmostEqual(self.x, 0.3, delta=0.04)
        self.assertFalse(any(r.status == 4 for r in self.results))

    def test_unknown_policy_controls_manager_dispatch_and_arrival(self):
        cls = type(self)
        cls.map_fill = -1
        cls.integrate = True
        self.spin_for(0.2)
        self.send_goal(0.6, 0.0)
        if self.allow_unknown:
            self.wait_for(lambda: any(s.status == 3 for s in self.statuses), timeout=10)
            self.assertAlmostEqual(self.x, 0.6, delta=0.04)
            self.assertEqual(self.plan_calls, 1)
            self.assertFalse(any('plan_invalid_on_current_map' in s.text for s in self.statuses))
        else:
            self.wait_for(lambda: any('plan_invalid_on_current_map' in s.text for s in self.statuses))
            self.assertFalse(self.results, 'Rejected plans must not reach the controller')
            self.assertAlmostEqual(self.x, 0.0)
            self.assertEqual(math.hypot(self.command.linear.x, self.command.linear.y), 0.0)

    def test_unknown_policy_controls_direct_controller_path(self):
        cls = type(self)
        cls.map_fill = -1
        cls.integrate = True
        self.spin_for(0.2)
        self.send_path(0.6, 0.0)
        self.wait_for(lambda: any(r.status in (3, 4) for r in self.results), timeout=10)
        terminal = next(r for r in self.results if r.status in (3, 4))
        self.assertEqual(terminal.status, 3 if self.allow_unknown else 4, terminal.text)
        self.assertAlmostEqual(self.x, 0.6 if self.allow_unknown else 0.0, delta=0.04)

    def test_unknown_permission_keeps_occupied_start_blocked(self):
        cls = type(self)
        cls.map_fill = -1
        cls.blocked = cls.integrate = True
        cls.x = 0.45
        self.spin_for(0.2)
        self.send_path(1.3, 0.0)
        self.wait_for(lambda: any(r.status == 4 for r in self.results))
        self.assertAlmostEqual(self.x, 0.45)
        self.assertEqual(math.hypot(self.command.linear.x, self.command.linear.y), 0.0)

    def test_unknown_permission_keeps_outside_map_blocked(self):
        cls = type(self)
        cls.map_fill = -1
        cls.integrate = True
        cls.x = 4.2
        self.spin_for(0.2)
        self.send_path(4.5, 0.0)
        self.wait_for(lambda: any(r.status == 4 for r in self.results))
        self.assertAlmostEqual(self.x, 4.2)
        self.assertEqual(math.hypot(self.command.linear.x, self.command.linear.y), 0.0)

    def test_local_start_recovery_replans_from_actual_exit(self):
        cls = type(self)
        cls.inflated_start = cls.integrate = cls.planner_local_recovery = True
        self.spin_for(0.1)
        self.send_goal(1.0, 0.0)
        self.wait_for(lambda: any(s.status == 3 for s in self.statuses), timeout=15)
        self.assertGreaterEqual(self.plan_calls, 2)
        self.assertAlmostEqual(self.plan_requests[0].start.pose.position.x, 0.0, places=3)
        self.assertGreaterEqual(self.plan_requests[1].start.pose.position.x, 0.19)
        self.assertTrue(any(s.text == 'start_recovered' for s in self.statuses))
        self.assertTrue(all(abs(r.goal.pose.position.x-1.0) < 1e-6 for r in self.plan_requests))

    def test_visual_path_is_retained_without_dispatching_a_duplicate_motion(self):
        self.send_goal(1.0, 0.0)
        self.wait_for(lambda: self.plan_calls == 1 and any(s.status == 1 for s in self.results))
        paths = []
        sub = self.node.create_subscription(Path, '/navigation/path', paths.append,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        try:
            self.wait_for(lambda: bool(paths and paths[-1].poses))
            self.assertEqual(paths[-1].header.frame_id, 'map')
            self.assertAlmostEqual(paths[-1].poses[-1].pose.position.x, 1.0)
            motion = f'{paths[-1].header.stamp.sec}.{paths[-1].header.stamp.nanosec}'
            self.assertTrue(any(s.goal_id.id == motion for s in self.results))
            self.spin_for(0.3)
            self.assertEqual(self.plan_calls, 1)
            self.assertFalse(any(s.status == 4 for s in self.results))
            self.stop_pub.publish(Empty())
            self.wait_for(lambda: bool(paths) and not paths[-1].poses)
        finally:
            self.node.destroy_subscription(sub)

    def test_runtime_diagnostics_report_occupied_physical_cell_before_return(self):
        self.set_params(debug_detection=True)
        type(self).blocked = True
        type(self).x = 0.45
        self.spin_for(0.2)
        motion = self.send_path(1.3, 0.0)
        self.wait_for(lambda: any(r.status == 4 for r in self.results))
        records = [line for line in FilePath('/tmp/navigation_test_simple_move.log').read_text().splitlines()
            if f'MotionDiagnostic motion={motion} ' in line]
        self.assertTrue(any('phase=waiting_for_local_clearance' in line and
            'local_reason=occupied_cell' in line and 'layer=physical' in line for line in records), records)
        self.assertEqual(math.hypot(self.command.linear.x,self.command.linear.y), 0.0)

    def test_manager_diagnostics_correlate_blocked_recovery_with_goal(self):
        request = SetParametersAtomically.Request()
        request.parameters = [Parameter('debug_detection', value=True).to_parameter_msg()]
        future = self.manager_params.call_async(request)
        self.wait_for(future.done)
        self.assertTrue(future.result().result.successful)
        try:
            type(self).planner_failure = 'start_blocked'
            self.send_goal(1.0,0.0)
            self.wait_for(lambda: any(s.status == 4 for s in self.statuses))
            goal = self.statuses[-1].goal_id.id
            log = FilePath('/tmp/navigation_test_mvn_pln.log').read_text()
            self.assertIn(f'NavigationDiagnostic goal={goal} ', log)
            self.assertIn(f'PlanDiagnostic goal={goal} reason=start_blocked', log)
            self.assertIn('reason=recovery_exhausted: start_blocked', log)
        finally:
            request.parameters = [Parameter('debug_detection', value=False).to_parameter_msg()]
            future = self.manager_params.call_async(request)
            self.wait_for(future.done)

    def test_curved_start_escape_tracks_physical_layer_at_low_speed(self):
        cls = type(self)
        cls.x = cls.y = 0.025
        cls.curved_escape = cls.integrate = True
        self.spin_for(0.1)
        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = self.node.get_clock().now().to_msg()
        for x,y in ((0.025,0.025),(0.025,0.125),(0.225,0.125),(0.225,0.025)):
            p = PoseStamped()
            p.pose.position.x,p.pose.position.y = x,y
            p.pose.orientation.w = 1.0
            path.poses.append(p)
        self.path_pub.publish(path)
        until = time.monotonic()+18
        while time.monotonic() < until and not any(s.status in (3,4) for s in self.results):
            rclpy.spin_once(self.node, timeout_sec=0.01)
            self.assertLessEqual(math.hypot(self.command.linear.x,self.command.linear.y), 0.081)
            self.assertFalse(0.05 <= self.x < 0.1 and -0.1 <= self.y < 0.1)
        self.assertTrue(any(s.status == 3 for s in self.results), [(s.status,s.text) for s in self.results])
        self.assertLess(math.hypot(self.x-0.225,self.y-0.025), 0.04)

    def test_recovery_paths_cannot_repeat_forever_without_global_plan(self):
        type(self).planner_recovery_forever = True
        self.send_goal(1.0, 0.0)
        self.wait_for(lambda: any(s.status == 4 for s in self.statuses), timeout=6)
        terminal = [s for s in self.statuses if s.status in (3,4)]
        self.assertEqual(len(terminal), 1)
        self.assertEqual((terminal[0].status, terminal[0].text), (4,'recovery_exhausted: start_blocked'))
        self.assertEqual(self.plan_calls, 4)

    def test_initial_blocked_goal_reaches_alternative_and_preserves_requested_goal(self):
        cls = type(self)
        cls.blocked_goal_position = (2.0, 0.0)
        cls.planner_goal_offset = -0.20
        cls.integrate = True
        self.spin_for(0.1)
        self.send_goal(2.0, 0.0, 1.0)
        self.wait_for(lambda: any(s.status in (3,4) for s in self.statuses), timeout=20)
        terminal = [s for s in self.statuses if s.status in (3,4)]
        self.assertEqual(len(terminal), 1)
        self.assertEqual((terminal[0].status, terminal[0].text), (3, 'reached_near_goal: goal_blocked'))
        self.assertLess(math.hypot(self.x-2.0, self.y), 0.33)
        self.assertLess(abs(math.remainder(self.yaw-1.0, 2*math.pi)), 0.06)
        self.assertAlmostEqual(self.plan_requests[0].goal.pose.position.x, 2.0)
        self.assertAlmostEqual(self.plan_requests[0].goal_recovery_radius, 0.33)
        cls.blocked_goal_position = None
        cls.planner_goal_offset = 0.0
        self.spin_for(0.1)
        previous = len(terminal)
        self.send_goal(2.0, 0.0, 1.0)
        self.wait_for(lambda: len([s for s in self.statuses if s.status in (3,4)]) > previous, timeout=10)
        terminal = [s for s in self.statuses if s.status in (3,4)]
        self.assertEqual(terminal[-1].text, 'goal_pose_reached')
        self.assertLess(math.hypot(self.x-2.0, self.y), 0.04)

    def test_alternative_at_radius_edge_does_not_finish_outside_region(self):
        cls = type(self)
        cls.blocked_goal_position = (2.0, 0.0)
        cls.planner_goal_offset = -0.32
        cls.integrate = True
        self.spin_for(0.1)
        self.send_goal(2.0, 0.0, 1.0)
        self.wait_for(lambda: any(s.status in (3,4) for s in self.statuses), timeout=20)
        terminal = next(s for s in self.statuses if s.status in (3,4))
        self.assertEqual((terminal.status,terminal.text), (3,'reached_near_goal: goal_blocked'))
        self.assertLess(math.hypot(self.x-2.0,self.y), 0.33)

    def test_late_blocked_goal_stops_and_aligns_before_single_success(self):
        cls = type(self)
        self.send_goal(2.0, 0.0, 1.0)
        self.wait_for(lambda: any(s.text == 'following_path' for s in self.statuses))
        cls.x = 1.90
        cls.blocked_goal_position = (2.0, 0.0)
        self.spin_for(0.2)
        cls.integrate = True
        self.wait_for(lambda: any(s.status in (3,4) for s in self.statuses), timeout=10)
        terminal = [s for s in self.statuses if s.status in (3,4)]
        self.assertEqual(len(terminal), 1)
        self.assertEqual((terminal[0].status, terminal[0].text), (3, 'reached_near_goal: goal_blocked'))
        self.assertLess(math.hypot(self.x-1.9, self.y), 0.04)
        self.assertLess(abs(self.yaw-1.0), 0.06)
        self.assertEqual(math.hypot(self.command.linear.x, self.command.linear.y), 0.0)

    def test_near_blocked_goal_requires_progress_and_absolute_radius(self):
        cls = type(self)
        for goal_x, robot_x in ((2.0, 1.75), (3.5, 3.15)):
            cls.x = 0.0
            cls.blocked_goal_position = None
            self.spin_for(0.1)
            self.send_goal(goal_x, 0.0)
            self.wait_for(lambda: any(s.text == 'following_path' for s in self.statuses))
            cls.x = robot_x
            cls.blocked_goal_position = (goal_x, 0.0)
            self.spin_for(0.8)
            self.assertFalse(any(s.status == 3 for s in self.statuses))
            self.assertFalse(any(s.text == 'aligning_near_blocked_goal' for s in self.statuses))
            self.stop_pub.publish(Empty())
            self.spin_for(0.15)
            cls.statuses.clear()

    def test_blocked_plan_retries_are_bounded_even_with_patience(self):
        cls = type(self)
        for reason in ('start_blocked','goal_blocked'):
            cls.planner_failure = reason
            calls = self.plan_calls
            self.send_goal(1.0, 0.0)
            self.wait_for(lambda: any(s.status == 4 for s in self.statuses), timeout=6)
            self.assertEqual(self.plan_calls-calls, 3)
            terminal = [s for s in self.statuses if s.status in (3,4)]
            self.assertEqual(len(terminal), 1)
            self.assertEqual(terminal[0].text, 'recovery_exhausted: '+reason)
            cls.statuses.clear()

    def test_free_goal_cannot_be_silently_relocated(self):
        type(self).planner_goal_offset = -0.20
        self.send_goal(1.0, 0.0)
        self.wait_for(lambda: any('planner_changed_goal' in s.text for s in self.statuses))
        self.assertFalse(any(s.text == 'following_path' or s.status == 3 for s in self.statuses))

    def test_approximate_success_rechecks_orientation_after_controller_result(self):
        cls = type(self)
        cls.blocked_goal_position = (2.0, 0.0)
        cls.planner_goal_offset = -0.20
        self.spin_for(0.1)
        self.send_goal(2.0, 0.0, 1.0)
        self.wait_for(lambda: any(s.status == 1 for s in self.results))
        active = next(s for s in self.results if s.status == 1)
        cls.x = 1.8
        forged = GoalStatus()
        forged.goal_id = active.goal_id
        forged.status = 3
        self.result_pub.publish(forged)
        self.spin_for(0.7)
        self.assertFalse(any(s.status == 3 for s in self.statuses))
        cls.yaw = 1.0
        self.wait_for(lambda: any(s.status == 3 for s in self.statuses))
        self.assertEqual(next(s.text for s in self.statuses if s.status == 3), 'reached_near_goal: goal_blocked')

    def test_stationary_odometry_jitter_finishes_without_progress_timeout(self):
        type(self).odom_jitter = 0.002
        self.spin_for(0.1)
        started = time.monotonic()
        self.send_path(0.0, 0.0)
        self.wait_for(lambda: any(r.status in (3, 4) for r in self.results), timeout=1.5)
        terminal = next(r for r in self.results if r.status in (3, 4))
        self.assertEqual((terminal.status, terminal.text), (3, 'pose_reached'))
        self.assertLess(time.monotonic()-started, 1.0)

    def test_repeated_odometry_transform_cannot_confirm_rest(self):
        type(self).freeze_odom = True
        self.send_path(0.0, 0.0)
        self.spin_for(0.35)
        self.assertFalse(any(r.status == 3 for r in self.results))
        self.wait_for(lambda: any(r.status == 4 for r in self.results), timeout=1.0)
        self.assertEqual(next(r.text for r in self.results if r.status == 4), 'robot_tf_unavailable')

    def test_short_recovery_leaves_inflation_before_finishing(self):
        cls = type(self)
        cls.short_escape = cls.integrate = True
        cls.x = cls.y = 0.025
        self.set_params(fine_dist_tolerance=0.08)
        self.spin_for(0.15)
        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = self.node.get_clock().now().to_msg()
        for x in (0.025, 0.075):
            pose = PoseStamped()
            pose.pose.position.x, pose.pose.position.y = x, 0.025
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        command = NavigationPath(path=path, mode=NavigationPath.RECOVER_START)
        self.navigation_path_pub.publish(command)
        self.wait_for(lambda: any(r.status in (3, 4) for r in self.results), timeout=5)
        self.assertEqual(next(r.status for r in self.results if r.status in (3, 4)), 3)
        self.assertGreaterEqual(self.x, 0.05)
        self.assertLess(self.x, 0.10)

    def test_control_stalls_retry_without_excluding_valid_routes(self):
        self.send_goal(1.0, 0.0)
        consumed = set()
        for reason in ('no_progress', 'position_stalled', 'orientation_stalled', 'head_alignment_timeout'):
            self.wait_for(lambda: any(r.status == 1 and r.goal_id.id not in consumed
                                     for r in self.results), timeout=3)
            active = next(r for r in reversed(self.results) if r.status == 1 and r.goal_id.id not in consumed)
            consumed.add(active.goal_id.id)
            self.result_pub.publish(GoalStatus(goal_id=active.goal_id, status=4, text=reason))
            self.spin_for(0.15)
        self.wait_for(lambda: any(s.status == 4 for s in self.statuses))
        self.assertEqual(next(s.text for s in self.statuses if s.status == 4),
                         'control_retries_exhausted: head_alignment_timeout')
        self.assertTrue(all(not r.avoided_paths for r in self.plan_requests))
        self.assertEqual(self.plan_calls, 4)

    def test_avoidance_does_not_alternate_head_targets_each_control_tick(self):
        cls = type(self)
        cls.blocked = cls.integrate = cls.head_auto = True
        self.set_params(move_head=True)
        self.spin_for(0.1)
        self.send_path(1.0, 0.0)
        self.spin_for(6.0)
        flips = [(b[0]-a[0], b[1]-a[1]) for a, b in zip(cls.head_records, cls.head_records[1:])
                 if b[0]-a[0] < 0.12 and abs(b[1]-a[1]) > 0.4]
        self.assertLessEqual(len(flips), 4, flips)
        self.assertGreater(math.hypot(self.x, self.y), 0.15)
        self.assertFalse(any(r.status == 4 for r in self.results))
        self.wait_for(lambda: any(r.status in (3, 4) for r in self.results), timeout=20)
        self.assertEqual(next(r.status for r in self.results if r.status in (3, 4)), 3)
        self.assertLess(math.hypot(self.x-1.0, self.y), 0.04)

    def test_short_goal_with_large_rotation_and_head_finishes(self):
        cls = type(self)
        cls.integrate = cls.head_auto = True
        self.set_params(move_head=True)
        self.send_goal(0.30, 0.0, 1.5)
        self.wait_for(lambda: any(s.status in (3, 4) for s in self.statuses), timeout=12)
        self.assertEqual(next(s.status for s in self.statuses if s.status in (3, 4)), 3)
        self.assertLess(math.hypot(self.x-0.30, self.y), 0.031)
        self.assertLess(abs(math.remainder(self.yaw-1.5, 2*math.pi)), 0.051)

    def test_terminal_yaw_progress_counts_before_position_tolerance(self):
        cls = type(self)
        cls.x = 0.15
        self.set_params(progress_timeout_sec=0.5)
        self.spin_for(0.1)
        self.send_path(0.30, 0.0, 1.5)
        for i in range(20):
            cls.yaw = (i+1)*0.03
            self.spin_for(0.1)
            self.assertFalse(any(r.status == 4 for r in self.results))
        self.wait_for(lambda: any(r.status == 4 for r in self.results), timeout=1.5)
        self.assertEqual(next(r.text for r in self.results if r.status == 4), 'position_stalled')

    def test_terminal_position_jitter_keeps_head_reference(self):
        cls = type(self)
        cls.head_auto = True
        self.set_params(move_head=True)
        cls.x = 0.98
        self.spin_for(0.1)
        self.send_path(1.0, 0.0, 1.0)
        self.spin_for(0.2)
        cls.head_goals.clear()
        cls.yaw, cls.x = 1.0, 1.04
        self.spin_for(0.15)
        self.assertFalse(self.head_goals, 'A brief terminal pose excursion must not restart head tracking')
        cls.x = 0.98
        self.wait_for(lambda: any(s.status == 3 for s in self.results))

    def send_path(self, x, y, yaw=0.0):
        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = self.node.get_clock().now().to_msg()
        for i in range(41):
            p = PoseStamped()
            p.pose.position.x = x*i/40
            p.pose.position.y = y*i/40
            p.pose.orientation.z, p.pose.orientation.w = math.sin(yaw/2), math.cos(yaw/2)
            path.poses.append(p)
        self.path_pub.publish(path)
        return f'{path.header.stamp.sec}.{path.header.stamp.nanosec}'

    def send_goal(self, x=0.0, y=0.0, yaw=0.0):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x, goal.pose.position.y = x, y
        goal.pose.orientation.z, goal.pose.orientation.w = math.sin(yaw/2), math.cos(yaw/2)
        self.goal_pub.publish(goal)

    def test_repeated_goal_is_evaluated_each_time(self):
        self.send_goal()
        self.wait_for(lambda: any(s.status == 3 for s in self.statuses))
        first = next(s.goal_id.id for s in self.statuses if s.status == 3)
        self.send_goal()
        self.wait_for(lambda: any(s.status == 3 and s.goal_id.id != first for s in self.statuses))
        self.assertEqual(self.plan_calls, 0)

    def test_repeated_completed_goal_replans_after_robot_moves(self):
        self.send_goal()
        self.wait_for(lambda: any(s.status == 3 for s in self.statuses))
        type(self).x = 0.4
        self.spin_for(0.15)
        self.send_goal()
        self.wait_for(lambda: self.plan_calls >= 1)

    def test_invalid_parameter_batch_does_not_disable_head(self):
        self.set_params(move_head=True)
        request = SetParametersAtomically.Request()
        request.parameters = [Parameter('move_head', value=False).to_parameter_msg(),
                              Parameter('max_linear_speed', value=-1.0).to_parameter_msg()]
        future = self.params.call_async(request)
        self.wait_for(future.done)
        self.assertFalse(future.result().result.successful)
        type(self).head_pan = -1.0
        self.send_path(0, 1)
        self.wait_for(lambda: bool(self.head_goals))
        self.spin_for(0.15)
        self.assertEqual(self.command.linear.y, 0.0)

    def test_runtime_enabling_head_stops_until_aligned(self):
        self.send_path(0, 1)
        self.wait_for(lambda: self.command.linear.y > 0.05)
        type(self).head_pan = -1.0
        self.set_params(move_head=True)
        self.wait_for(lambda: bool(self.head_goals) and self.command.linear.y == 0.0)
        type(self).head_pan = math.pi/2
        self.wait_for(lambda: self.command.linear.y > 0.05)

    def test_persistent_path_deviation_requests_replan(self):
        self.send_path(1, 0)
        self.wait_for(lambda: self.command.linear.x > 0.02)
        type(self).y = 0.5
        self.wait_for(lambda: any(r.status == 4 and r.text == 'path_deviation' for r in self.results))
        self.assertEqual(self.command.linear.x, 0.0)

    def test_same_coordinates_wrong_yaw_rotates_without_plan(self):
        type(self).integrate = True
        self.send_goal(yaw=0.3)
        self.wait_for(lambda: abs(self.command.angular.z) > 0.05)
        self.assertAlmostEqual(self.command.linear.x, 0.0)
        self.assertAlmostEqual(self.command.linear.y, 0.0)
        self.wait_for(lambda: any(s.status == 3 for s in self.statuses), timeout=8)
        self.assertEqual(self.plan_calls, 0)
        self.assertLess(abs(self.yaw-0.3), 0.06)

    def test_orientation_correction_keeps_original_position_reference(self):
        type(self).integrate = True
        self.send_goal(x=0.02, yaw=0.5)
        self.wait_for(lambda: abs(self.command.angular.z) > 0.05)
        type(self).x = -0.02
        self.wait_for(lambda: any(s.status == 3 for s in self.statuses), timeout=10)
        self.assertLess(abs(self.x-0.02), 0.03)
        self.assertEqual(self.plan_calls, 0)

    def test_omni_can_translate_laterally(self):
        self.send_path(0, 1)
        self.wait_for(lambda: self.command.linear.y > 0.05)

    def test_short_omni_transfer_keeps_already_correct_base_heading(self):
        self.set_params(move_head=True)
        type(self).head_auto = type(self).integrate = True
        self.send_path(0.0, 0.6, 0.0)
        self.wait_for(lambda: self.y > 0.25, timeout=8)
        self.assertLess(abs(self.yaw), 0.10)
        self.wait_for(lambda: any(r.status == 3 for r in self.results), timeout=8)
        self.assertLess(abs(self.yaw), 0.05)

    def test_high_speed_straight_reaches_launch_limit(self):
        self.set_params(max_linear_speed=0.9)
        type(self).integrate = True
        self.send_path(3.2, 0)
        self.wait_for(lambda: self.command.linear.x > 0.85, timeout=8)
        self.wait_for(lambda: any(r.status == 3 for r in self.results), timeout=12)
        self.assertLess(abs(self.x-3.2), 0.04)

    def test_short_omni_route_interpolates_to_goal_without_turning_away(self):
        # Initial/final yaw and the 65 cm displacement from the supplied log.
        cls = type(self)
        cls.yaw = 2.158
        cls.integrate = cls.head_auto = True
        self.set_params(move_head=True)
        self.spin_for(0.1)
        path = Path()
        path.header.frame_id, path.header.stamp = 'map', self.node.get_clock().now().to_msg()
        for x, y in ((0.0, 0.0), (0.2, -0.04), (0.6, 0.25)):
            pose = PoseStamped()
            pose.pose.position.x, pose.pose.position.y = x, y
            pose.pose.orientation.z, pose.pose.orientation.w = math.sin(2.731/2), math.cos(2.731/2)
            path.poses.append(pose)
        self.path_pub.publish(path)
        minimum_yaw, maximum_yaw = cls.yaw, cls.yaw
        deadline = time.monotonic()+18
        while time.monotonic() < deadline and not any(r.status in (3, 4) for r in self.results):
            rclpy.spin_once(self.node, timeout_sec=0.01)
            minimum_yaw, maximum_yaw = min(minimum_yaw, cls.yaw), max(maximum_yaw, cls.yaw)
        self.assertTrue(any(r.status == 3 for r in self.results), [(r.status, r.text) for r in self.results])
        self.assertGreater(minimum_yaw, 2.0)
        self.assertLess(maximum_yaw, 2.9)
        self.assertLess(math.hypot(self.x-0.6, self.y-0.25), 0.04)

    def test_map_localization_jitter_does_not_repeatedly_stop_aligned_motion(self):
        cls = type(self)
        self.set_params(move_head=True)
        cls.integrate = cls.head_auto = True
        self.send_path(1.5, 0.0)
        self.wait_for(lambda: self.x > 0.15, timeout=8)
        cls.map_jitter = 0.006  # 12 mm jumps in map; odom remains continuous.
        longest_stop = 0.0
        stopped_since = None
        deadline = time.monotonic()+12
        while time.monotonic() < deadline and self.x < 1.2:
            rclpy.spin_once(self.node, timeout_sec=0.01)
            if math.hypot(self.command.linear.x, self.command.linear.y) < 0.005:
                if stopped_since is None:
                    stopped_since = time.monotonic()
                longest_stop = max(longest_stop, time.monotonic()-stopped_since)
            else:
                stopped_since = None
        self.assertGreaterEqual(self.x, 1.2)
        self.assertLess(longest_stop, 0.15)
        self.assertFalse(any(r.status == 4 for r in self.results))

    def test_local_map_detour_recovers_original_path_without_aborting(self):
        type(self).blocked = True
        type(self).integrate = True
        self.spin_for(0.1)
        self.send_path(1.3, 0)
        self.wait_for(lambda: any(r.status in (3, 4) for r in self.results), timeout=20)
        self.assertTrue(any(r.status == 3 for r in self.results),
                        [(self.x,self.y,self.yaw)]+[(r.status,r.text) for r in self.results])
        self.assertLess(abs(self.y), 0.04)

    def test_occupied_start_cannot_be_cleared_by_local_recovery(self):
        type(self).blocked = True
        type(self).x = 0.45
        self.spin_for(0.2)
        self.send_path(1.3, 0)
        self.wait_for(lambda: any(r.status == 4 for r in self.results))
        self.assertEqual(self.command.linear.x, 0.0)
        self.assertEqual(self.command.linear.y, 0.0)

    def test_curved_route_stays_close_and_reaches_final_pose(self):
        type(self).integrate = True
        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = self.node.get_clock().now().to_msg()
        for i in range(61):
            angle = math.pi/2*i/60
            p = PoseStamped()
            p.pose.position.x = 0.6*math.sin(angle)
            p.pose.position.y = 0.6*(1-math.cos(angle))
            p.pose.orientation.z, p.pose.orientation.w = math.sin(angle/2), math.cos(angle/2)
            path.poses.append(p)
        self.path_pub.publish(path)
        deadline = time.monotonic()+15
        maximum_error = 0.0
        while time.monotonic() < deadline and not any(r.status in (3, 4) for r in self.results):
            rclpy.spin_once(self.node, timeout_sec=0.01)
            maximum_error = max(maximum_error, abs(math.hypot(self.x, self.y-0.6)-0.6))
        self.assertTrue(any(r.status == 3 for r in self.results), [(r.status, r.text) for r in self.results])
        self.assertLess(maximum_error, 0.15)
        self.assertLess(math.hypot(self.x-0.6, self.y-0.6), 0.04)
        self.assertLess(abs(self.yaw-math.pi/2), 0.06)

    def test_differential_never_commands_lateral_velocity(self):
        self.set_params(motion_mode='differential')
        self.send_path(0, 1)
        self.wait_for(lambda: self.command.angular.z > 0.05)
        self.spin_for(0.4)
        self.assertEqual(self.command.linear.y, 0.0)

    def test_curve_with_head_tracking_does_not_repeatedly_stop(self):
        self.set_params(move_head=True)
        type(self).integrate = type(self).head_auto = True
        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = self.node.get_clock().now().to_msg()
        for i in range(81):
            angle = math.pi/2*i/80
            p = PoseStamped()
            p.pose.position.x = 0.8*math.sin(angle)
            p.pose.position.y = 0.8*(1-math.cos(angle))
            p.pose.orientation.z, p.pose.orientation.w = math.sin(angle/2), math.cos(angle/2)
            path.poses.append(p)
        self.path_pub.publish(path)
        self.wait_for(lambda: self.x > 0.15, timeout=10)
        deadline = time.monotonic()+12
        stopped_since = None
        longest_stop = 0.0
        while time.monotonic() < deadline and math.hypot(self.x-0.8, self.y-0.8) > 0.12:
            rclpy.spin_once(self.node, timeout_sec=0.01)
            if math.hypot(self.command.linear.x, self.command.linear.y) < 0.005:
                if stopped_since is None:
                    stopped_since = time.monotonic()
                longest_stop = max(longest_stop, time.monotonic()-stopped_since)
            else:
                stopped_since = None
        self.assertLess(math.hypot(self.x-0.8, self.y-0.8), 0.13)
        self.assertLess(longest_stop, 0.15)
        self.assertFalse(any(r.status == 4 for r in self.results))

    def test_moderate_head_lag_slows_without_freezing_base(self):
        self.set_params(move_head=True)
        self.send_path(1, 0)
        self.wait_for(lambda: self.command.linear.x > 0.15)
        type(self).head_pan = -0.25
        self.spin_for(0.5)
        self.assertGreater(self.command.linear.x, 0.01)
        self.assertFalse(any(r.status == 4 for r in self.results))

    def test_final_position_noise_does_not_restart_translation(self):
        self.send_path(0.3, 0, 0.8)
        self.wait_for(lambda: self.command.linear.x > 0.02)
        type(self).x = 0.275
        # Let the acceleration/jerk-limited command stop before injecting jitter.
        self.wait_for(lambda: math.hypot(self.command.linear.x, self.command.linear.y) == 0.0,
                      timeout=0.8)
        for x in (0.265, 0.276, 0.266, 0.275):
            type(self).x = x
            self.spin_for(0.1)
            self.assertEqual(self.command.linear.x, 0.0)
            self.assertEqual(self.command.linear.y, 0.0)
        self.assertGreater(abs(self.command.angular.z), 0.05)
        type(self).x = 0.23
        # After the larger positional drift, let orientation settle too; the
        # approach intentionally limits translation while angular error is large.
        type(self).yaw = 0.8
        self.wait_for(lambda: self.command.linear.x > 0.01)

    def test_stale_planner_response_cannot_dispatch_motion(self):
        type(self).planner_stale_response = True
        self.send_goal(x=1.0)
        self.wait_for(lambda: self.plan_calls >= 2)
        self.assertFalse(any(s.text == 'following_path' for s in self.statuses))
        self.assertEqual(self.command.linear.x, 0.0)
        self.assertEqual(self.plan_requests[0].minimum_map_stamp,
                         self.plan_requests[-1].minimum_map_stamp)
        type(self).planner_stale_response = False
        self.wait_for(lambda: any(s.text == 'following_path' for s in self.statuses))

    def test_repeated_blockage_stops_identical_recovery_and_resets_for_new_goal(self):
        self.send_goal(x=1.0)
        previous_id = ''
        for attempt in range(2):
            self.wait_for(lambda: any(r.status == 1 and r.goal_id.id != previous_id for r in self.results))
            active = next(r for r in reversed(self.results) if r.status == 1)
            previous_id = active.goal_id.id
            if attempt == 1:
                # Route exclusions need current obstacle evidence, not just a controller result.
                type(self).blocked = True
                self.spin_for(0.1)
            result = GoalStatus()
            result.goal_id, result.status, result.text = active.goal_id, 4, 'local_trajectory_blocked'
            self.result_pub.publish(result)
            self.spin_for(0.2)
        self.wait_for(lambda: any(s.text.startswith('recovery_exhausted') for s in self.statuses), timeout=7)
        self.assertTrue(any(r.avoided_paths for r in self.plan_requests))
        self.assertEqual(self.command.linear.x, 0.0)
        type(self).blocked = False
        self.spin_for(0.1)
        self.send_goal(x=1.0)
        self.wait_for(lambda: not self.plan_requests[-1].avoided_paths)
        self.wait_for(lambda: self.command.linear.x > 0.02)

    def test_head_gates_translation_and_runtime_disable_releases_it(self):
        self.set_params(move_head=True)
        type(self).head_pan = -1.0
        self.send_path(0, 1)
        self.wait_for(lambda: bool(self.head_goals))
        self.spin_for(0.3)
        self.assertEqual(self.command.linear.x, 0.0)
        self.assertEqual(self.command.linear.y, 0.0)
        self.set_params(move_head=False)
        self.wait_for(lambda: self.command.linear.y > 0.05)
        count = len(self.head_goals)
        self.spin_for(0.2)
        self.assertEqual(len(self.head_goals), count)

    def test_head_out_of_range_rotates_base_before_translation(self):
        self.set_params(move_head=True, head_pan_min=-0.5, head_pan_max=0.5)
        self.send_path(0, 1)
        self.wait_for(lambda: self.command.angular.z > 0.05)
        self.assertEqual(self.command.linear.x, 0.0)
        self.assertEqual(self.command.linear.y, 0.0)
        self.assertLessEqual(self.head_goals[-1].data[0], 0.5)

    def test_head_limit_recovery_can_continue_translation(self):
        self.set_params(move_head=True, head_pan_min=-0.5, head_pan_max=0.5)
        type(self).integrate = type(self).head_auto = True
        self.send_path(0, 1.0, math.pi/2)
        self.wait_for(lambda: self.y > 0.15, timeout=12)
        self.assertTrue(all(-0.5 <= h.data[0] <= 0.5 for h in self.head_goals))

    def test_short_omni_goal_finishes_with_restricted_head_pan(self):
        self.set_params(move_head=True, head_pan_min=-0.5, head_pan_max=0.5)
        type(self).integrate = type(self).head_auto = True
        self.send_path(0.0, 0.6, 0.0)
        self.wait_for(lambda: any(r.status in (3,4) for r in self.results), timeout=18)
        self.assertTrue(any(r.status == 3 for r in self.results), [(r.status,r.text) for r in self.results])
        self.assertLess(abs(self.yaw), 0.06)

    def test_final_orientation_begins_before_destination(self):
        type(self).integrate = True
        self.send_path(0.6, 0.0, 0.6)
        self.wait_for(lambda: self.x > 0.42 and self.yaw > 0.05, timeout=12)
        self.assertLess(self.x, 0.60)
        self.wait_for(lambda: self.x > 0.565, timeout=12)
        self.assertLess(abs(self.yaw-0.6), 0.15)
        self.wait_for(lambda: any(r.status == 3 for r in self.results), timeout=12)
        self.assertLess(abs(self.yaw-0.6), 0.06)

    def test_blocked_route_requests_new_plan(self):
        self.send_goal(x=1.0)
        self.wait_for(lambda: self.plan_calls >= 1)
        type(self).blocked = True
        self.wait_for(lambda: any('route_blocked' in s.text or 'local_trajectory_blocked' in s.text for s in self.statuses))
        self.wait_for(lambda: self.plan_calls >= 2, timeout=5)
        self.assertAlmostEqual(self.command.linear.x, 0.0)

    def test_old_controller_result_cannot_complete_new_goal(self):
        self.send_goal(x=1.0)
        self.wait_for(lambda: any(r.status == 1 for r in self.results))
        old = next(r.goal_id for r in self.results if r.status == 1)
        self.send_goal(y=1.0)
        self.spin_for(0.3)
        message = GoalStatus()
        message.goal_id, message.status = old, 3
        self.result_pub.publish(message)
        self.spin_for(0.3)
        self.assertFalse(any(s.status == 3 for s in self.statuses))

    def test_relative_motion_enables_its_obstacle_monitor(self):
        self.enable_pub.publish(Bool(data=False))
        self.set_params(use_pot_fields=True)
        type(self).integrate = True
        command = Float32MultiArray()
        command.data = [0.15, 0.0]
        self.relative_pub.publish(command)
        self.wait_for(lambda: self.command.linear.x > 0.02)
        self.wait_for(lambda: any(r.status == 3 and r.goal_id.id == '-1' for r in self.results), timeout=8)

    def test_initial_base_alignment_uses_continuous_adaptive_window(self):
        type(self).integrate = True
        self.send_path(0, 1.0, math.pi/2)
        self.wait_for(lambda: self.y > 0.05, timeout=12)
        stopped = 0
        until = time.monotonic()+12
        while self.y < 0.5 and time.monotonic() < until:
            rclpy.spin_once(self.node, timeout_sec=0.01)
            stopped += math.hypot(self.command.linear.x, self.command.linear.y) < 0.01
        self.assertGreaterEqual(self.y, 0.5)
        self.assertLess(stopped, 3)
        self.assertLess(abs(self.yaw-math.pi/2), 0.25)

    def test_preventive_zone_reduces_speed_without_aborting(self):
        self.set_params(use_pot_fields=True)
        self.enable_pub.publish(Bool(data=True))
        type(self).scan_angle, type(self).scan_range = 0.0, 0.45
        self.send_path(1, 0)
        self.wait_for(lambda: self.command.linear.x > 0.03)
        self.spin_for(0.7)
        self.assertFalse(self.risk)
        self.assertLess(self.obstacle_limit, 0.3)
        self.assertLessEqual(math.hypot(self.command.linear.x, self.command.linear.y), self.obstacle_limit+0.01)
        self.assertFalse(any(r.status == 4 for r in self.results))

    def test_rotation_near_preventive_margin_does_not_abort(self):
        self.set_params(use_pot_fields=True)
        type(self).integrate = True
        type(self).scan_angle, type(self).scan_range = math.pi/2, 0.36
        self.send_goal(yaw=0.3)
        self.wait_for(lambda: abs(self.command.angular.z) > 0.05)
        self.wait_for(lambda: any(s.status == 3 for s in self.statuses), timeout=8)
        self.assertFalse(any(r.text == 'collision_risk' for r in self.results))

    def test_critical_obstacle_still_stops_immediately(self):
        self.set_params(use_pot_fields=True)
        self.enable_pub.publish(Bool(data=True))
        self.send_path(1, 0)
        self.wait_for(lambda: self.command.linear.x > 0.03)
        type(self).scan_angle, type(self).scan_range = 0.0, 0.24
        self.wait_for(lambda: any(r.status == 4 and r.text == 'collision_risk' for r in self.results))
        self.assertEqual(self.command.linear.x, 0.0)

    def test_local_check_uses_braking_distance_instead_of_constant_speed(self):
        type(self).blocked = True
        self.spin_for(0.1)
        self.send_path(1, 0)
        self.wait_for(lambda: self.command.linear.x > 0.35)
        self.assertFalse(any(r.status == 4 for r in self.results))

    def test_preventive_force_precedes_immediate_collision(self):
        self.enable_pub.publish(Bool(data=True))
        self.intent_pub.publish(Vector3(x=1.0))
        type(self).scan_angle, type(self).scan_range = 0.0, 0.65
        self.wait_for(lambda: self.force.x < -0.01 and self.ready is True and self.risk is False)
        self.assertFalse(self.risk)
        type(self).scan_range = 0.24
        self.wait_for(lambda: self.risk is True)

    def test_hsr_self_points_are_ignored_but_critical_exterior_is_not(self):
        self.enable_pub.publish(Bool(data=True))
        self.intent_pub.publish(Vector3(x=1.0))
        type(self).scan_range = 0.20
        self.wait_for(lambda: self.ready is True)
        self.spin_for(0.2)
        self.assertFalse(self.risk)
        type(self).scan_range = 0.24
        self.wait_for(lambda: self.risk is True)

    def test_dense_cloud_keeps_monitor_and_lidar_responsive(self):
        cls = type(self)
        for label, qos in [('best_effort', qos_profile_sensor_data), ('reliable', 10)]:
            with self.subTest(publisher=label):
                self.node.destroy_publisher(cls.cloud_pub)
                cls.cloud_pub = self.node.create_publisher(PointCloud2, '/points', qos)
                cls.results.clear()
                cls.cloud_points = 1
                cls.cloud_payload = array.array('B', struct.pack('<fff', 2.0, 0.0, 0.4))
                cls.cloud_period_sec = 0.02
                self.enable_pub.publish(Bool(data=True))
                self.set_params(use_pot_fields=True)
                self.wait_for(lambda: self.ready is True and self.risk is False)
                self.send_path(1.0, 0.0)
                self.wait_for(lambda: self.command.linear.x > 0.03)
                cls.cloud_points = 640*480
                cls.cloud_payload = array.array('B', struct.pack('<fff', 2.0, 0.0, 0.4)*cls.cloud_points)
                cls.cloud_period_sec = 0.05
                # More than 100 cloud points are required for a finite limit;
                # this proves a full VGA observation was actually processed.
                self.wait_for(lambda: math.isfinite(self.obstacle_limit) and self.obstacle_limit>0.5, timeout=6)
                cls.monitor_times.clear()
                cls.monitor_ready.clear()
                cls.scan_angle, cls.scan_range = 0.0, 0.65
                self.wait_for(lambda: self.force.x < -0.01)
                self.spin_for(3.0)
                self.assertGreater(len(self.monitor_times), 40)
                gaps = [b-a for a, b in zip(self.monitor_times, self.monitor_times[1:])]
                self.assertLess(max(gaps), 0.3)
                failures = [r.text for r in self.results if r.status == 4]
                if failures:
                    # Best-effort DDS may lose large fragmented samples. Loss
                    # must stop translation while the monitor/LiDAR stay live.
                    self.assertTrue(all(r == 'obstacle_data_stale' for r in failures), failures)
                    self.assertIn(False, self.monitor_ready)
                    self.assertEqual(self.command.linear.x, 0.0)
                elif self.ready is False:
                    # A recent dropout can still be in the bounded sensor wait,
                    # before ABORTED. Verify both the stop and automatic recovery.
                    self.assertEqual(math.hypot(self.command.linear.x, self.command.linear.y), 0.0)
                    self.assertIn('waiting_for_obstacle_data', [r.text for r in self.results])
                    cls.cloud_points = 1
                    cls.cloud_payload = array.array('B', struct.pack('<fff', 2.0, 0.0, 0.4))
                    cls.cloud_period_sec = 0.02
                    self.wait_for(lambda: self.ready is True and self.command.linear.x > 0.03)
                else:
                    self.assertGreater(self.command.linear.x, 0.03)
                print(f' VGA cloud load ({label}): maximum monitor gap {max(gaps):.3f}s', end='')
                self.stop_pub.publish(Empty())
                self.enable_pub.publish(Bool(data=False))
                self.spin_for(0.15)
                cls.scan_range = float('inf')
        self.node.destroy_publisher(cls.cloud_pub)
        cls.cloud_pub = self.node.create_publisher(PointCloud2, '/points', qos_profile_sensor_data)

    def test_repeated_cloud_enable_preserves_subscription_and_validation(self):
        cls = type(self)
        self.enable_pub.publish(Bool(data=True))
        self.wait_for(lambda: self.ready is True)
        cls.monitor_ready.clear()
        for _ in range(20):
            self.cloud_enable_pub.publish(Bool(data=True))
            self.spin_for(0.05)
        self.assertGreater(len(self.monitor_ready), 20)
        self.assertTrue(all(self.monitor_ready))
        # Prove that cloud processing continues, not only the ready heartbeat.
        cls.cloud_payload = array.array('B', struct.pack('<fff', *(float('nan'),)*3))
        self.wait_for(lambda: self.ready is False)
        cls.cloud_payload = array.array('B', struct.pack('<fff', 2.0, 0.0, 0.4))
        self.wait_for(lambda: self.ready is True)

    def test_cloud_toggle_and_monitor_restart_keep_callbacks_live(self):
        cls = type(self)
        self.enable_pub.publish(Bool(data=True))
        self.wait_for(lambda: self.ready is True)
        for _ in range(2):
            self.cloud_enable_pub.publish(Bool(data=False))
            self.spin_for(0.1)
            self.cloud_enable_pub.publish(Bool(data=False))
            cls.cloud_payload = array.array('B', struct.pack('<fff', *(float('nan'),)*3))
            self.spin_for(0.6)
            self.assertTrue(self.ready, 'Disabled cloud must leave the LiDAR monitor responsive')
            self.cloud_enable_pub.publish(Bool(data=True))
            self.wait_for(lambda: self.ready is False)
            cls.cloud_payload = array.array('B', struct.pack('<fff', 2.0, 0.0, 0.4))
            self.wait_for(lambda: self.ready is True)
            self.enable_pub.publish(Bool(data=False))
            self.wait_for(lambda: self.ready is False)
            self.enable_pub.publish(Bool(data=True))
            self.wait_for(lambda: self.ready is True)

    def test_recent_cloud_tf_fallback_allows_sensor_readiness(self):
        self.enable_pub.publish(Bool(data=True))
        # Separate topics can arrive in different spin iterations.
        self.wait_for(lambda: self.ready is True and self.risk is False)
        self.set_params(use_pot_fields=True)
        self.send_path(0.3, 0)
        self.wait_for(lambda: self.command.linear.x > 0.02)
        self.assertFalse(any(r.text == 'obstacle_data_stale' for r in self.results))

    def test_stale_cloud_tf_fallback_is_rejected_then_recovers(self):
        self.enable_pub.publish(Bool(data=True))
        self.wait_for(lambda: self.ready is True)
        type(self).cloud_tf_lag = 0.40
        self.wait_for(lambda: self.ready is False)
        type(self).cloud_tf_lag = 0.10
        self.wait_for(lambda: self.ready is True)

    def test_old_cloud_stamp_is_rejected_then_recovers(self):
        self.enable_pub.publish(Bool(data=True))
        self.wait_for(lambda: self.ready is True)
        type(self).cloud_stamp_offset = 0.7
        self.wait_for(lambda: self.ready is False)
        type(self).cloud_stamp_offset = 0.0
        self.wait_for(lambda: self.ready is True)

    def test_lateral_obstacle_is_detected_and_stale_sensor_stops(self):
        self.enable_pub.publish(Bool(data=True))
        self.intent_pub.publish(Vector3(y=1.0))
        type(self).scan_angle, type(self).scan_range = math.pi/2, 0.24
        self.wait_for(lambda: self.risk is True)
        type(self).scan_range = float('inf')
        self.wait_for(lambda: self.risk is False)
        type(self).publish_scan = False
        self.wait_for(lambda: self.ready is False and self.obstacle_limit == 0.0)

    def test_head_can_align_while_obstacle_stream_is_unavailable(self):
        self.set_params(move_head=True, use_pot_fields=True)
        cls = type(self)
        cls.head_auto = True
        cls.head_pan = -1.5
        cls.publish_scan = False
        self.enable_pub.publish(Bool(data=True))
        self.spin_for(0.1)
        self.send_path(1.0, 0.0)
        self.wait_for(lambda: bool(self.head_goals))
        self.spin_for(0.7)
        self.assertGreater(self.head_pan, -1.0)
        self.assertEqual(self.command.linear.x, 0.0)
        self.assertEqual(self.command.angular.z, 0.0)
        cls.publish_scan = True
        self.wait_for(lambda: self.command.linear.x > 0.02, timeout=5)

    def test_pan_wrap_selects_nearby_mechanical_limit(self):
        self.set_params(move_head=True)
        type(self).head_pan = -3.0
        self.spin_for(0.1)
        self.send_path(-1.0, 0.0)
        self.wait_for(lambda: self.command.linear.x < -0.02)
        self.assertLess(abs(self.command.angular.z), 0.1)
        self.assertLess(self.head_goals[-1].data[0], -3.0)

    def test_inflation_escape_keeps_real_pose_and_low_speed(self):
        cls = type(self)
        cls.inflated_start = cls.integrate = True
        cls.scan_angle, cls.scan_range = math.pi/2, 0.7
        self.set_params(use_pot_fields=True)
        self.spin_for(0.1)
        self.send_goal(1.0, 0.0)
        until = time.monotonic()+12
        moving = []
        while time.monotonic() < until and not any(s.status == 3 for s in self.statuses):
            rclpy.spin_once(self.node, timeout_sec=0.01)
            if self.x < 0.15:
                moving.append(math.hypot(self.command.linear.x, self.command.linear.y))
            if self.x > 0.20:
                cls.scan_range = float('inf')
        self.assertTrue(any(s.status == 3 for s in self.statuses), [(s.status,s.text) for s in self.statuses])
        self.assertTrue(any(s.text == 'escaping_inflation' for s in self.results))
        self.assertLessEqual(max(moving), 0.081)
        self.assertAlmostEqual(self.plan_requests[0].start.pose.position.x, 0.0, places=3)

    def test_missing_obstacle_data_waits_without_replanning_loop(self):
        self.set_params(use_pot_fields=True)
        self.send_goal(1.0, 0.0)
        self.wait_for(lambda: self.command.linear.x > 0.02)
        type(self).publish_scan = False
        self.wait_for(lambda: any(r.status == 4 and r.text == 'obstacle_data_stale' for r in self.results), timeout=5)
        self.spin_for(0.2)
        count = self.plan_calls
        self.spin_for(0.8)
        self.assertEqual(self.plan_calls, count)
        self.assertEqual(self.command.linear.x, 0.0)
        type(self).publish_scan = True
        self.wait_for(lambda: self.plan_calls > count, timeout=5)
        self.assertTrue(any(s.text == 'obstacle_data_recovered' for s in self.statuses))

    def test_rotation_inside_planning_padding_keeps_position(self):
        cls = type(self)
        cls.inflated_start = cls.integrate = True
        self.spin_for(0.1)
        self.send_goal(0.0, 0.0, 0.8)
        self.wait_for(lambda: any(s.status == 3 for s in self.statuses), timeout=8)
        self.assertLess(math.hypot(self.x, self.y), 0.005)
        self.assertEqual(self.plan_calls, 0)

    def test_long_lookahead_does_not_cut_an_obstacle_corner(self):
        cls = type(self)
        cls.corner_obstacle = cls.integrate = cls.head_auto = True
        self.set_params(move_head=True)
        self.spin_for(0.1)
        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = self.node.get_clock().now().to_msg()
        points = [(0.02*i, 0.0) for i in range(21)]
        points += [(0.4+0.3*math.sin(math.pi*i/40), 0.3-0.3*math.cos(math.pi*i/40)) for i in range(1,21)]
        points += [(0.7, 0.3+0.03*i) for i in range(1,31)]
        for x, y in points:
            pose = PoseStamped()
            pose.pose.position.x, pose.pose.position.y = x, y
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        path.poses[-1].pose.orientation.z = path.poses[-1].pose.orientation.w = math.sqrt(0.5)
        self.path_pub.publish(path)
        until = time.monotonic()+20
        while time.monotonic() < until and not any(s.status in (3,4) for s in self.results):
            rclpy.spin_once(self.node, timeout_sec=0.01)
        self.assertTrue(any(s.status == 3 for s in self.results), [(s.status,s.text) for s in self.results])
        self.assertFalse(any(s.status == 4 for s in self.results))

    def test_omni_translation_stays_straight_while_base_aligns_near_wall(self):
        cls = type(self)
        cls.rotation_wall = cls.integrate = True
        cls.yaw = math.pi/2
        self.spin_for(0.1)
        self.send_path(2.0, 0.0)
        until = time.monotonic()+18
        early = []
        while time.monotonic() < until and not any(s.status in (3,4) for s in self.results):
            rclpy.spin_once(self.node, timeout_sec=0.01)
            if self.x > 0.1 and abs(self.yaw) > 0.5:
                early.append(math.hypot(self.command.linear.x, self.command.linear.y))
            self.assertLess(abs(self.y), 0.03)
        self.assertTrue(any(s.status == 3 for s in self.results), [(s.status,s.text) for s in self.results])
        self.assertTrue(early)
        self.assertGreater(max(early), 0.20)

    def test_head_limit_projection_does_not_fight_initial_yaw_profile(self):
        cls = type(self)
        cls.integrate = cls.head_auto = True
        cls.yaw = 1.635
        self.set_params(move_head=True)
        self.spin_for(0.1)
        self.send_path(0.0, -2.0, -math.pi/2)
        until = time.monotonic()+18
        stops, was_stopped = 0, False
        while time.monotonic() < until and not any(s.status in (3,4) for s in self.results):
            rclpy.spin_once(self.node, timeout_sec=0.01)
            if -1.7 < self.y < -0.1:
                stopped = math.hypot(self.command.linear.x, self.command.linear.y) < 0.01
                stops += stopped and not was_stopped
                was_stopped = stopped
        self.assertTrue(any(s.status == 3 for s in self.results), [(s.status,s.text) for s in self.results])
        self.assertFalse(any(s.status == 4 for s in self.results))
        self.assertLessEqual(stops, 1)


if __name__ == '__main__':
    unittest.main(verbosity=2)
