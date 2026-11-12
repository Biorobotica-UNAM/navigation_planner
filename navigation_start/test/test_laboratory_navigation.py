#!/usr/bin/env python3
"""Full navigation stack on the actual laboratory/prohibition maps; no hardware."""
import array
import math
import os
from pathlib import Path as FilePath
import subprocess
import time
import unittest

import numpy as np
from PIL import Image
import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from ament_index_python.packages import get_package_prefix
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from geometry_msgs.msg import PoseArray, Pose
from nav_msgs.msg import OccupancyGrid, Path
from path_planner.srv import PlanPath
from rcl_interfaces.srv import SetParametersAtomically
from rclpy.parameter import Parameter
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Empty, Float64MultiArray
from tf2_ros import TransformBroadcaster

os.environ['CYCLONEDDS_URI'] = (
    '<CycloneDDS><Domain><General><AllowMulticast>false</AllowMulticast></General>'
    '<Discovery><ParticipantIndex>auto</ParticipantIndex><Peers><Peer Address="127.0.0.1"/>'
    '</Peers></Discovery></Domain></CycloneDDS>')


class LaboratoryNavigation(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if os.environ.get('ROS_LOCALHOST_ONLY') != '1' or not os.environ.get('ROS_DOMAIN_ID'):
            raise RuntimeError('Use an isolated ROS_DOMAIN_ID and ROS_LOCALHOST_ONLY=1')
        rclpy.init()
        cls.node = Node('laboratory_navigation_fixture')
        cls.logs, cls.processes = [], []
        cls.addClassCleanup(cls.cleanup)
        maps = FilePath(__file__).resolve().parents[1]/'maps'
        cls.grids = []
        for relative in ('maps/lab_2026/map.yaml', 'prohibition_maps/lab_2026/map.yaml'):
            metadata = maps/relative
            config = yaml.safe_load(metadata.read_text())
            pixels = np.flipud(np.asarray(Image.open(metadata.parent/config['image'])))
            probability = 1-pixels.astype(float)/255
            values = np.full(pixels.shape, -1, dtype=np.int8)
            values[probability < config['free_thresh']] = 0
            values[probability > config['occupied_thresh']] = 100
            grid = OccupancyGrid()
            grid.header.frame_id = 'map'
            grid.info.resolution = config['resolution']
            grid.info.height, grid.info.width = values.shape
            grid.info.origin.position.x, grid.info.origin.position.y, yaw = config['origin']
            grid.info.origin.orientation.z, grid.info.origin.orientation.w = math.sin(yaw/2), math.cos(yaw/2)
            grid.data = array.array('b', values.ravel().tobytes())
            cls.grids.append(grid)
            if len(cls.grids) == 1:
                cls.physical_map = values
        cls.x, cls.y, cls.yaw = 0.041, 0.940, 1.635
        cls.head_pan = cls.head_target = 0.0
        cls.command = Twist()
        cls.statuses, cls.trace = [], []
        cls.minimum_clearance = float('inf')
        cls.dynamic_obstacle = None
        cls.uncertain_points = []
        cls.head_auto = True
        cls.results = []
        cls.integrate = False
        cls.last_maps = cls.last_scan = 0.0
        cls.tf = TransformBroadcaster(cls.node)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        cls.raw_pub = cls.node.create_publisher(OccupancyGrid, '/lab_test/raw_map', qos)
        cls.prohibition_pub = cls.node.create_publisher(OccupancyGrid, '/lab_test/prohibition', qos)
        cls.goal_pub = cls.node.create_publisher(PoseStamped, '/lab_test/goal', 10)
        cls.path_pub = cls.node.create_publisher(Path, '/lab_test/simple_move/goal_path', 10)
        cls.uncertainty_pub = cls.node.create_publisher(PoseArray, '/lab_test/uncertain_obstacles', qos)
        cls.stop_pub = cls.node.create_publisher(Empty, '/lab_test/stop', 10)
        cls.head_pub = cls.node.create_publisher(Float64MultiArray, '/hardware/head/current_pose', 10)
        cls.scan_pub = cls.node.create_publisher(LaserScan, '/lab_scan', 10)
        cls.subs = [
            cls.node.create_subscription(Twist, '/lab_cmd_vel', lambda m: setattr(cls, 'command', m), 10),
            cls.node.create_subscription(GoalStatus, '/lab_test/status', lambda m: cls.statuses.append(m), 10),
            cls.node.create_subscription(GoalStatus, '/lab_test/simple_move/goal_reached', lambda m: cls.results.append(m), 10),
            cls.node.create_subscription(Float64MultiArray, '/hardware/head/goal_pose',
                                         lambda m: setattr(cls, 'head_target', m.data[0]), 10),
        ]
        configurations = [
            ('map_augmenter', 'map_augmenter_node', ['-p', 'base_map_topic:=raw_map',
                '-p', 'prohibition_map_topic:=prohibition', '-p', 'use_prohibition_map:=true',
                '-p', 'static_obstacle_inflation_radius:=0.25', '-p', 'dynamic_obstacle_inflation_radius:=0.25',
                '-p', 'circular_inflation:=true', '-p', 'cost_radius:=0.9', '-p', 'cost_peak:=15']),
            ('map_augmenter', 'obstacle_detector_node', ['-p', 'use_lidar:=true',
                '-p', 'base_link_name:=base_link', '-p', 'laser_scan_topic:=/lab_scan',
                '-p', 'laser_min_x:=0.1', '-p', 'laser_max_x:=3.0', '-p', 'laser_min_y:=-3.0',
                '-p', 'laser_max_y:=3.0', '-p', 'laser_min_z:=-0.1', '-p', 'laser_max_z:=0.5',
                '-p', 'static_obstacle_ignore_radius:=0.08']),
            ('path_planner', 'path_planner_node', ['-p', 'diagonal_paths:=true']),
            ('simple_move', 'simple_move_node', ['-p', 'max_linear_speed:=0.9',
                '-p', 'control_alpha:=0.2', '-p', 'control_beta:=0.6', '-p', 'move_head:=true',
                '-p', 'fine_dist_tolerance:=0.08', '-p', 'angle_tolerance:=0.08',
                '-p', 'head_alignment_tolerance:=0.20', '-p', 'angular_damping:=0.50',
                '-p', 'use_pot_fields:=true', '-r', '/cmd_vel:=/lab_cmd_vel']),
            ('potential_fields', 'potential_fields_node', ['-p', 'use_lidar:=true',
                '-p', 'use_point_cloud:=false', '-p', 'laser_scan_topic:=/lab_scan',
                '-p', 'laser_min_z:=-0.1', '-p', 'laser_pot_fields_d0:=0.75',
                '-p', 'laser_pot_fields_k_rej:=0.4', '-p', 'lidar_points_threshold:=2',
                '-r', '/cmd_vel:=/lab_cmd_vel']),
            ('mvn_pln', 'mvn_pln_node', ['-p', 'goal_position_tolerance_m:=0.08',
                '-p', 'goal_angle_tolerance_rad:=0.08', '-r', '/nav_control/goal:=/lab_test/goal']),
        ]
        for package, executable, extra in configurations:
            if os.environ.get('NAVIGATION_TEST_DEBUG') == '1':
                extra += ['-p', 'debug_detection:=true']
            if executable == 'obstacle_detector_node':
                # Inject historical cloud evidence independently of the live lidar.
                extra += ['-p', 'uncertain_obstacles_topic:=/lab_test/detector_uncertainty']
            log = open(f"{os.environ.get('NAVIGATION_LAB_LOG_PREFIX', '/tmp/laboratory_')}{executable}.log", 'w')
            cls.logs.append(log)
            binary = FilePath(get_package_prefix(package))/'lib'/package/executable
            cls.processes.append(subprocess.Popen([str(binary), '--ros-args', '-r', '__ns:=/lab_test',
                '-p', 'use_namespace:=true']+extra, stdout=log, stderr=subprocess.STDOUT))
        cls.client = cls.node.create_client(PlanPath, '/lab_test/path_planner/plan_navigation')
        cls.params = cls.node.create_client(SetParametersAtomically, '/lab_test/simple_move_node/set_parameters_atomically')
        cls.timer = cls.node.create_timer(0.02, cls.tick)
        cls.spin_for(2)
        if any(p.poll() is not None for p in cls.processes):
            raise RuntimeError('A node exited; see /tmp/laboratory_*.log')

    @classmethod
    def cleanup(cls):
        for p in cls.processes:
            p.terminate()
        for p in cls.processes:
            try:
                p.wait(timeout=5)
            except subprocess.TimeoutExpired:
                p.kill()
                p.wait(timeout=5)
        for log in cls.logs:
            log.close()
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def tick(cls):
        if cls.integrate:
            cls.x += (math.cos(cls.yaw)*cls.command.linear.x-math.sin(cls.yaw)*cls.command.linear.y)*0.02
            cls.y += (math.sin(cls.yaw)*cls.command.linear.x+math.cos(cls.yaw)*cls.command.linear.y)*0.02
            cls.yaw += cls.command.angular.z*0.02
            cls.trace.append((cls.x, cls.y, cls.yaw, math.hypot(cls.command.linear.x, cls.command.linear.y), cls.command.angular.z))
            if cls.dynamic_obstacle is not None:
                cls.minimum_clearance = min(cls.minimum_clearance,
                    math.hypot(cls.x-cls.dynamic_obstacle[0], cls.y-cls.dynamic_obstacle[1])-0.08)
            for ox,oy in cls.uncertain_points:
                cls.minimum_clearance = min(cls.minimum_clearance, math.hypot(cls.x-ox, cls.y-oy))
        if cls.head_auto:
            cls.head_pan += max(-0.03, min(0.03, cls.head_target-cls.head_pan))
        cls.head_pub.publish(Float64MultiArray(data=[cls.head_pan, -0.5]))
        stamp = cls.node.get_clock().now().to_msg()
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
        uncertainty = PoseArray()
        uncertainty.header.frame_id, uncertainty.header.stamp = 'map', stamp
        for x, y in cls.uncertain_points:
            point = Pose()
            point.position.x, point.position.y, point.orientation.w = x, y, 1.0
            uncertainty.poses.append(point)
        cls.uncertainty_pub.publish(uncertainty)
        if time.monotonic()-cls.last_maps >= 0.5:
            cls.last_maps = time.monotonic()
            for publisher, grid in zip((cls.raw_pub, cls.prohibition_pub), cls.grids):
                grid.header.stamp = stamp
                publisher.publish(grid)
        info = cls.grids[0].info
        gx, gy = (cls.x-info.origin.position.x)/info.resolution, (cls.y-info.origin.position.y)/info.resolution
        # Exact point-to-occupied-cell-square distance around the physical base.
        ix, iy = int(gx), int(gy)
        for cy, cx in np.argwhere(cls.physical_map[max(0, iy-7):iy+8, max(0, ix-7):ix+8] > 0):
            cx += max(0, ix-7); cy += max(0, iy-7)
            distance = math.hypot(max(cx-gx, 0, gx-cx-1), max(cy-gy, 0, gy-cy-1))*info.resolution
            cls.minimum_clearance = min(cls.minimum_clearance, distance)
        if time.monotonic()-cls.last_scan >= 0.05:
            cls.last_scan = time.monotonic()
            angles = np.linspace(-math.pi, math.pi, 361)+cls.yaw
            distances = np.arange(0.10, 3.01, 0.025)
            wx = cls.x+np.cos(angles)[:, None]*distances
            wy = cls.y+np.sin(angles)[:, None]*distances
            mx = ((wx-info.origin.position.x)/info.resolution).astype(int)
            my = ((wy-info.origin.position.y)/info.resolution).astype(int)
            valid = (mx >= 0)&(mx < info.width)&(my >= 0)&(my < info.height)
            hits = valid & (cls.physical_map[np.clip(my, 0, info.height-1), np.clip(mx, 0, info.width-1)] > 0)
            if cls.dynamic_obstacle is not None:
                ox, oy = cls.dynamic_obstacle
                hits |= (wx-ox)**2+(wy-oy)**2 < 0.08**2
            ranges = np.where(hits.any(axis=1), distances[hits.argmax(axis=1)], float('inf'))
            scan = LaserScan()
            scan.header.stamp, scan.header.frame_id = stamp, 'base_link'
            scan.angle_min, scan.angle_max, scan.angle_increment = -math.pi, math.pi, math.pi/180
            scan.range_min, scan.range_max, scan.ranges = 0.1, 3.0, ranges.astype(float).tolist()
            cls.scan_pub.publish(scan)

    @classmethod
    def spin_for(cls, seconds):
        until = time.monotonic()+seconds
        while time.monotonic() < until:
            rclpy.spin_once(cls.node, timeout_sec=0.01)

    def setUp(self):
        self.stop_pub.publish(Empty())
        type(self).integrate = False
        self.spin_for(0.2)
        type(self).dynamic_obstacle = None
        type(self).uncertain_points = []
        type(self).head_auto = True
        self.set_params(use_pot_fields=True, local_avoidance_timeout_sec=1.5)
        self.results.clear()
        self.statuses.clear()
        self.trace.clear()
        type(self).minimum_clearance = float('inf')

    def set_params(self, **values):
        request = SetParametersAtomically.Request()
        request.parameters = [Parameter(name, value=value).to_parameter_msg() for name, value in values.items()]
        future = self.params.call_async(request)
        until = time.monotonic()+4
        while not future.done() and time.monotonic() < until:
            rclpy.spin_once(self.node, timeout_sec=0.01)
        self.assertTrue(future.done())
        self.assertTrue(future.result().result.successful, future.result().result.reason)

    def plan(self, start, goal, recovery_radius=0.33):
        request = PlanPath.Request()
        request.start.header.frame_id = request.goal.header.frame_id = 'map'
        request.start.pose.position.x, request.start.pose.position.y = start
        request.goal.pose.position.x, request.goal.pose.position.y = goal
        request.start.pose.orientation.w = request.goal.pose.orientation.w = 1.0
        request.goal_recovery_radius = recovery_radius
        future = self.client.call_async(request)
        until = time.monotonic()+6
        while not future.done() and time.monotonic() < until:
            rclpy.spin_once(self.node, timeout_sec=0.01)
        self.assertTrue(future.done())
        self.assertEqual(future.result().reason, '')
        self.assertTrue(future.result().path.poses)
        return future.result()

    def navigate(self, start, goal, timeout=45, introduce_obstacle=False, approximate=False):
        cls = type(self)
        cls.x, cls.y, cls.yaw = start
        cls.command = Twist()
        cls.head_pan = cls.head_target = 0.0
        self.spin_for(0.6)
        message = PoseStamped()
        message.header.frame_id = 'map'
        message.pose.position.x, message.pose.position.y, angle = goal
        message.pose.orientation.z, message.pose.orientation.w = math.sin(angle/2), math.cos(angle/2)
        cls.integrate = True
        self.goal_pub.publish(message)
        until = time.monotonic()+timeout
        inserted = False
        while time.monotonic() < until:
            rclpy.spin_once(self.node, timeout_sec=0.01)
            if introduce_obstacle and not inserted and math.hypot(self.x-start[0], self.y-start[1]) > 0.3:
                cls.dynamic_obstacle = (cls.x+0.7*math.cos(cls.yaw), cls.y+0.7*math.sin(cls.yaw))
                inserted = True
            if any(s.status in (3, 4) for s in self.statuses):
                break
        cls.integrate = False
        self.assertTrue(any(s.status == 3 for s in self.statuses),
                        [(s.status, s.text) for s in self.statuses]+[('pose', (self.x,self.y,self.yaw))])
        self.assertLess(math.hypot(self.x-goal[0], self.y-goal[1]), 0.33 if approximate else 0.09)
        self.assertLess(abs(math.remainder(self.yaw-goal[2], 2*math.pi)), 0.09)
        self.assertGreaterEqual(self.minimum_clearance, 0.225)
        print(f' peak={max(t[3] for t in self.trace):.3f}m/s clearance={self.minimum_clearance:.3f}m', end='')

    def test_logged_corner_route_at_launch_speed(self):
        self.navigate((0.041, 0.940, 1.635), (-1.253, -3.962, 1.573))

    def test_logged_goal_9_with_large_final_rotation(self):
        self.navigate((-0.714, -3.998, -0.126), (-0.150795, -3.24419, 3.03394), timeout=25)

    def test_logged_goal_10_next_to_table(self):
        self.navigate((-0.108, -3.297, 3.030), (-0.992634, -3.68924, 1.56822), timeout=25)

    def test_logged_terminal_pose_next_to_wall(self):
        self.navigate((-1.253, -3.962, 1.573), (-0.279, -4.110, 1.503), timeout=25)
        changes = [a[4]*b[4] < -0.01 for a,b in zip(self.trace, self.trace[1:])]
        self.assertLess(sum(changes), 4)

    def test_straight_corridor_preserves_speed(self):
        self.navigate((-2.245, -1.008, -math.pi/2), (-2.009, -2.660, -math.pi/2), timeout=25)
        self.assertGreater(max(t[3] for t in self.trace), 0.45)

    def test_small_start_changes_do_not_require_nudging_robot(self):
        for dx, dy in ((0, 0), (-0.02, 0), (0.02, 0), (0, -0.02), (0, 0.02)):
            request = PlanPath.Request()
            request.start.header.frame_id = request.goal.header.frame_id = 'map'
            request.start.pose.position.x, request.start.pose.position.y = 0.041+dx, 0.940+dy
            request.start.pose.orientation.w = 1.0
            request.goal.pose.position.x, request.goal.pose.position.y = -0.279, -4.110
            request.goal.pose.orientation.w = 1.0
            # This goal is only 4 mm from an inflated cell edge. Use the launch's
            # position tolerance rather than requesting an exact guarded pose.
            request.goal_recovery_radius = 0.08
            future = self.client.call_async(request)
            until = time.monotonic()+4
            while not future.done() and time.monotonic() < until:
                rclpy.spin_once(self.node, timeout_sec=0.01)
            self.assertTrue(future.done())
            self.assertEqual(future.result().reason, '', (dx,dy,future.result().reason))
            self.assertTrue(future.result().path.poses)
            endpoint = future.result().path.poses[-1].pose.position
            self.assertLessEqual(math.hypot(endpoint.x+0.279, endpoint.y+4.110), 0.08)

    def test_obstacle_appearing_during_motion_is_avoided(self):
        self.navigate((-2.245, -1.008, -math.pi/2), (-2.009, -2.660, -math.pi/2),
                      timeout=35, introduce_obstacle=True)

    def test_uncertain_obstacle_is_used_by_planner_and_clears(self):
        cls = type(self)
        start, goal = (-2.245, -1.008), (-2.009, -2.660)
        baseline = self.plan(start, goal)
        middle = baseline.path.poses[len(baseline.path.poses)//2].pose.position
        cls.uncertain_points = [(middle.x, middle.y)]
        self.spin_for(0.7)
        started = time.monotonic()
        detour = self.plan(start, goal)
        self.assertFalse(detour.goal_relocated)
        self.assertLess(time.monotonic()-started, 2.0)
        # Check complete segments, including smoothed links between waypoints.
        for a, b in zip(detour.path.poses, detour.path.poses[1:]):
            ax, ay = a.pose.position.x-middle.x, a.pose.position.y-middle.y
            dx, dy = b.pose.position.x-a.pose.position.x, b.pose.position.y-a.pose.position.y
            t = max(0.0, min(1.0, -(ax*dx+ay*dy)/max(1e-12, dx*dx+dy*dy)))
            self.assertGreaterEqual(math.hypot(ax+t*dx, ay+t*dy), 0.28-1e-6)
        cls.uncertain_points = []
        self.spin_for(0.7)
        restored = self.plan(start, goal)
        self.assertFalse(restored.goal_relocated)
        self.assertLess(min(math.hypot(p.pose.position.x-middle.x, p.pose.position.y-middle.y)
                            for p in restored.path.poses), 0.10)

    def test_uncertain_start_recovers_without_moving_closer(self):
        cls = type(self)
        start, obstacle = (-2.100, -3.139), (-1.850, -3.139)
        cls.x, cls.y, cls.yaw = *start, -0.166
        cls.uncertain_points = [obstacle]
        self.spin_for(0.7)
        result = self.plan(start, (-2.245, -1.008))
        self.assertTrue(result.start_recovery)
        floor = 0.25
        for a, b in zip(result.path.poses, result.path.poses[1:]):
            ax, ay = a.pose.position.x-obstacle[0], a.pose.position.y-obstacle[1]
            dx, dy = b.pose.position.x-a.pose.position.x, b.pose.position.y-a.pose.position.y
            t = max(0.0, min(1.0, -(ax*dx+ay*dy)/max(1e-12, dx*dx+dy*dy)))
            self.assertGreaterEqual(math.hypot(ax+t*dx, ay+t*dy), floor-1e-6)
            floor = min(0.28, math.hypot(ax+dx, ay+dy))
        endpoint = result.path.poses[-1].pose.position
        self.assertGreaterEqual(math.hypot(endpoint.x-obstacle[0], endpoint.y-obstacle[1]), 0.28-1e-6)

    def test_logged_blocked_goal_completes_inside_safe_arrival_region(self):
        cls = type(self)
        goal = (-1.69942, -2.89339, -0.142956)
        cls.uncertain_points = [goal[:2]]
        self.navigate((-2.100, -3.139, -0.166), goal, timeout=20, approximate=True)
        terminal = [s for s in self.statuses if s.status in (3, 4)]
        self.assertEqual(len(terminal), 1)
        self.assertEqual(terminal[0].text, 'reached_near_goal: goal_blocked')
        self.assertGreaterEqual(math.hypot(self.x-goal[0], self.y-goal[1]), 0.28-1e-3)
        self.assertFalse(any('replanning:' in s.text and 'waiting_for_updated_map' not in s.text
                             for s in self.statuses))

    def test_head_wait_after_geometric_rejection_uses_head_deadline(self):
        cls = type(self)
        self.set_params(use_pot_fields=False, local_avoidance_timeout_sec=0.5)
        cls.x, cls.y, cls.yaw = -2.100, -3.139, 0.0
        cls.head_pan = cls.head_target = 0.0
        cls.head_auto = False
        cls.uncertain_points = [(cls.x+dx, cls.y+dy)
                                for dx,dy in ((0.28,0),(-0.28,0),(0,0.28),(0,-0.28))]
        self.spin_for(0.7)
        path = Path()
        path.header.frame_id, path.header.stamp = 'map', self.node.get_clock().now().to_msg()
        for x,y in ((cls.x,cls.y),(cls.x,cls.y+0.60)):
            pose = PoseStamped()
            pose.pose.position.x, pose.pose.position.y, pose.pose.orientation.w = x,y,1.0
            path.poses.append(pose)
        self.path_pub.publish(path)
        until = time.monotonic()+0.4
        while not any(r.text == 'waiting_for_local_clearance' for r in self.results) and time.monotonic() < until:
            rclpy.spin_once(self.node, timeout_sec=0.01)
        self.assertTrue(any(r.text == 'waiting_for_local_clearance' for r in self.results))
        cls.uncertain_points = []
        self.spin_for(0.9)
        self.assertTrue(any(r.text == 'aligning_head' for r in self.results))
        self.assertFalse(any(r.status == 4 for r in self.results))
        self.assertEqual(math.hypot(self.command.linear.x, self.command.linear.y), 0.0)
        cls.head_auto = cls.integrate = True
        until = time.monotonic()+12
        while not any(r.status in (3,4) for r in self.results) and time.monotonic() < until:
            rclpy.spin_once(self.node, timeout_sec=0.01)
        cls.integrate = False
        self.assertTrue(any(r.status == 3 for r in self.results), [(r.status,r.text) for r in self.results])


if __name__ == '__main__':
    unittest.main(verbosity=2)
