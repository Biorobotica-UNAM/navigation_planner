#!/usr/bin/env python3
"""Actual planner/detector against controlled maps and sensors, without hardware."""
import math
import copy
import os
from pathlib import Path as FilePath
import subprocess
import time
import unittest

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, DurabilityPolicy
from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import PointStamped, PoseStamped, TransformStamped
from nav_msgs.msg import OccupancyGrid, Path
from nav_msgs.srv import GetMap
from path_planner.srv import PlanPath
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformBroadcaster

os.environ['CYCLONEDDS_URI'] = (
    '<CycloneDDS><Domain><General>'
    '<AllowMulticast>false</AllowMulticast></General><Discovery><ParticipantIndex>auto</ParticipantIndex><Peers>'
    '<Peer Address="127.0.0.1"/></Peers></Discovery></Domain></CycloneDDS>')


class PlanningMaps(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if os.environ.get('ROS_LOCALHOST_ONLY') != '1' or not os.environ.get('ROS_DOMAIN_ID'):
            raise RuntimeError('Use an isolated ROS_DOMAIN_ID with ROS_LOCALHOST_ONLY=1')
        rclpy.init()
        cls.node = Node('planning_map_fixture')
        cls.logs, cls.processes = [], []
        cls.addClassCleanup(cls.cleanup)
        for package, executable, extra in [
            ('path_planner', 'path_planner_node', ['-p', 'diagonal_paths:=true']),
            ('map_augmenter', 'obstacle_detector_node', [
                '-p', 'use_lidar:=true', '-p', 'base_link_name:=base_link',
                '-p', 'laser_scan_topic:=/test_memory_scan', '-p', 'ignore_static_obstacles:=false',
                '-p', 'laser_min_x:=0.1', '-p', 'laser_max_x:=2.0',
                '-p', 'laser_min_y:=-2.0', '-p', 'laser_max_y:=2.0',
                '-p', 'laser_min_z:=-0.1', '-p', 'laser_max_z:=0.5']),
        ]:
            log = open(f'/tmp/navigation_test_{executable}.log', 'w')
            cls.logs.append(log)
            binary = FilePath(get_package_prefix(package))/'lib'/package/executable
            cls.processes.append(subprocess.Popen(
                [str(binary), '--ros-args', '-r', '__ns:=/map_test',
                 '-p', 'use_namespace:=true'] + extra, stdout=log, stderr=subprocess.STDOUT))
        cls.grid = OccupancyGrid()
        cls.grid.header.frame_id = 'map'
        cls.grid.info.width = cls.grid.info.height = 100
        cls.grid.info.resolution = 0.05
        cls.grid.info.origin.position.x = cls.grid.info.origin.position.y = -2.0
        cls.grid.info.origin.orientation.w = 1.0
        cls.grid.data = [0]*10000
        cls.physical_grid = None
        cls.mismatch, cls.stale = False, False
        cls.scan_range = float('inf')
        cls.obstacles = None
        cls.last_map_update = 0.0
        cls.services = []
        for name in ('static', 'augmented'):
            cls.services.append(cls.node.create_service(
                GetMap, f'/map_test/map_augmenter/get_{name}_map', cls.map_response))
            cls.services.append(cls.node.create_service(
                GetMap, f'/map_test/map_augmenter/get_{name}_cost_map', cls.cost_response))
        cls.base_pub = cls.node.create_publisher(
            OccupancyGrid, '/map_test/map_augmenter/base_map',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        cls.clearance_pub = cls.node.create_publisher(OccupancyGrid, '/map_test/clearance_map',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        cls.scan_pub = cls.node.create_publisher(LaserScan, '/test_memory_scan', 10)
        cls.tf = TransformBroadcaster(cls.node)
        cls.obstacles_sub = cls.node.create_subscription(
            OccupancyGrid, '/map_test/obstacles_map', lambda m: setattr(cls, 'obstacles', m),
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        cls.client = cls.node.create_client(PlanPath, '/map_test/path_planner/plan_navigation')
        cls.timer = cls.node.create_timer(0.02, cls.tick)
        cls.spin_for(1.0)
        if not cls.client.service_is_ready():
            raise RuntimeError('Planner did not start; see /tmp/navigation_test_path_planner_node.log')

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
    def map_response(cls, request, response):
        response.map = cls.grid
        return response

    @classmethod
    def cost_response(cls, request, response):
        response.map = copy.deepcopy(cls.grid)
        if cls.mismatch:
            response.map.header.stamp = (cls.node.get_clock().now()+Duration(seconds=1)).to_msg()
        return response

    @classmethod
    def tick(cls):
        stamp = cls.node.get_clock().now()
        if time.monotonic()-cls.last_map_update >= 0.2:
            cls.last_map_update = time.monotonic()
            cls.grid.header.stamp = (stamp-Duration(seconds=2 if cls.stale else 0)).to_msg()
            cls.base_pub.publish(cls.grid)
            physical = copy.deepcopy(cls.physical_grid if cls.physical_grid is not None else cls.grid)
            physical.header.stamp = cls.grid.header.stamp
            cls.clearance_pub.publish(physical)
        transforms = []
        for parent, child in [('map', 'odom'), ('odom', 'base_link')]:
            tf = TransformStamped()
            tf.header.stamp, tf.header.frame_id, tf.child_frame_id = stamp.to_msg(), parent, child
            tf.transform.rotation.w = 1.0
            transforms.append(tf)
        cls.tf.sendTransform(transforms)
        scan = LaserScan()
        scan.header.stamp, scan.header.frame_id = stamp.to_msg(), 'base_link'
        scan.angle_min, scan.angle_max, scan.angle_increment = -math.pi, math.pi, math.pi/180
        scan.range_min, scan.range_max = 0.1, 10.0
        scan.ranges = [float('inf')]*361
        scan.ranges[180] = cls.scan_range
        cls.scan_pub.publish(scan)

    @classmethod
    def spin_for(cls, seconds):
        deadline = time.monotonic()+seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(cls.node, timeout_sec=0.01)

    def setUp(self):
        type(self).stale = type(self).mismatch = False
        type(self).scan_range = float('inf')
        self.grid.data = [0]*10000
        type(self).physical_grid = None
        self.spin_for(0.6)

    def plan(self, minimum=None, avoided=None, required=None, goal_radius=0.0, start=(0.0,0.0), goal=(2.0,0.0)):
        request = PlanPath.Request()
        request.start.header.frame_id = request.goal.header.frame_id = 'map'
        request.start.pose.orientation.w = request.goal.pose.orientation.w = 1.0
        request.start.pose.position.x, request.start.pose.position.y = start
        request.goal.pose.position.x, request.goal.pose.position.y = goal
        request.goal_recovery_radius = goal_radius
        if minimum is not None:
            request.minimum_map_stamp = minimum
        if avoided is not None:
            request.avoided_paths = avoided
        if required is not None:
            request.required_obstacles = required
        future = self.client.call_async(request)
        deadline = time.monotonic()+4
        while not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.01)
        self.assertTrue(future.done())
        return future.result()

    def test_start_recovery_is_local_and_independent_of_blocked_global_goal(self):
        physical = copy.deepcopy(self.grid)
        for y in range(38,42):
            physical.data[y*100+41] = 100
        type(self).physical_grid = physical
        self.grid.data = [100]*10000
        data = list(self.grid.data)
        for x in range(44,70): data[40*100+x] = 0
        self.grid.data = data
        self.spin_for(0.6)
        result = self.plan(start=(0.025,0.025), goal=(1.5,1.5), goal_radius=0.33)
        self.assertEqual(result.reason, '')
        self.assertTrue(result.start_recovery)
        self.assertFalse(result.goal_relocated)
        self.assertGreater(len(result.path.poses), 2)
        self.assertAlmostEqual(result.path.poses[0].pose.position.x, 0.025)
        self.assertAlmostEqual(result.path.poses[-1].pose.position.x, 0.225, places=5)

    def test_goal_alternative_is_opt_in_bounded_and_outside_obstacles(self):
        data = list(self.grid.data)
        for y in range(45,56):
            for x in range(55,66): data[y*100+x] = 100
        self.grid.data = data
        self.spin_for(0.6)
        exact = self.plan(goal=(1.025,0.525))
        self.assertEqual(exact.reason, 'goal_blocked')
        result = self.plan(goal=(1.025,0.525), goal_radius=0.33)
        self.assertEqual(result.reason, '')
        self.assertTrue(result.goal_relocated)
        p = result.path.poses[-1].pose.position
        self.assertLess(math.hypot(p.x-1.025,p.y-0.525), 0.33)
        x,y = int((p.x+2)/self.grid.info.resolution),int((p.y+2)/self.grid.info.resolution)
        self.assertEqual(self.grid.data[y*100+x], 0)
        boundary = self.plan(goal=(1.025,0.525), goal_radius=math.hypot(p.x-1.025,p.y-0.525))
        self.assertEqual(boundary.reason, 'goal_blocked')
        impossible = self.plan(goal=(1.025,0.525), goal_radius=0.05)
        self.assertEqual(impossible.reason, 'goal_blocked')

    def test_requires_map_version_then_accepts_same_fixed_requirement(self):
        required = (self.node.get_clock().now()+Duration(seconds=0.5)).to_msg()
        result = self.plan(required)
        self.assertEqual(result.reason, 'waiting_for_updated_map')
        self.assertFalse(result.path.poses)
        self.spin_for(0.9)
        result = self.plan(required)
        self.assertEqual(result.reason, '')
        self.assertTrue(result.path.poses)
        self.assertGreaterEqual(rclpy.time.Time.from_msg(result.map_stamp),
                                rclpy.time.Time.from_msg(required))

    def test_mismatched_pair_cannot_reuse_older_map_for_replanning(self):
        type(self).mismatch = True
        required = self.node.get_clock().now().to_msg()
        self.spin_for(0.5)
        result = self.plan(required)
        self.assertEqual(result.reason, 'waiting_for_updated_map')
        self.assertFalse(result.path.poses)
        type(self).mismatch = False
        self.spin_for(0.5)
        self.assertTrue(self.plan(required).path.poses)

    def test_recent_receipt_does_not_make_old_map_fresh(self):
        type(self).stale = True
        self.spin_for(1.3)
        result = self.plan()
        self.assertEqual(result.reason, 'waiting_for_updated_map')
        self.assertFalse(result.path.poses)

    def test_recovery_costs_produce_an_alternative_without_persistent_map_changes(self):
        normal = self.plan()
        self.assertTrue(normal.path.poses)
        avoided = Path()
        avoided.header.frame_id = 'map'
        for i in range(21):
            pose = PoseStamped()
            pose.pose.position.x = 0.04*i
            pose.pose.orientation.w = 1.0
            avoided.poses.append(pose)
        result = self.plan(avoided=[avoided])
        self.assertEqual(result.reason, '')
        self.assertTrue(result.path.poses)
        self.assertGreater(max(abs(p.pose.position.y) for p in result.path.poses), 0.15)
        ordinary = self.plan()
        self.assertTrue(ordinary.path.poses)
        self.assertLess(max(abs(p.pose.position.y) for p in ordinary.path.poses), 0.08)

    def test_new_obstacle_is_included_in_replanned_route(self):
        ordinary = self.plan()
        self.assertTrue(ordinary.path.poses)
        self.assertLess(max(abs(p.pose.position.y) for p in ordinary.path.poses), 0.08)
        data = [0]*10000
        for y in range(33, 48):
            for x in range(55, 66):
                data[y*100+x] = 100
        self.grid.data = data
        required = self.node.get_clock().now().to_msg()
        self.spin_for(0.5)
        result = self.plan(required)
        self.assertEqual(result.reason, '')
        self.assertTrue(result.path.poses)
        self.assertGreater(max(abs(p.pose.position.y) for p in result.path.poses), 0.35)
        self.assertGreaterEqual(rclpy.time.Time.from_msg(result.map_stamp),
                                rclpy.time.Time.from_msg(required))
        for a, b in zip(result.path.poses, result.path.poses[1:]):
            for sample in range(21):
                t = sample/20
                x = a.pose.position.x+t*(b.pose.position.x-a.pose.position.x)
                y = a.pose.position.y+t*(b.pose.position.y-a.pose.position.y)
                self.assertEqual(data[int((y+2)/0.05)*100+int((x+2)/0.05)], 0)

    def test_recent_map_must_contain_the_sensor_witness(self):
        witness = PointStamped()
        witness.header.frame_id = 'map'
        witness.point.x, witness.point.y = 1.025, 0.025
        result = self.plan(required=[witness])
        self.assertEqual(result.reason, 'waiting_for_obstacle_map')
        self.assertFalse(result.path.poses)
        data = list(self.grid.data)
        data[40*100+60] = 100
        self.grid.data = data
        self.spin_for(0.5)
        result = self.plan(required=[witness])
        self.assertEqual(result.reason, '')
        self.assertTrue(result.path.poses)

    def test_obstacle_survives_temporary_loss_then_expires(self):
        type(self).scan_range = 0.8
        deadline = time.monotonic()+3
        while (self.obstacles is None or 100 not in self.obstacles.data) and time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.01)
        self.assertIsNotNone(self.obstacles)
        self.assertIn(100, self.obstacles.data)
        type(self).scan_range = float('inf')
        self.spin_for(1.2)
        self.assertIn(100, self.obstacles.data)
        self.spin_for(1.6)
        self.assertNotIn(100, self.obstacles.data)


if __name__ == '__main__':
    unittest.main(verbosity=2)
