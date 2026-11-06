#!/usr/bin/env python3
"""Real navigation/head nodes, camera-frame depth and a simulated base; no hardware."""
import array
import math
import os
from pathlib import Path as FilePath
import struct
import subprocess
import time
import unittest

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.srv import SetParametersAtomically
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from ament_index_python.packages import get_package_prefix
from actionlib_msgs.msg import GoalStatus
from control_msgs.msg import JointTrajectoryControllerState
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Bool, Empty, Float64MultiArray, Float32
from trajectory_msgs.msg import JointTrajectory
from tf2_ros import TransformBroadcaster

os.environ['CYCLONEDDS_URI'] = (
    '<CycloneDDS><Domain><General><AllowMulticast>false</AllowMulticast></General>'
    '<Discovery><ParticipantIndex>auto</ParticipantIndex><Peers><Peer Address="127.0.0.1"/>'
    '</Peers></Discovery></Domain></CycloneDDS>')


class DepthNavigation(unittest.TestCase):
    serial = 0

    def setUp(self):
        if os.environ.get('ROS_LOCALHOST_ONLY') != '1' or not os.environ.get('ROS_DOMAIN_ID'):
            raise RuntimeError('Use an isolated ROS_DOMAIN_ID and ROS_LOCALHOST_ONLY=1')
        type(self).serial += 1
        self.ns = f'/depth_{self.serial}'
        rclpy.init()
        self.node = Node(f'depth_fixture_{self.serial}')
        self.processes, self.logs = [], []
        self.addCleanup(self.cleanup)
        self.x = self.y = self.yaw = 0.0
        self.pan, self.tilt = 0.0, -0.6
        self.pitch, self.camera_height = None, 0.8
        self.cloud_in_base = False
        self.head_position = np.array([self.pan, self.tilt])
        self.head_from = self.head_position.copy()
        self.head_to = self.head_position.copy()
        self.head_started, self.head_duration = time.monotonic(), 0.1
        self.command, self.risk, self.limit = Twist(), False, float('inf')
        self.integrate = False
        self.raw_table = False
        self.world_points, self.sensor_points = [], None
        self.statuses, self.trace, self.trajectories = [], [], []
        self.obstacles = self.augmented = self.clearance = None
        self.last_depth = self.last_map = 0.0
        self.tf = TransformBroadcaster(self.node)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.raw = self.node.create_publisher(OccupancyGrid, self.ns+'/raw', qos)
        self.cloud = self.node.create_publisher(PointCloud2, self.ns+'/points', qos_profile_sensor_data)
        self.head_state = self.node.create_publisher(JointTrajectoryControllerState, self.ns+'/head_state', 10)
        self.goal = self.node.create_publisher(PoseStamped, self.ns+'/goal', 10)
        self.stop = self.node.create_publisher(Empty, self.ns+'/stop', 10)
        self.enable = self.node.create_publisher(Bool, self.ns+'/potential_fields/enable', 10)
        self.head_goal = self.node.create_publisher(Float64MultiArray, self.ns+'/head_goal', 10)
        self.subs = [
            self.node.create_subscription(Twist, self.ns+'/cmd', lambda m: setattr(self, 'command', m), 10),
            self.node.create_subscription(GoalStatus, self.ns+'/status', lambda m: self.statuses.append(m), 10),
            self.node.create_subscription(OccupancyGrid, self.ns+'/obstacles_map', lambda m: setattr(self, 'obstacles', m), qos),
            self.node.create_subscription(OccupancyGrid, self.ns+'/augmented_map', lambda m: setattr(self, 'augmented', m), qos),
            self.node.create_subscription(OccupancyGrid, self.ns+'/clearance_map', lambda m: setattr(self, 'clearance', m), qos),
            self.node.create_subscription(Bool, self.ns+'/potential_fields/collision_risk', lambda m: setattr(self, 'risk', m.data), 10),
            self.node.create_subscription(Float32, self.ns+'/potential_fields/speed_limit', lambda m: setattr(self, 'limit', m.data), 10),
            self.node.create_subscription(JointTrajectory, self.ns+'/head_trajectory', self.trajectory, 10),
        ]
        configurations = [
            ('map_augmenter', 'map_augmenter_node', ['-p', 'base_map_topic:=raw', '-p', 'map_frame:=world',
                '-p', 'use_prohibition_map:=false', '-p', 'circular_inflation:=true',
                '-p', 'base_link_name:=robot_base', '-p', 'cost_radius:=0.6', '-p', 'cost_peak:=15']),
            ('map_augmenter', 'obstacle_detector_node', ['-p', 'use_point_cloud:=true', '-p', 'map_frame:=world',
                '-p', 'odom_frame:=nav_odom', '-p', 'base_link_name:=robot_base', '-p', 'point_cloud_topic:='+self.ns+'/points',
                '-p', 'cloud_min_range:=0.6', '-p', 'cloud_max_range:=5.0',
                '-p', 'cloud_sensor_frame:=depth_camera',
                '-p', 'cloud_min_x:=0.0', '-p', 'cloud_max_x:=2.5',
                '-p', 'cloud_min_y:=-1.0', '-p', 'cloud_max_y:=1.0',
                '-p', 'cloud_min_z:=0.05', '-p', 'cloud_max_z:=1.5',
                '-p', 'cloud_min_neighbor_cells:=2', '-p', 'obstacle_timeout_sec:=4.0',
                '-p', 'debug_detection_period_sec:=0.25',
                '-p', 'unobserved_obstacle_hold_sec:=1.0']),
            ('potential_fields', 'potential_fields_node', ['-p', 'use_lidar:=false', '-p', 'base_link_name:=robot_base',
                '-p', 'odom_frame:=nav_odom', '-p', 'use_point_cloud:=true', '-p', 'point_cloud_topic:='+self.ns+'/points',
                '-p', 'cloud_min_range:=0.6', '-p', 'cloud_max_range:=5.0',
                '-p', 'cloud_sensor_frame:=depth_camera',
                '-p', 'cloud_max_x:=2.5', '-p', 'cloud_points_threshold:=3',
                '-r', '/cmd_vel:='+self.ns+'/cmd']),
            ('path_planner', 'path_planner_node', ['-p', 'diagonal_paths:=true']),
            ('simple_move', 'simple_move_node', ['-p', 'max_linear_speed:=0.5', '-p', 'base_link_name:=robot_base',
                '-p', 'map_frame:=world', '-p', 'odom_name:=nav_odom',
                '-p', 'use_pot_fields:=true', '-p', 'move_head:=false',
                '-p', 'head_goal_topic:='+self.ns+'/head_goal',
                '-p', 'head_feedback_topic:='+self.ns+'/head_pose',
                '-r', '/cmd_vel:='+self.ns+'/cmd']),
            ('mvn_pln', 'mvn_pln_node', ['-p', 'base_link_name:=robot_base',
                '-p', 'map_frame:=world', '-r', '/nav_control/goal:='+self.ns+'/goal']),
            ('hsr_hardware', 'hsr_head_node', ['-p', 'head_goal_topic:='+self.ns+'/head_goal',
                '-p', 'head_current_topic:='+self.ns+'/head_pose',
                '-p', 'head_state_topic:='+self.ns+'/head_state',
                '-p', 'head_cmd_topic_:='+self.ns+'/head_trajectory']),
        ]
        for package, executable, extra in configurations:
            log = open(f'/tmp/depth_{self.serial}_{executable}.log', 'w')
            self.logs.append(log)
            binary = FilePath(get_package_prefix(package))/'lib'/package/executable
            self.processes.append(subprocess.Popen([str(binary), '--ros-args', '-r', '__ns:='+self.ns,
                '-p', 'use_namespace:=true']+extra, stdout=log, stderr=subprocess.STDOUT))
        self.params = self.node.create_client(SetParametersAtomically,
            self.ns+'/simple_move_node/set_parameters_atomically')
        self.timer = self.node.create_timer(0.02, self.tick)
        self.spin_for(1.8)
        self.assertTrue(all(p.poll() is None for p in self.processes), 'See /tmp/depth_* logs')
        self.enable.publish(Bool(data=True))
        self.spin_for(0.3)

    def cleanup(self):
        for p in self.processes:
            p.terminate()
        for p in self.processes:
            try:
                p.wait(timeout=3)
            except subprocess.TimeoutExpired:
                p.kill()
                p.wait(timeout=3)
        for log in self.logs:
            log.close()
        self.node.destroy_node()
        rclpy.shutdown()

    def trajectory(self, message):
        self.trajectories.append(message)
        self.head_from = self.head_position.copy()
        self.head_to = np.array(message.points[0].positions)
        self.head_started = time.monotonic()
        duration = message.points[0].time_from_start
        self.head_duration = duration.sec+duration.nanosec*1e-9

    def tick(self):
        now = time.monotonic()
        if self.integrate:
            self.x += (math.cos(self.yaw)*self.command.linear.x-math.sin(self.yaw)*self.command.linear.y)*0.02
            self.y += (math.sin(self.yaw)*self.command.linear.x+math.cos(self.yaw)*self.command.linear.y)*0.02
            self.yaw += self.command.angular.z*0.02
            self.trace.append((self.x, self.y, self.yaw, math.hypot(self.command.linear.x, self.command.linear.y)))
        phase = min(1.0, (now-self.head_started)/self.head_duration)
        self.head_position = self.head_from+(self.head_to-self.head_from)*phase
        self.pan, self.tilt = self.head_position
        state = JointTrajectoryControllerState()
        # Use pan first: the former driver incorrectly assumed tilt first.
        state.joint_names = ['head_pan_joint', 'head_tilt_joint']
        state.actual.positions = [float(self.pan), float(self.tilt)]
        self.head_state.publish(state)
        stamp = self.node.get_clock().now().to_msg()
        transforms = []
        for parent, child in [('world', 'nav_odom'), ('nav_odom', 'robot_base')]:
            tf = TransformStamped()
            tf.header.stamp, tf.header.frame_id, tf.child_frame_id = stamp, parent, child
            tf.transform.rotation.w = 1.0
            if child == 'robot_base':
                tf.transform.translation.x, tf.transform.translation.y = self.x, self.y
                tf.transform.rotation.z, tf.transform.rotation.w = math.sin(self.yaw/2), math.cos(self.yaw/2)
            transforms.append(tf)
        pitch = -self.tilt if self.pitch is None else self.pitch
        c, s, cp, sp = math.cos(self.pan), math.sin(self.pan), math.cos(pitch), math.sin(pitch)
        rotation = np.array([[s, -c*sp, c*cp], [-c, -s*sp, s*cp], [0, -cp, -sp]])
        tf = TransformStamped()
        tf.header.stamp, tf.header.frame_id, tf.child_frame_id = stamp, 'robot_base', 'depth_camera'
        tf.transform.translation.x, tf.transform.translation.z = 0.1, self.camera_height
        qw = math.sqrt(max(1e-9, 1+np.trace(rotation)))/2
        tf.transform.rotation.w = qw
        tf.transform.rotation.x = (rotation[2, 1]-rotation[1, 2])/(4*qw)
        tf.transform.rotation.y = (rotation[0, 2]-rotation[2, 0])/(4*qw)
        tf.transform.rotation.z = (rotation[1, 0]-rotation[0, 1])/(4*qw)
        transforms.append(tf)
        self.tf.sendTransform(transforms)
        if now-self.last_map > 0.2:
            self.last_map = now
            grid = OccupancyGrid()
            grid.header.stamp, grid.header.frame_id = stamp, 'world'
            grid.info.width = grid.info.height = 160
            grid.info.resolution = 0.05
            grid.info.origin.position.x = grid.info.origin.position.y = -4.0
            grid.info.origin.orientation.w = 1.0
            data = [0]*25600
            if self.raw_table:
                for iy in range(78, 83):
                    for ix in range(92, 96):
                        data[iy*160+ix] = 100
            grid.data = data
            self.raw.publish(grid)
        if now-self.last_depth > 0.05:
            self.last_depth = now
            if self.sensor_points is not None:
                points = self.sensor_points
            else:
                points = []
                # Finite floor returns keep an empty scene observable. An all-NaN
                # frame represents unknown depth and must never release motion.
                floor = [(self.x+d, self.y+y, 0.0)
                    for d in (0.8, 1.2, 1.8, 2.5) for y in (-0.3, 0.0, 0.3)]
                for x, y, z in self.world_points+floor:
                    dx, dy = x-self.x, y-self.y
                    base = np.array([math.cos(self.yaw)*dx+math.sin(self.yaw)*dy-0.1,
                        -math.sin(self.yaw)*dx+math.cos(self.yaw)*dy, z-self.camera_height])
                    point = rotation.T@base
                    if point[2] > 0 and 0.6 <= np.linalg.norm(point) <= 5.0:
                        points.extend([point]*4)
            if len(points) == 0:
                points = [(float('nan'),)*3]
            cloud = PointCloud2()
            cloud.header.stamp, cloud.header.frame_id = stamp, 'depth_camera'
            if self.cloud_in_base:
                points = [rotation@np.array(p)+np.array([0.1, 0, self.camera_height]) for p in points]
                cloud.header.frame_id = 'robot_base'
            cloud.height, cloud.width, cloud.point_step = 1, len(points), 12
            cloud.row_step = cloud.width*12
            cloud.fields = [PointField(name=name, offset=4*i, datatype=PointField.FLOAT32, count=1)
                for i, name in enumerate(('x', 'y', 'z'))]
            cloud.data = array.array('B', b''.join(struct.pack('<fff', *p) for p in points))
            self.cloud.publish(cloud)

    def spin_for(self, seconds):
        until = time.monotonic()+seconds
        while time.monotonic() < until:
            rclpy.spin_once(self.node, timeout_sec=0.01)

    def occupied(self, x, y):
        if self.obstacles is None:
            return False
        grid = self.obstacles
        ix = int((x-grid.info.origin.position.x)/grid.info.resolution)
        iy = int((y-grid.info.origin.position.y)/grid.info.resolution)
        return grid.data[iy*grid.info.width+ix] > 0

    def test_camera_range_precedes_base_crop_for_front_and_downward_views(self):
        for pitch, in_base in ((0.0, False), (1.1, True)):
            self.pitch = pitch
            self.cloud_in_base = in_base
            self.sensor_points = [(0.0, 0.0, 0.55)]*20
            self.spin_for(0.4)
            x, z = 0.1+0.55*math.cos(pitch), 0.8-0.55*math.sin(pitch)
            self.assertFalse(self.occupied(x, 0), (pitch, x, z))
            self.sensor_points = [(0.0, 0.0, 0.65)]*20
            self.spin_for(0.4)
            self.assertTrue(self.occupied(0.1+0.65*math.cos(pitch), 0))

    def test_thin_supported_cloud_obstacle_reaches_map_and_prevents_approach(self):
        self.world_points = [(0.245, 0.0, 0.10)]*5
        self.spin_for(0.8)
        self.assertTrue(self.risk)
        self.assertTrue(self.occupied(0.245, 0.0))
        goal = PoseStamped()
        goal.header.frame_id = 'world'
        goal.pose.position.x, goal.pose.orientation.w = 1.5, 1.0
        self.goal.publish(goal)
        self.integrate = True
        self.spin_for(2)
        self.assertTrue(self.trace)
        # Starting in planning padding may permit a safe retreat. It must
        # never permit approaching the critical obstacle during that recovery.
        self.assertGreaterEqual(min(math.hypot(t[0]-0.245, t[1]) for t in self.trace), 0.243)
        self.assertLess(max(t[0] for t in self.trace), 0.005)

    def test_blind_near_obstacle_survives_timeout_until_observed_free(self):
        client = self.node.create_client(SetParametersAtomically,
            self.ns+'/obstacle_detector_node/set_parameters_atomically')
        self.assertTrue(client.wait_for_service(timeout_sec=2))
        request = SetParametersAtomically.Request()
        request.parameters = [Parameter('debug_detection', value=True).to_parameter_msg()]
        future = client.call_async(request)
        until = time.monotonic()+2
        while not future.done() and time.monotonic() < until:
            rclpy.spin_once(self.node,timeout_sec=0.01)
        self.assertTrue(future.done() and future.result().result.successful)
        self.world_points = [(0.75, 0.0, 0.4)]*5
        self.spin_for(0.6)
        self.assertTrue(self.occupied(0.75, 0))
        self.x = 0.25
        self.spin_for(4.5)
        self.assertTrue(self.occupied(0.75, 0), 'Near-range dropout must not erase the obstacle')
        log = FilePath(f'/tmp/depth_{self.serial}_obstacle_detector_node.log').read_text()
        self.assertRegex(log, r'retained_blind=[1-9]')
        self.assertIn('camera_range_m=', log)
        self.assertIn('confirmed_refreshes=', log)
        self.camera_height = 1.1
        self.world_points = [(1.15, 0.0, -0.3)]*5
        self.spin_for(0.8)
        self.assertFalse(self.occupied(0.75, 0), 'Leaving the blind range should allow expired evidence to clear')
        log = FilePath(f'/tmp/depth_{self.serial}_obstacle_detector_node.log').read_text()
        self.assertRegex(log, r'expired_(before|after)=[1-9]')
        # Check ray clearing independently, while the new evidence is still young.
        self.world_points = [(0.75,0.0,0.4)]*5
        self.spin_for(0.6)
        self.assertTrue(self.occupied(0.75,0))
        self.world_points = [(1.15,0.0,-0.3)]*5
        self.spin_for(0.8)
        self.assertFalse(self.occupied(0.75,0), 'Three valid rays should clear evidence before its timeout')
        log = FilePath(f'/tmp/depth_{self.serial}_obstacle_detector_node.log').read_text()
        self.assertRegex(log, r'ray_cleared=[1-9]')
        self.node.destroy_client(client)

    def test_observable_ghost_expires_during_camera_loss_without_releasing_base(self):
        self.world_points = [(0.75, 0.0, 0.4)]*5
        self.spin_for(0.6)
        self.assertTrue(self.occupied(0.75, 0))
        self.sensor_points = [(float('nan'),)*3]*5
        self.spin_for(0.7)
        goal = PoseStamped()
        goal.header.frame_id = 'world'
        goal.pose.position.x, goal.pose.orientation.w = 1.5, 1.0
        self.goal.publish(goal)
        self.integrate = True
        self.spin_for(4.5)
        self.assertFalse(self.occupied(0.75, 0), 'Observable memory must not be retained forever')
        self.assertLess(math.hypot(self.x, self.y), 0.005)
        self.assertEqual(math.hypot(self.command.linear.x, self.command.linear.y), 0.0)

    def test_floor_is_rejected_in_raw_and_confirmed_clouds_at_both_tilts(self):
        evidence = self.node.create_publisher(PointCloud2,
            self.ns+'/potential_fields/confirmed_obstacles', 1)
        for pitch in (0.0, 1.1):
            self.pitch = pitch
            message = PointCloud2()
            message.header.frame_id = 'robot_base'
            message.width, message.height = 2, 1
            message.point_step, message.row_step = 12, 24
            message.fields = [PointField(name=n, offset=i*4, datatype=PointField.FLOAT32, count=1)
                              for i,n in enumerate(('x','y','z'))]
            message.data = struct.pack('<ffffff', 0.9, 0.0, 0.0, 1.2, 0.0, 0.1)
            for _ in range(8):
                message.header.stamp = self.node.get_clock().now().to_msg()
                evidence.publish(message)
                self.spin_for(0.1)
            self.assertFalse(self.occupied(0.9, 0), pitch)
            self.assertTrue(self.occupied(1.2, 0), pitch)
        self.node.destroy_publisher(evidence)

    def test_non_aligned_motion_is_continuous_with_actual_head_node(self):
        request = SetParametersAtomically.Request()
        request.parameters = [Parameter('move_head', value=True).to_parameter_msg()]
        future = self.params.call_async(request)
        self.spin_for(0.3)
        self.assertTrue(future.done() and future.result().result.successful)
        goal = PoseStamped()
        goal.header.frame_id = 'world'
        goal.pose.position.y = 2.0
        goal.pose.orientation.z = goal.pose.orientation.w = math.sqrt(0.5)
        self.integrate = True
        self.goal.publish(goal)
        until = time.monotonic()+25
        while time.monotonic() < until and not any(s.status in (3, 4) for s in self.statuses):
            rclpy.spin_once(self.node, timeout_sec=0.01)
        self.assertTrue(any(s.status == 3 for s in self.statuses), [(s.status, s.text) for s in self.statuses])
        moving = [t for t in self.trace if 0.10 < t[1] < 1.80]
        self.assertTrue(moving)
        self.assertLess(sum(t[3] < 0.01 for t in moving), 3)
        velocities = [np.array(t.points[0].velocities) for t in self.trajectories]
        self.assertTrue(velocities)
        self.assertLessEqual(max(np.max(np.abs(v)) for v in velocities), 1.5001)

    def test_person_seen_only_by_depth_is_avoided(self):
        goal = PoseStamped()
        goal.header.frame_id = 'world'
        goal.pose.position.x, goal.pose.orientation.w = 2.2, 1.0
        self.integrate = True
        self.goal.publish(goal)
        until = time.monotonic()+5
        while self.x < 0.15 and time.monotonic() < until:
            rclpy.spin_once(self.node, timeout_sec=0.01)
        self.assertGreaterEqual(self.x, 0.15)
        self.world_points = [(1.1+0.15*math.cos(a), 0.15*math.sin(a), z)
            for a in np.linspace(0, 2*math.pi, 36) for z in np.linspace(0.2, 1.3, 12)]
        until = time.monotonic()+30
        while time.monotonic() < until and not any(s.status in (3, 4) for s in self.statuses):
            rclpy.spin_once(self.node, timeout_sec=0.01)
        self.assertTrue(any(s.status == 3 for s in self.statuses), [(s.status, s.text) for s in self.statuses])
        self.assertGreater(min(math.hypot(t[0]-1.1, t[1])-0.15 for t in self.trace), 0.23)

    def test_all_nan_depth_stops_instead_of_declaring_free_space(self):
        goal = PoseStamped()
        goal.header.frame_id = 'world'
        goal.pose.position.x, goal.pose.orientation.w = 2.0, 1.0
        self.goal.publish(goal)
        until = time.monotonic()+5
        while self.command.linear.x <= 0.03 and time.monotonic() < until:
            rclpy.spin_once(self.node, timeout_sec=0.01)
        self.assertGreater(self.command.linear.x, 0.03)
        self.sensor_points = [(float('nan'),)*3]
        self.spin_for(0.8)
        self.assertEqual(self.command.linear.x, 0.0)
        self.spin_for(0.8)
        self.assertEqual(self.command.linear.x, 0.0)

    def test_actual_planner_and_controller_escape_table_padding(self):
        self.raw_table = True
        self.x = 0.365
        # Use a cell center: a boundary return can round into the adjacent cell
        # and put this deliberately close start inside physical clearance.
        self.world_points = [(0.625, 0.025, 0.1)]*5
        self.spin_for(0.7)
        for grid, blocked in ((self.augmented, True), (self.clearance, False)):
            self.assertIsNotNone(grid)
            ix = int((self.x-grid.info.origin.position.x)/grid.info.resolution)
            iy = int((self.y-grid.info.origin.position.y)/grid.info.resolution)
            self.assertEqual(grid.data[iy*grid.info.width+ix] > 0, blocked)
        goal = PoseStamped()
        goal.header.frame_id = 'world'
        goal.pose.position.x, goal.pose.orientation.w = -0.5, 1.0
        self.integrate = True
        self.goal.publish(goal)
        until = time.monotonic()+15
        while time.monotonic() < until and not any(s.status in (3, 4) for s in self.statuses):
            rclpy.spin_once(self.node, timeout_sec=0.01)
        self.assertTrue(any(s.status == 3 for s in self.statuses),
            [(s.status, s.text) for s in self.statuses]+[('pose', (self.x,self.y,self.yaw))])
        self.assertTrue(any(s.text == 'escaping_inflation' for s in self.statuses))
        self.assertTrue(all(0.6-t[0] >= 0.23 for t in self.trace))


if __name__ == '__main__':
    unittest.main(verbosity=2)
