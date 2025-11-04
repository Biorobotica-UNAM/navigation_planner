# SPDX-License-Identifier: MIT
# ROS 2 Launch file converted from XML to Python
# Goal: avoid hardcoded values. Every tunable value is exposed as a launch argument
# with a sensible default. Nodes consume LaunchConfiguration variables only.
#
# Variable groupings (for readability):
#  - Core/time & lifecycle
#  - Map files & RViz config
#  - Topics (I/O interfaces)
#  - Sensor usage toggles
#  - Downsampling & thresholds
#  - Spatial ROI limits (laser & cloud)
#  - Frames & TF behavior
#  - AMCL parameters
#  - Potential fields parameters
#  - Planner/control parameters
#  - Misc per-node settings

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    LaunchConfiguration,
)
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, TextSubstitution
#from launch.substitutions import ConcatSubstitution
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    # =============================
    # Core/time & lifecycle
    # =============================
    use_sim_time = LaunchConfiguration('use_sim_time')
    lifecycle_nodes = LaunchConfiguration('lifecycle_nodes')  # YAML list as string
    namespace = LaunchConfiguration('namespace')  # Optional ROS namespace for all nodes

    # =============================
    # Map files & RViz config paths (defaults resolved via FindPackageShare)
    # =============================
    nav_pkg = LaunchConfiguration('nav_pkg')
    static_map_file = LaunchConfiguration('static_map_file')
    prohibition_map_file = LaunchConfiguration('prohibition_map_file')
    rviz_config = LaunchConfiguration('rviz_config')
    rviz_log_level = LaunchConfiguration('rviz_log_level')
    rviz_logger = LaunchConfiguration('rviz_logger')

    # =============================
    # Topics (I/O interfaces)
    # =============================
    odom_topic = LaunchConfiguration('odom_topic')
    pose_topic = LaunchConfiguration('pose_topic')
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')
    laser_scan_topic = LaunchConfiguration('laser_scan_topic')
    point_cloud_topic = LaunchConfiguration('point_cloud_topic')
    bumper_f_topic = LaunchConfiguration('bumper_f_topic')

    # =============================
    # Sensor usage toggles
    # =============================
    use_lidar = LaunchConfiguration('use_lidar')
    use_sonars = LaunchConfiguration('use_sonars')
    use_point_cloud = LaunchConfiguration('use_point_cloud')
    use_bumper = LaunchConfiguration('use_bumper')
    use_pot_fields = LaunchConfiguration('use_pot_fields')

    # =============================
    # Downsampling & thresholds
    # =============================
    cloud_downsampling = LaunchConfiguration('cloud_downsampling')
    lidar_downsampling = LaunchConfiguration('lidar_downsampling')
    cloud_points_threshold = LaunchConfiguration('cloud_points_threshold')
    lidar_points_threshold = LaunchConfiguration('lidar_points_threshold')

    # =============================
    # Spatial ROI limits (laser & cloud)
    # =============================
    laser_min_x = LaunchConfiguration('laser_min_x')
    laser_max_x = LaunchConfiguration('laser_max_x')
    laser_min_y = LaunchConfiguration('laser_min_y')
    laser_max_y = LaunchConfiguration('laser_max_y')
    laser_min_z = LaunchConfiguration('laser_min_z')
    laser_max_z = LaunchConfiguration('laser_max_z')

    cloud_min_x = LaunchConfiguration('cloud_min_x')
    cloud_max_x = LaunchConfiguration('cloud_max_x')
    cloud_min_y = LaunchConfiguration('cloud_min_y')
    cloud_max_y = LaunchConfiguration('cloud_max_y')
    cloud_min_z = LaunchConfiguration('cloud_min_z')
    cloud_max_z = LaunchConfiguration('cloud_max_z')

    # =============================
    # Frames & TF behavior
    # =============================
    global_frame_id = LaunchConfiguration('global_frame_id')
    frame_prefix = LaunchConfiguration('frame_prefix')  # optional prefix for frames (e.g., 'hardware/robotino/')
    odom_frame_id = LaunchConfiguration('odom_frame_id')
    base_frame_id = LaunchConfiguration('base_frame_id')
    tf_broadcast = LaunchConfiguration('tf_broadcast')

    # =============================
    # AMCL parameters
    # =============================
    amcl_min_particles = LaunchConfiguration('amcl_min_particles')
    amcl_max_particles = LaunchConfiguration('amcl_max_particles')
    amcl_transform_tolerance = LaunchConfiguration('amcl_transform_tolerance')

    # =============================
    # Potential fields parameters
    # =============================
    laser_pot_fields_d0 = LaunchConfiguration('laser_pot_fields_d0')
    laser_pot_fields_k_rej = LaunchConfiguration('laser_pot_fields_k_rej')
    cloud_pot_fields_d0 = LaunchConfiguration('cloud_pot_fields_d0')
    cloud_pot_fields_k_rej = LaunchConfiguration('cloud_pot_fields_k_rej')

    # =============================
    # Planner/control parameters
    # =============================
    mvn_pln_patience = LaunchConfiguration('mvn_pln_patience')
    max_linear_speed = LaunchConfiguration('max_linear_speed')
    max_angular_speed = LaunchConfiguration('max_angular_speed')
    control_alpha = LaunchConfiguration('control_alpha')
    control_beta = LaunchConfiguration('control_beta')
    move_head = LaunchConfiguration('move_head')

    # =============================
    # Declare all arguments with defaults (no hardcoded values inside nodes)
    # =============================
    declared_args = [
        # Namespacing
        DeclareLaunchArgument('namespace', default_value='', description='ROS namespace to push all nodes into. Leave empty for global namespace.'),
        DeclareLaunchArgument('frame_prefix', default_value='', description="Optional prefix for TF frame_ids (e.g., 'hardware/robotino/'). Leave empty for plain frames."),
        # Core/time & lifecycle
        DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulation clock (Gazebo/RViz).'),
        DeclareLaunchArgument(
            'lifecycle_nodes',
            default_value=TextSubstitution(text="['static_map_server', 'prohibition_map_server', 'amcl']"),
            description='YAML list of nodes to manage in nav2_lifecycle_manager.'
        ),
        # Package/share & file paths
        DeclareLaunchArgument('nav_pkg', default_value='navigation_start', description='Package that provides maps and RViz config.'),
        DeclareLaunchArgument(
            'static_map_file',
            default_value=PathJoinSubstitution([FindPackageShare(nav_pkg), 'maps', 'maps', 'wrs2020', 'map.yaml']),
            description='YAML map for static map_server.'
        ),
        DeclareLaunchArgument(
            'prohibition_map_file',
            default_value=PathJoinSubstitution([FindPackageShare(nav_pkg), 'maps', 'prohibition_maps', 'wrs2020', 'map.yaml']),
            description='YAML map for prohibition map_server.'
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=PathJoinSubstitution([FindPackageShare(nav_pkg), 'rviz', 'config.rviz']),
            description='RViz2 configuration file.'
        ),
        DeclareLaunchArgument('rviz_log_level', default_value='warn', description='RViz2 logger level (debug|info|warn|error|fatal).'),
        DeclareLaunchArgument('rviz_logger', default_value='rviz2', description='RViz logger name to set level for.'),
        # Topics
        DeclareLaunchArgument('odom_topic', default_value='odom', description='Odometry topic (relative; namespace applied upstream).'),
        DeclareLaunchArgument('pose_topic', default_value='pose', description='AMCL pose output topic (relative by default).'),
        DeclareLaunchArgument('cmd_vel_topic', default_value='cmd_vel', description='Velocity command topic (relative; namespace applied upstream).'),
        DeclareLaunchArgument('laser_scan_topic', default_value='scan', description='Laser scan input topic (relative; namespace applied upstream).'),
        DeclareLaunchArgument('point_cloud_topic', default_value='camera/depth/points', description='Point cloud input topic (relative; change as needed).'),
        DeclareLaunchArgument('bumper_f_topic', default_value='/base_f_bumper_sensor', description='Front bumper topic.'),
        # Sensor toggles
        DeclareLaunchArgument('use_lidar', default_value='true', description='Enable LiDAR data for processing.'),
        DeclareLaunchArgument('use_sonars', default_value='false', description='Enable sonars.'),
        DeclareLaunchArgument('use_point_cloud', default_value='false', description='Enable point cloud for processing.'),
        DeclareLaunchArgument('use_bumper', default_value='false', description='Enable bumper sensor.'),
        DeclareLaunchArgument('use_pot_fields', default_value='true', description='Enable potential fields in controller.'),
        # Downsampling & thresholds
        DeclareLaunchArgument('cloud_downsampling', default_value='5', description='Voxel/skip factor for cloud.'),
        DeclareLaunchArgument('lidar_downsampling', default_value='1', description='Skip factor for LiDAR.'),
        DeclareLaunchArgument('cloud_points_threshold', default_value='100', description='Minimal cloud points to trust.'),
        DeclareLaunchArgument('lidar_points_threshold', default_value='20', description='Minimal LiDAR points to trust.'),
        # Spatial ROI limits
        DeclareLaunchArgument('laser_min_x', default_value='0.17'),
        DeclareLaunchArgument('laser_max_x', default_value='0.50'),
        DeclareLaunchArgument('laser_min_y', default_value='-0.25'),
        DeclareLaunchArgument('laser_max_y', default_value='0.25'),
        DeclareLaunchArgument('laser_min_z', default_value='0.025'),
        DeclareLaunchArgument('laser_max_z', default_value='1.00'),
        DeclareLaunchArgument('cloud_min_x', default_value='0.30'),
        DeclareLaunchArgument('cloud_max_x', default_value='0.80'),
        DeclareLaunchArgument('cloud_min_y', default_value='-0.30'),
        DeclareLaunchArgument('cloud_max_y', default_value='0.30'),
        DeclareLaunchArgument('cloud_min_z', default_value='0.01'),
        DeclareLaunchArgument('cloud_max_z', default_value='1.20'),
        # Frames & TF
        DeclareLaunchArgument('global_frame_id', default_value='map'),
        DeclareLaunchArgument('odom_frame_id', default_value=ConcatSubstitution([frame_prefix, TextSubstitution(text='odom')])),
        DeclareLaunchArgument('base_frame_id', default_value=ConcatSubstitution([frame_prefix, TextSubstitution(text='base_link')])),
        DeclareLaunchArgument('tf_broadcast', default_value='true'),
        # AMCL params
        DeclareLaunchArgument('amcl_min_particles', default_value='500'),
        DeclareLaunchArgument('amcl_max_particles', default_value='3000'),
        DeclareLaunchArgument('amcl_transform_tolerance', default_value='0.2'),
        # Potential fields params
        DeclareLaunchArgument('laser_pot_fields_d0', default_value='0.50'),
        DeclareLaunchArgument('laser_pot_fields_k_rej', default_value='0.40'),
        DeclareLaunchArgument('cloud_pot_fields_d0', default_value='0.80'),
        DeclareLaunchArgument('cloud_pot_fields_k_rej', default_value='0.20'),
        # Planner/control params
        DeclareLaunchArgument('mvn_pln_patience', default_value='true'),
        DeclareLaunchArgument('max_linear_speed', default_value='0.8'),
        DeclareLaunchArgument('max_angular_speed', default_value='1.25'),
        DeclareLaunchArgument('control_alpha', default_value='0.2'),
        DeclareLaunchArgument('control_beta', default_value='0.8'),
        DeclareLaunchArgument('move_head', default_value='true'),
    ]

    # =============================
    # Nodes
    # =============================
    static_map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='static_map_server',
        output='screen',
        respawn=True,
        parameters=[
            {
                'yaml_filename': static_map_file,
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
            }
        ],
    )

    prohibition_map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='prohibition_map_server',
        output='screen',
        respawn=True,
        parameters=[
            {
                'yaml_filename': prohibition_map_file,
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
            }
        ],
        remappings=[('/map', '/fixed_prohibition_layer_map')],
    )

    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        respawn=True,
        parameters=[
            {
                # particle/timing
                'min_particles': amcl_min_particles,
                'max_particles': amcl_max_particles,
                'transform_tolerance': amcl_transform_tolerance,
                # frames
                'global_frame_id': global_frame_id,
                'odom_frame_id': odom_frame_id,
                'base_frame_id': base_frame_id,
                'tf_broadcast': ParameterValue(tf_broadcast, value_type=bool),
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
            }
        ],
        remappings=[
            ('scan', laser_scan_topic),
            ('odom', odom_topic),
            ('amcl_pose', pose_topic),
            ('particle_cloud', 'particle_cloud'),
        ],
    )

    lifecycle_manager_map = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        respawn=True,
        parameters=[
            {
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
                'autostart': True,
                'node_names': lifecycle_nodes,
            }
        ],
    )

    map_enhancer = Node(
        package='augment_gridmap_online',
        executable='augment_gridmap_online_node',
        name='map_enhancer',
        output='screen',
        respawn=True,
        parameters=[
            {
                'obstacle_radius': 0.05,  # kept as constant; expose if you plan to tune it
                'debug': True,
                'input_map': '/fixed_prohibition_layer_map',
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
            }
        ],
        remappings=[
            ('point_obstacle', '/clicked_point'),
            ('/grid_map/get_augmented_map', '/prohibition_map'),
            ('/grid_map/augmented_map', '/prohibition_layer_map'),
        ],
    )

    map_augmenter = Node(
        package='map_augmenter',
        executable='map_augmenter_node',
        name='map_augmenter',
        output='screen',
        respawn=True,
        parameters=[
            {
                'laser_scan_topic': laser_scan_topic,
                'point_cloud_topic': point_cloud_topic,
                'static_map_server': '/static_map_server/map',
                'prohibition_map_server': '/prohibition_map',
                'base_link_name': base_frame_id,
                'use_lidar': ParameterValue(use_lidar, value_type=bool),
                'use_sonars': ParameterValue(use_sonars, value_type=bool),
                'use_point_cloud': ParameterValue(use_point_cloud, value_type=bool),
                'cloud_downsampling': cloud_downsampling,
                'decay_factor': 20,
                'inflation_radius': 0.22,
                'cost_radius': 0.25,
                'use_online': False,
                # Laser ROI
                'laser_max_x': laser_max_x,
                'laser_min_x': laser_min_x,
                'laser_max_y': laser_max_y,
                'laser_min_y': laser_min_y,
                'laser_max_z': laser_max_z,
                'laser_min_z': laser_min_z,
                # Cloud ROI
                'cloud_max_x': cloud_max_x,
                'cloud_min_x': cloud_min_x,
                'cloud_max_y': cloud_max_y,
                'cloud_min_y': cloud_min_y,
                'cloud_max_z': cloud_max_z,
                'cloud_min_z': cloud_min_z,
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
            }
        ],
        remappings=[('point_obstacle', '/clicked_point')],
    )

    path_planner = Node(
        package='path_planner',
        executable='path_planner_node',
        name='path_planner',
        output='screen',
        respawn=True,
        parameters=[
            {
                'odom_frame_id': odom_frame_id,
                'base_frame_id': base_frame_id,
                'tf_broadcast': ParameterValue(tf_broadcast, value_type=bool),
                'diagonal_paths': True,
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
            }
        ],
    )

    simple_move = Node(
        package='simple_move',
        executable='simple_move_node',
        name='simple_move',
        output='screen',
        respawn=True,
        parameters=[
            {
                'max_linear_speed': max_linear_speed,
                'max_angular_speed': max_angular_speed,
                'control_alpha': control_alpha,
                'control_beta': control_beta,
                'move_head': ParameterValue(move_head, value_type=bool),
                'use_pot_fields': ParameterValue(use_pot_fields, value_type=bool),
                'base_link_name': base_frame_id,
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
            }
        ],
        remappings=[('cmd_vel', cmd_vel_topic)],
    )

    potential_fields = Node(
        package='potential_fields',
        executable='potential_fields_node',
        name='potential_fields',
        output='screen',
        respawn=True,
        parameters=[
            {
                'debug': False,
                'show_image': True,
                'use_pot_fields': ParameterValue(use_pot_fields, value_type=bool),
                'use_lidar': ParameterValue(use_lidar, value_type=bool),
                'use_point_cloud': ParameterValue(use_point_cloud, value_type=bool),
                # Laser ROI
                'laser_max_x': laser_max_x,
                'laser_min_x': laser_min_x,
                'laser_max_y': laser_max_y,
                'laser_min_y': laser_min_y,
                'laser_max_z': laser_max_z,
                'laser_min_z': laser_min_z,
                # Cloud ROI
                'cloud_max_x': cloud_max_x,
                'cloud_min_x': cloud_min_x,
                'cloud_max_y': cloud_max_y,
                'cloud_min_y': cloud_min_y,
                'cloud_max_z': cloud_max_z,
                'cloud_min_z': cloud_min_z,
                # PF params
                'laser_pot_fields_d0': laser_pot_fields_d0,
                'laser_pot_fields_k_rej': laser_pot_fields_k_rej,
                'cloud_pot_fields_d0': cloud_pot_fields_d0,
                'cloud_pot_fields_k_rej': cloud_pot_fields_k_rej,
                # Sampling/thresholds
                'cloud_downsampling': cloud_downsampling,
                'cloud_points_threshold': cloud_points_threshold,
                'lidar_points_threshold': lidar_points_threshold,
                # Topics
                'point_cloud_topic': point_cloud_topic,
                'laser_scan_topic': laser_scan_topic,
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
            }
        ],
        remappings=[('cmd_vel', cmd_vel_topic)],
    )

    publish_enable = Node(
        package='potential_fields',
        executable='publish_enable.py',
        name='publish_enable',
        output='screen',
        respawn=True,
        parameters=[{'use_sim_time': ParameterValue(use_sim_time, value_type=bool)}],
    )

    mvn_pln = Node(
        package='mvn_pln',
        executable='mvn_pln_node',
        name='mvn_pln',
        output='screen',
        respawn=True,
        parameters=[
            {
                'patience': ParameterValue(mvn_pln_patience, value_type=bool),
                'base_link_name': base_frame_id,
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
            }
        ],
        remappings=[('nav_control/goal', 'move_base_simple/goal')],
    )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=[
            '-d', rviz_config,
            '--ros-args',
            '--log-level',
            ConcatSubstitution([rviz_logger, TextSubstitution(text=':='), rviz_log_level]),
        ],
    )

    # Push namespace (if non-empty) so nodes/topics become relative to it. Absolute topics (starting with '/') are NOT affected.
    return LaunchDescription(declared_args + [
        PushRosNamespace(namespace),
        static_map_server,
        prohibition_map_server,
        amcl,
        lifecycle_manager_map,
        map_enhancer,
        map_augmenter,
        path_planner,
        simple_move,
        potential_fields,
        publish_enable,
        mvn_pln,
        rviz2,
    ])
