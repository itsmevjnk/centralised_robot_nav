from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():
    declared_arguments = []
    nodes = []

    declared_arguments.append(
        DeclareLaunchArgument(
            'name',
            description='The robot\'s name.'
        )
    )
    name = LaunchConfiguration('name')

    declared_arguments.append(
        DeclareLaunchArgument(
            'ns',
            default_value='/',
            description='The robot\'s namespace.'
        )
    )
    ns = LaunchConfiguration('ns')

    declared_arguments.append(
        DeclareLaunchArgument(
            'map_frame',
            default_value='map',
            description='The map TF frame used by the robot.'
        )
    )
    map_frame = LaunchConfiguration('map_frame')

    declared_arguments.append(
        DeclareLaunchArgument(
            'robot_frame',
            default_value='base_link',
            description='The robot\'s TF frame.'
        )
    )
    robot_frame = LaunchConfiguration('robot_frame')
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use Gazebo simulated clock instead of the actual clock.'
        )
    )
    use_sim_time = LaunchConfiguration('use_sim_time')

    declared_arguments.append(
        DeclareLaunchArgument(
            'publish_pose',
            default_value='true',
            description='Publish pose to /robot_poses.',
        )
    )
    publish_pose = LaunchConfiguration('publish_pose')

    nodes.append(
        Node(
            package='pose_publisher',
            executable='pub_node',
            namespace=ns,
            condition=IfCondition(publish_pose),
            parameters=[{
                'map_frame': map_frame,
                'robot_frame': robot_frame,
                'out_map_frame': map_frame,
                'out_robot_frame': name,
                'use_sim_time': use_sim_time
            }],
            remappings=[
                ('pose', '/robot_poses'),
                ('/tf', 'tf'),
                ('/tf_static', 'tf_static')
            ]
        )
    )

    nodes.append(
        Node(
            package='central_nav',
            executable='path_erase_node',
            namespace=ns,
            parameters=[{
                'use_sim_time': use_sim_time
            }],
            remappings=[
                ('path', 'plan')
            ]
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            'filter_path',
            default_value='true',
            description='Filter the navigation path to reduce segment count.'
        )
    )
    filter_path = LaunchConfiguration('filter_path')

    declared_arguments.append(
        DeclareLaunchArgument(
            'filter_min_length',
            default_value='0.05',
            description='The minimum segment length in the navigation path after filtering.'
        )
    )
    filter_min_length = LaunchConfiguration('filter_min_length')

    nodes.append(
        Node(
            package='central_nav',
            executable='path_pub_node',
            namespace=ns,
            parameters=[{
                'robot_name': name,
                'use_sim_time': use_sim_time
            }],
            remappings=[
                ('path_in', 'plan'),
                ('path_out', '/robot_paths')
            ],
            condition=UnlessCondition(filter_path)
        )
    )

    nodes.append(
        Node(
            package='central_nav',
            executable='path_filter_node',
            namespace=ns,
            parameters=[{
                'robot_name': name,
                'min_length': filter_min_length,
                'use_sim_time': use_sim_time
            }],
            remappings=[
                ('path_in', 'plan'),
                ('path_out', '/robot_paths')
            ],
            condition=IfCondition(filter_path)
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            'init_goal',
            default_value='false',
            description='Whether this robot\'s navigation has an initial goal.'
        )
    )
    init_goal = LaunchConfiguration('init_goal')

    declared_arguments.append(
        DeclareLaunchArgument(
            'init_goal_x',
            default_value='0.0',
            description='The robot\'s initial goal\'s X axis coordinate.'
        )
    )
    init_goal_x = LaunchConfiguration('init_goal_x')

    declared_arguments.append(
        DeclareLaunchArgument(
            'init_goal_y',
            default_value='0.0',
            description='The robot\'s initial goal\'s Y axis coordinate.'
        )
    )
    init_goal_y = LaunchConfiguration('init_goal_y')

    declared_arguments.append(
        DeclareLaunchArgument(
            'init_goal_yaw',
            default_value='0.0',
            description='The robot\'s initial goal\'s yaw orientation.'
        )
    )
    init_goal_yaw = LaunchConfiguration('init_goal_yaw')

    nodes.append(
        Node(
            package='nav2_goal_cancel',
            executable='string_node',
            namespace=ns,
            parameters=[{
                'robot_name': name,
                'init_goal': init_goal,
                'init_goal_x': init_goal_x,
                'init_goal_y': init_goal_y,
                'init_goal_yaw': init_goal_yaw,
                'use_sim_time': use_sim_time
            }],
            remappings=[
                ('pass', '/robot_pass'),
                ('stop', '/robot_stop'),
                ('/navigate_to_pose/_action/cancel_goal', 'navigate_to_pose/_action/cancel_goal')
            ]
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            'telemetry',
            default_value='true',
            description='Output telemetry on this robot.'
        )
    )
    telemetry = LaunchConfiguration('telemetry')

    nodes.append(
        Node(
            package='central_nav',
            executable='cmdvel_telemetry_node',
            namespace=ns,
            parameters=[{
                'robot_name': name,
                'use_sim_time': use_sim_time
            }],
            remappings=[
                ('telemetry', '/telemetry')
            ],
            condition=IfCondition(telemetry)
        )
    )

    nodes.append(
        Node(
            package='central_nav',
            executable='state_telemetry_node',
            namespace=ns,
            parameters=[{
                'robot_name': name,
                'use_sim_time': use_sim_time
            }],
            remappings=[
                ('telemetry', '/telemetry'),
                ('goal_status', 'navigate_to_pose/_action/status')
            ],
            condition=IfCondition(telemetry)
        )
    )

    return LaunchDescription(declared_arguments + nodes)