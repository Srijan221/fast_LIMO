from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PythonExpression, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    # --- args (keep same UX as your fast_limo.launch.py) ---
    rviz_cfg = LaunchConfiguration('rviz')

    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='True',
        description='Whether to run an rviz instance'
    )

    # Accumulator tunables (exposed as launch args with sane defaults)
    cloud_topic_arg = DeclareLaunchArgument(
        'cloud_topic', default_value=TextSubstitution(text='/fast_limo/pointcloud'),
        description='Per-frame cloud to accumulate (e.g. /fast_limo/final_raw or /fast_limo/deskewed)'
    )
    odom_topic_arg = DeclareLaunchArgument(
        'odom_topic', default_value=TextSubstitution(text='/fast_limo/state'),
        description='Odometry topic providing pose for each cloud'
    )
    output_topic_arg = DeclareLaunchArgument(
        'output_topic', default_value=TextSubstitution(text='/fast_limo/final_map'),
        description='Latched map topic to publish on demand'
    )
    world_frame_arg = DeclareLaunchArgument(
        'world_frame', default_value=TextSubstitution(text='map'),
        description='World frame id used for the accumulated map'
    )
    voxel_leaf_arg = DeclareLaunchArgument(
        'voxel_leaf_size', default_value=TextSubstitution(text='0.1'),
        description='Voxel filter leaf size in meters (0.0 disables downsampling)'
    )
    max_points_arg = DeclareLaunchArgument(
        'max_points', default_value=TextSubstitution(text='8000000'),
        description='Maximum point count before clearing to prevent OOM'
    )
    accept_delay_arg = DeclareLaunchArgument(
        'accept_delay_sec', default_value=TextSubstitution(text='0.1'),
        description='Max allowed time skew (sec) between cloud and odom'
    )
    min_pub_arg = DeclareLaunchArgument(
        'min_publish_period_sec', default_value=TextSubstitution(text='0.5'),
        description='Min seconds between auto-publishes after updates'
    )
    pub_arg = DeclareLaunchArgument(
        'publish_period_sec', default_value=TextSubstitution(text='0.0'),
        description='Periodic publish period in seconds (0 disables timer)'
    )
    # --- fast_limo node (unchanged except file name) ---
    limo_node = Node(
        package='fast_limo',
        namespace='',
        executable='fast_limo_multi_exec',
        name='fast_limo',
        output='screen',
        parameters=[PathJoinSubstitution([
            FindPackageShare('fast_limo'),
            'config',
            'params.yaml'
        ])]
    )

    # --- accumulate map node (new) ---
    accumulate_node = Node(
        package='fast_limo',
        namespace='',
        executable='accumulate_map_node',
        name='fast_limo_accumulate_map',
        output='screen',
        parameters=[{
            'cloud_topic': LaunchConfiguration('cloud_topic'),
            'odom_topic': LaunchConfiguration('odom_topic'),
            'output_topic': LaunchConfiguration('output_topic'),
            'world_frame': LaunchConfiguration('world_frame'),
            'voxel_leaf_size': LaunchConfiguration('voxel_leaf_size'),
            'max_points': LaunchConfiguration('max_points'),
            'accept_delay_sec': LaunchConfiguration('accept_delay_sec'),
            'min_publish_period_sec': LaunchConfiguration('min_publish_period_sec'),
            'publish_period_sec': LaunchConfiguration('publish_period_sec'),
        }]
    )

    # --- optional RViz, same pattern as your file ---
    rviz_conditioned = ExecuteProcess(
        condition=IfCondition(PythonExpression([rviz_cfg])),
        cmd=[[
            'ros2 run rviz2 rviz2 -d ',
            PathJoinSubstitution([
                FindPackageShare('fast_limo'),
                'config',
                'rviz',
                'limo.rviz'
            ])
        ]],
        shell=True
    )

    return LaunchDescription([
        # args
        rviz_arg,
        cloud_topic_arg, odom_topic_arg, output_topic_arg,
        world_frame_arg, voxel_leaf_arg, max_points_arg, accept_delay_arg, min_pub_arg, pub_arg,

        # nodes
        limo_node,
        accumulate_node,

        # tools
        rviz_conditioned
    ])

