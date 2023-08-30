from ament_index_python import get_package_share_directory
import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os

def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package='husky_simulation').find('husky_simulation')
    warehouse_dir = get_package_share_directory('aws_robomaker_small_warehouse_world')
    rviz_nav2 = get_package_share_directory('husky_navigation')

    default_model_path = os.path.join(pkg_share, 'urdf/husky_ual.urdf.xacro')
    default_rviz_config_path = os.path.join(rviz_nav2, 'rviz/nav2_default_view.rviz')
    world_path = os.path.join(pkg_share, 'worlds/industrial_warehouse.world')
       
    declare_world_cmd = launch.actions.DeclareLaunchArgument(
        'world',
        default_value=world_path,
        description='Full path to world model file to load')
    

    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': launch_ros.parameter_descriptions.ParameterValue(
                                            Command(['xacro ', LaunchConfiguration('model')]), 
                                            value_type=str)}]
    )

    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        condition=launch.conditions.UnlessCondition(LaunchConfiguration('gui'))
    )

    spawn_entity = launch_ros.actions.Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        arguments=['-entity', 'husky', '-topic', 'robot_description',
                   '-x', LaunchConfiguration('x_pose'),
                   '-y', LaunchConfiguration('y_pose'),
                   '-z', LaunchConfiguration('z_pose'),
                   '-R', LaunchConfiguration('roll_pose'),
                   '-P', LaunchConfiguration('pitch_pose'),
                   '-Y', LaunchConfiguration('yaw_pose')],
        output='screen'
    )
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )    


    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='gui', default_value='True',
                                             description='Flag to enable joint_state_publisher_gui'),
        launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path,
                                             description='Absolute path to robot urdf file'),
        launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                             description='Absolute path to rviz config file'),
        launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='True',
                                             description='Flag to enable use_sim_time'),
        launch.actions.DeclareLaunchArgument(name='x_pose', default_value='0.0',
                                             description='Initial x-axis pose of the robot'),
        launch.actions.DeclareLaunchArgument(name='y_pose', default_value='0.0',
                                             description='Initial y-axis pose of the robot'),
        launch.actions.DeclareLaunchArgument(name='z_pose', default_value='0.0',
                                             description='Initial z-axis pose of the robot'),
        launch.actions.DeclareLaunchArgument(name='roll_pose', default_value='0.0',
                                             description='Initial roll pose of the robot'),
        launch.actions.DeclareLaunchArgument(name='pitch_pose', default_value='0.0',
                                             description='Initial pitch pose of the robot'),
        launch.actions.DeclareLaunchArgument(name='yaw_pose', default_value='0.0',
                                             description='Initial yaw pose of the robot'),
        launch.actions.ExecuteProcess(cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', world_path], output='screen'),
        joint_state_publisher_node,
        robot_state_publisher_node,
        spawn_entity,
        declare_world_cmd,
        rviz_node
    ])





