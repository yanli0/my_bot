import os

from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition

def generate_launch_description():
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    gazebo_world = LaunchConfiguration('gazebo_world', default='~/dev_ws/src/my_bot/worlds/empty.world')
    use_ros2_control = LaunchConfiguration('use_ros2_control', default='true')
    
    #Launch robot_state_publisher
    package_name='my_bot'
    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': use_sim_time, 'use_ros2_control':use_ros2_control}.items()
    )    
    
    #Include gazebo_params file in which Gazebo clock rate can be set
    gazebo_params_path = os.path.join(get_package_share_directory(package_name), 'config', 'gazebo_params.yaml')
    
    #Launch Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]), launch_arguments={'use_sim_time': use_sim_time, 'world': gazebo_world, 'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_path}.items()
    )
    
    #run spawner
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', 
            '-entity', 'my_bot'],
            output='screen'
    )
    
    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["diff_cont"],
        condition=IfCondition(use_ros2_control)
    )
    
    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["joint_broad"],
        condition=IfCondition(use_ros2_control)
    )
    
    delay_diff_drive_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[diff_drive_spawner],
        )
    )
        
    delay_joint_broad_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[joint_broad_spawner],
        )
    )
            
    #Launch them all
    return LaunchDescription([
        rsp,
        gazebo,
        spawn_entity,
        delay_diff_drive_spawner,
        delay_joint_broad_spawner,
    ])
        
