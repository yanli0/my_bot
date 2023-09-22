
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
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    #Launch robot_state_publisher
    package_name='my_bot'
    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': use_sim_time}.items()
    )    
    
    robot_description = Command(['ros2 param get --hide-type /robot_state_publisher robot_description'])
    controller_params = os.path.join(
        get_package_share_directory('my_bot'), 
        'config',
        'my_bot_controllers.yaml'
        )
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[{'robot_description': robot_description},
                controller_params],
    )
    
    #launch controller manager after robot state publisher have finished starting up
    delay_controller_manager = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=rsp
            on_exit=[controller_manager],
        )
    )
    
    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["diff_cont"],
    )
    
    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["joint_broad"],
    )
    
    delay_diff_drive_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=controller_manager,
            on_exit=[diff_drive_spawner],
        )
    )
        
    delay_joint_broad_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=controller_manager,
            on_exit=[joint_broad_spawner],
        )
    )
            
    #Launch them all
    return LaunchDescription([
        rsp,
        delay_controller_manager,
        delay_diff_drive_spawner,
        delay_joint_broad_spawner,
    ])
        
