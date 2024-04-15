import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import TimerAction


def generate_launch_description():

    # robot
    robot_name = 'mvp2_test_robot'

    # mvp_mission param
    mvp_mission_path = os.path.join(
        get_package_share_directory('mvp_helm'),
        'config'
        )
    mvp_mission_param_file = os.path.join(mvp_mission_path, 'mvp_mission.yaml') 
    
    # behaviors param
    mvp_teleop_param_file = os.path.join(mvp_mission_path, 'bhv_teleop.yaml') 

    # helm param 
    mvp_helm_path = os.path.join(
        get_package_share_directory('mvp_helm'),
        'mvp_mission_config'
        )
    mvp_helm_config_file = os.path.join(mvp_helm_path, 'helm.yaml') 

    # launch the node
    return LaunchDescription([

        TimerAction(period=0.0,
            actions=[
                    Node(
                        package="mvp_helm",
                        executable="mvp_helm",
                        namespace=robot_name,
                        name="mvp_helm",
                        prefix=['stdbuf -o L'],
                        output="screen",
                        parameters=[
                            {'helm_config_file': mvp_helm_config_file},
                            {'tf_prefix': robot_name},
                            mvp_mission_param_file,
                            mvp_teleop_param_file
                            ]
                        )
            ])
        
])