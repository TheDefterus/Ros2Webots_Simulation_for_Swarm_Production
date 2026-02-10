from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, OpaqueFunction, DeclareLaunchArgument
from launch_ros.actions import SetRemap

def launch_setup(context):
    launches= []
    launches.append(
        IncludeLaunchDescription("/opt/ros/jazzy/share/webots_ros2_epuck" + "/launch/" + "robot_launch.py", launch_arguments={
            'rviz': "True",
            'nav': "True",
        }.items())
    )
    launches.append(
        SetRemap(
            src='/cmd_vel_smoothed',
            dst='/cmd_vel_test'
        )
    )


    return launches

def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            'bobat_namespace',
            default_value="Worldly",
            description='name for a bobat, typically the same followed by _#',
        )
    )
    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )

