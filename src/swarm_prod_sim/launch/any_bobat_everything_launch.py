import os, math
import rclpy

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, OpaqueFunction, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import FindExecutable
from launch.actions import ExecuteProcess

from ament_index_python.packages import get_package_share_directory


def launch_setup(context):
    launches = []
    package_dir = get_package_share_directory("swarm_prod_sim")
    rclpy.init(args=[])
    logger = rclpy.create_node("logger")

    robot_type_cfg = LaunchConfiguration("robot_type")
    number_cfg = LaunchConfiguration("number")
    starting_index_cfg = LaunchConfiguration("starting_index")

    robot_type = robot_type_cfg.perform(context)
    number = int(number_cfg.perform(context))
    starting_index = int(starting_index_cfg.perform(context))
    bias = 0.25

    # to-do: re implement any bobat launch

    for i in range(starting_index, starting_index + number):
        robot_namespace = robot_type + "_" + str(i)

        launches.append(
            IncludeLaunchDescription(
                package_dir + "/launch/" + "bobat_number_launch.py", launch_arguments={
                    "bobat_namespace":robot_namespace,
                }.items()
            )
        )




    return launches


def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_type",
            default_value="bobat",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "number",
            default_value="1",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "starting_index",
            default_value="0",
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
