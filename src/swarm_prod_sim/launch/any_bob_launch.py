from math import floor

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, OpaqueFunction, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import FindExecutable
from launch.actions import ExecuteProcess

from ament_index_python.packages import get_package_share_directory

import rclpy

enum = {
    0:"-6 0 0",
    1:"6 0 0",
    2:"-6 -8 0",
    3:"6 -8 0", }

def launch_setup(context):
    package_dir = get_package_share_directory("my_package")
    # rclpy.init(args=[])
    # logger = rclpy.create_node("logger")

    number_config=LaunchConfiguration('number')
    number=int(number_config.perform(context))

    robot_type = "bob"
    robot_name = "bob_{robot_i}"

    bias = 2

    launches = []
    for i in range(number):
        bobat_name = '{robot_name}'.format(robot_name=robot_name).format(robot_i=i+1)
        # logger.get_logger().info(bobat_name)
        # launches.append(
        #     IncludeLaunchDescription(package_dir + "/launch/" + "bobat_number_launch.py", launch_arguments={
        #     'bobat_namespace': bobat_name,
        #     }.items())
        #     )
        trans_x = number/2*(-bias) + i * bias
        if number == 4:
            robot_to_spawn = "name \\\"{robot_name}\\\" translation {robot_translation} ".format(
                robot_name=bobat_name,
                robot_translation=str(enum[i])
            )
            robot_string = "\"data: " + robot_type + " { " + robot_to_spawn + "}\""
        else:
            robot_to_spawn = "name \\\"{robot_name}\\\" translation {robot_translation} ".format(
                robot_name=bobat_name,
                robot_translation=str(trans_x) + " 0 0",)
            robot_string = "\"data: " + robot_type + " { " + robot_to_spawn + "}\""
        # logger.get_logger().info(robot_string)
        launches.append(
            ExecuteProcess(
                name=bobat_name + "_spawner",
                cmd=[[
                    FindExecutable(name="ros2"),
                    " service call ",
                    "/Ros2Supervisor/spawn_node_from_string ",
                    "webots_ros2_msgs/srv/SpawnNodeFromString ",
                    robot_string,
                ]],
                shell=True
            ),
        )
        # logger.get_logger().info(str(i+1) + " iteration done")
    return launches

def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            'number',
            default_value="1",
            description='number of bobats to spawn',
        )
    )
    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])