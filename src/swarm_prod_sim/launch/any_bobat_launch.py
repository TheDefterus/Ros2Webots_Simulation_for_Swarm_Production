from math import floor

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, OpaqueFunction, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import FindExecutable
from launch.actions import ExecuteProcess

from ament_index_python.packages import get_package_share_directory

import rclpy




def launch_setup(context):
    package_dir = get_package_share_directory("swarm_prod_sim")
    # rclpy.init(args=[])
    # logger = rclpy.create_node("logger_minus_1")

    number_config=LaunchConfiguration('number')
    number=int(number_config.perform(context))


    robot_type = "bobat" # LaunchConfiguration("robot_type")
    robot_name = "bobat_{robot_i}"

    bias = 0.25

    launches = []
    for i in range(number):
        bobat_name = '{robot_name}'.format(robot_name=robot_name).format(robot_i=i+1)
        # logger.get_logger().info(bobat_name)
        launches.append(
            IncludeLaunchDescription(package_dir + "/launch/" + "bobat_number_launch.py", launch_arguments={
            'bobat_namespace': bobat_name,
            }.items())
            )
        trans = floor(number/2)*(-bias) + i * bias

        robot_to_spawn = "name \\\"{robot_name}\\\" translation {robot_translation} ".format(
            robot_name=bobat_name,
            robot_translation=str(trans) + " 0 0",)
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
    declared_arguments.append(
        DeclareLaunchArgument(
            'robot_type',
            default_value="bobat",
            description='what kinda robot? rn its only bobat',
        )
    )
    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])