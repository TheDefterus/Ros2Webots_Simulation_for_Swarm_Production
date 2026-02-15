import os, math
import rclpy

import launch
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, OpaqueFunction, DeclareLaunchArgument, RegisterEventHandler, ExecuteProcess
from launch.substitutions import LaunchConfiguration, FindExecutable

from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher


def generate_spiral(n_points=25):
    # Directions: Right (1,0), Up (0,1), Left (-1,0), Down (0,-1)
    dx, dy = [1, 0, -1, 0], [0, -1, 0, 1]
    x, y = 0, 0
    coords = [(x, y)]

    step_size = 1
    direction = 0

    while len(coords) < n_points:
        # Move in two directions (e.g., Right then Down) before increasing step size
        for _ in range(2):
            for _ in range(step_size):
                if len(coords) >= n_points:
                    return coords
                x += dx[direction]
                y += dy[direction]
                coords.append((x, y))
            direction = (direction + 1) % 4
        step_size += 1
    return coords


def launch_setup(context):
    launches = []
    rviz_configs = []
    package_dir = get_package_share_directory("swarm_prod_sim")
    # rclpy.init(args=[])
    # logger = rclpy.create_node("logger")

    robot_type_cfg = LaunchConfiguration("robot_type")
    number_cfg = LaunchConfiguration("number")
    starting_index_cfg = LaunchConfiguration("starting_index")
    launch_webots_cfg = LaunchConfiguration("webots")
    launch_rviz_cfg = LaunchConfiguration("rviz")

    robot_type = robot_type_cfg.perform(context)
    number = int(number_cfg.perform(context))
    starting_index = int(starting_index_cfg.perform(context))
    launch_webots = bool(launch_webots_cfg.perform(context))
    launch_rviz = bool(launch_rviz_cfg.perform(context))
    bias = 0.25

    # to-do: re implement any bobat launch (More types, URDFs, etc)
    spiral = generate_spiral(number)

    for i in range(starting_index, starting_index + number):
        robot_namespace = robot_type + "_" + str(i + 1)

        robot_to_spawn = "name \\\"{robot_namespace}\\\" translation {robot_translation} ".format(
            robot_namespace=robot_namespace,
            robot_translation="{} {} 0".format(spiral[i][0] * bias, spiral[i][1] * bias),
        )
        robot_string = "\"data: " + robot_type + " { " + robot_to_spawn + "}\""
        # logger.get_logger().info(robot_string) # DEBUG
        launches.append(
            ExecuteProcess(
                name=robot_namespace + "_spawner",
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
        launches.append(
            IncludeLaunchDescription(
                package_dir + "/launch/" + "bobat_number_launch.py", launch_arguments={
                    "bobat_namespace":robot_namespace,
                }.items()
            )
        )
        pass

    # start webots too?
    if launch_webots:
        webots_handle = WebotsLauncher(
            world=os.path.join(package_dir, "worlds", "emptyish_arena.wbt"),
            ros2_supervisor=True,
        )
        launches.append(webots_handle)
        launches.append(webots_handle._supervisor)
        launches.append(
            RegisterEventHandler(
                event_handler=launch.event_handlers.OnProcessExit(
                    target_action=webots_handle,
                    on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
                )
            )
        )

    # start rviz ?
    if launch_rviz:
        rviz_cmd = Node(
            condition=IfCondition(launch_rviz_cfg),
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=['-d', os.path.join(package_dir, "resource", "bobat.rviz")], # add file
        )
        launches.append(rviz_cmd)


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
    declared_arguments.append(
        DeclareLaunchArgument(
            "webots",
            default_value="true",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz",
            default_value="true",
        )
    )
    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
