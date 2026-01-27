import os
import launch
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher


def generate_launch_description():
    package_dir = get_package_share_directory('my_package')
    launch_dir = os.path.join(package_dir, 'launch')




    webots = WebotsLauncher(
    	world=os.path.join(package_dir, 'worlds', 'emptyish_arena.wbt'),
   	    ros2_supervisor=True,

	)


    requester = Node(
        package='my_package',
        executable='requester',
    )

    return LaunchDescription([
        webots,
        webots._supervisor,
        requester,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])

