from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, OpaqueFunction
from ament_index_python.packages import get_package_share_directory


def launch_setup(context):
    package_dir = get_package_share_directory("my_package")
    launches = []
    for i in range(5):
        bobat_name = 'bobat_{}'.format(i+1)
        launches.append(
            IncludeLaunchDescription(package_dir + "/launch/" + "bobat_number_launch.py", launch_arguments={
            'bobat_namespace': bobat_name,
            }.items())
            )
    return launches

def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=launch_setup)])