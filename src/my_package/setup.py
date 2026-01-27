from setuptools import setup

package_name = 'my_package'
data_files = []
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name + '/launch', ['launch/launch_world_and_worldly.py']))
data_files.append(('share/' + package_name + '/launch', ['launch/launch_worldly_bobat.py']))
data_files.append(('share/' + package_name + '/launch', ['launch/test_launch.py']))
data_files.append(('share/' + package_name + '/launch', ['launch/many_bots_launch.py']))
data_files.append(('share/' + package_name + '/launch', ['launch/bobat_number_launch.py']))
data_files.append(('share/' + package_name + '/launch', ['launch/launch_5_bobat.py']))
data_files.append(('share/' + package_name + '/worlds', ['worlds/many_bots.wbt']))
data_files.append(('share/' + package_name + '/worlds', ['worlds/my_world.wbt']))
data_files.append(('share/' + package_name + '/worlds', ['worlds/emptyish_arena.wbt']))
data_files.append(('share/' + package_name + '/resource', ['resource/my_robot.urdf']))
data_files.append(('share/' + package_name + '/resource', ['resource/Webots_robot_string.wbt']))
data_files.append(('share/' + package_name + '/resource', ['resource/bobat.proto']))
data_files.append(('share/' + package_name, ['package.xml']))

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user.name@mail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'my_robot_driver = my_package.my_robot_driver:main',
            'requester= my_package.requester:main',
            'obstacle_avoider = my_package.obstacle_avoider:main',
        ],
    },
)
