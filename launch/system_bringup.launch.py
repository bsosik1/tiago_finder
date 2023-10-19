import os
from os import environ, pathsep

from ament_index_python import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.actions import SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
import yaml
from launch_ros.substitutions import FindPackageShare


def get_params_from_yaml(path):
    with open(path, 'r') as file:
        params = yaml.safe_load(file)
    print('YAML params loaded successfully!')

    return params


def get_model_paths(packages_names):
    model_paths = ''
    for package_name in packages_names:
        if model_paths != '':
            model_paths += pathsep

        package_path = get_package_prefix(package_name)
        model_path = os.path.join(package_path, 'share')

        model_paths += model_path

    return model_paths


def get_resource_paths(packages_names):
    resource_paths = ''
    for package_name in packages_names:
        if resource_paths != '':
            resource_paths += pathsep

        package_path = get_package_prefix(package_name)
        resource_paths += package_path

    return resource_paths


def generate_launch_description():

    # READ AND PASS CONFIG FROM YAML FILE

    global_config = get_params_from_yaml(
        os.path.join(get_package_share_directory('tiago_finder'),
                     'config', 'global_config.yaml')
    )

    use_nav = global_config['global_config']['ros__parameters']['use_navigation']
    print(f'Use navigation: {use_nav}')

    use_moveit = global_config['global_config']['ros__parameters']['use_moveit']
    print(f'Use moveit: {use_moveit}')

    world_name = global_config['global_config']['ros__parameters']['world_name']
    print(f'world: {world_name}')

    # DECLARE LAUNCH ARGUMENTS

    navigation_arg = DeclareLaunchArgument(
        'navigation', default_value=use_nav,
        description='Specify whether to launch Nav2'
    )

    moveit_arg = DeclareLaunchArgument(
        'moveit', default_value=use_moveit,
        description='Specify whether to launch MoveIt2'
    )

    world_name_arg = DeclareLaunchArgument(
        'world_name', default_value=world_name,
        description='Specify which world to load'
    )

    # LAUNCH BRINGUP

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('pal_gazebo_worlds'),
                'launch',
                'pal_gazebo.launch.py'
            ])
        ])
    )

    tiago_spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('tiago_gazebo'),
                'launch',
                'tiago_spawn.launch.py'
            ])
        ])
    )

    tiago_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('tiago_bringup'),
                'launch',
                'tiago_bringup.launch.py'
            ])
        ])
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('tiago_2dnav'),
                'launch',
                'tiago_sim_nav_bringup.launch.py'
            ])
        ]),
        condition=IfCondition(LaunchConfiguration('navigation'))
    )

    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('tiago_moveit_config'),
                'launch',
                'move_group.launch.py'
            ])
        ]),
        condition=IfCondition(LaunchConfiguration('moveit'))
    )

    # TUCK ARM NODE

    tuck_arm = Node(package='tiago_gazebo',
                    executable='tuck_arm.py',
                    emulate_tty=True,
                    output='both')

    # GAZEBO RESOURCES

    packages = ['tiago_description', 'pmb2_description',
                'hey5_description', 'pal_gripper_description']
    model_path = get_model_paths(packages)
    resource_path = get_resource_paths(packages)

    if 'GAZEBO_MODEL_PATH' in environ:
        model_path += pathsep + environ['GAZEBO_MODEL_PATH']

    if 'GAZEBO_RESOURCE_PATH' in environ:
        resource_path += pathsep + environ['GAZEBO_RESOURCE_PATH']

    ld = LaunchDescription()
    ld.add_action(SetEnvironmentVariable('GAZEBO_MODEL_PATH', model_path))

    ld.add_action(world_name_arg)
    ld.add_action(gazebo)

    ld.add_action(tiago_spawn)
    ld.add_action(tiago_bringup)

    ld.add_action(navigation_arg)
    ld.add_action(navigation)

    ld.add_action(moveit_arg)
    ld.add_action(moveit)
    ld.add_action(tuck_arm)

    return ld
