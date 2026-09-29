"""Standalone ROS 2 Humble + Gazebo Fortress + MoveIt + basic/advanced LLM planner."""
from pathlib import Path
import tempfile
from uuid import uuid4

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction, IncludeLaunchDescription,
                            OpaqueFunction, RegisterEventHandler, SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro

from ur3_llm_control.configuration import read_yaml
from ur3_llm_control.simulation import add_parallel_gripper, add_parallel_gripper_semantic, world_sdf


def setup(context):
    share = Path(get_package_share_directory('ur3_llm_control'))
    ur_share = Path(get_package_share_directory('ur_description'))
    moveit_share = Path(get_package_share_directory('ur_moveit_config'))
    scene_file = LaunchConfiguration('scene_file').perform(context)
    scene = read_yaml(scene_file)
    controllers_file = str(share / 'config/controllers.yaml')
    urdf = xacro.process_file(str(ur_share / 'urdf/ur.urdf.xacro'), mappings={
        'name': 'ur', 'ur_type': LaunchConfiguration('ur_type').perform(context),
        'sim_ignition': 'true', 'simulation_controllers': controllers_file,
        'safety_limits': 'true',
    }).toxml()
    # Fail explicitly if a newer incompatible UR description is installed.
    if 'ign_ros2_control' not in urdf:
        raise RuntimeError('Cần ur_description bản Humble có sim_ignition/ign_ros2_control')
    urdf = add_parallel_gripper(urdf)
    srdf = add_parallel_gripper_semantic(xacro.process_file(str(moveit_share / 'srdf/ur.srdf.xacro'),
                                                mappings={'name': 'ur', 'prefix': ''}).toxml())
    robot = {'robot_description': urdf, 'robot_description_semantic': srdf}
    temp_dir = Path(tempfile.mkdtemp(prefix='ur3_llm_'))
    world = temp_dir / 'llm_lab.sdf'
    world.write_text(world_sdf(scene))
    description_file = temp_dir / 'ur3.urdf'
    description_file.write_text(urdf)

    # This launch uses no files from the week-1 packages or the downloaded upstream repo.
    gz = IncludeLaunchDescription(PythonLaunchDescriptionSource(str(
        Path(get_package_share_directory('ros_gz_sim')) / 'launch/gz_sim.launch.py')),
        launch_arguments={'gz_args': '-s -r -v 2 ' + str(world)}.items())
    gui = IncludeLaunchDescription(PythonLaunchDescriptionSource(str(
        Path(get_package_share_directory('ros_gz_sim')) / 'launch/gz_sim.launch.py')),
        launch_arguments={'gz_args': '-g -v 2'}.items(),
        condition=IfCondition(LaunchConfiguration('gazebo_gui')))
    spawn = Node(package='ros_gz_sim', executable='create', output='screen',
                 arguments=['-world', scene['world_name'], '-name', 'ur', '-file', str(description_file)])
    state = Node(package='robot_state_publisher', executable='robot_state_publisher', output='screen',
                 parameters=[{'robot_description': urdf, 'use_sim_time': True}])
    jsb = Node(package='controller_manager', executable='spawner', output='screen',
               arguments=['joint_state_broadcaster', '-c', '/controller_manager', '--controller-manager-timeout', '90'])
    jtc = Node(package='controller_manager', executable='spawner', output='screen',
               arguments=['joint_trajectory_controller', '-c', '/controller_manager', '--controller-manager-timeout', '90'])
    gripper_controller = Node(package='controller_manager', executable='spawner', output='screen',
               arguments=['gripper_controller', '-c', '/controller_manager', '--controller-manager-timeout', '90'])
    bridge = Node(package='ros_gz_bridge', executable='parameter_bridge', output='screen', arguments=[
        '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
        f"/world/{scene['world_name']}/set_pose@ros_gz_interfaces/srv/SetEntityPose",
    ])
    ompl = {'planning_plugin': 'ompl_interface/OMPLPlanner',
            'request_adapters': ' '.join([
                'default_planner_request_adapters/AddTimeOptimalParameterization',
                'default_planner_request_adapters/FixWorkspaceBounds',
                'default_planner_request_adapters/FixStartStateBounds',
                'default_planner_request_adapters/FixStartStateCollision',
                'default_planner_request_adapters/FixStartStatePathConstraints']),
            'start_state_max_bounds_error': 0.1}
    ompl.update(read_yaml(moveit_share / 'config/ompl_planning.yaml'))
    ompl['ur_manipulator']['longest_valid_segment_fraction'] = 0.002
    controllers = read_yaml(moveit_share / 'config/controllers.yaml')
    controllers['scaled_joint_trajectory_controller']['default'] = False
    controllers['joint_trajectory_controller']['default'] = True
    kin = str(moveit_share / 'config/kinematics.yaml')
    common = [robot, kin, {'use_sim_time': True,
        'robot_description_planning': read_yaml(moveit_share / 'config/joint_limits.yaml')}]
    move_group = Node(package='moveit_ros_move_group', executable='move_group', output='screen',
        parameters=common + [{'move_group': ompl,
            'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager',
            'moveit_simple_controller_manager': controllers,
            'moveit_manage_controllers': False,
            'publish_robot_description_semantic': True,
            'publish_planning_scene': True, 'publish_geometry_updates': True,
            'publish_state_updates': True, 'publish_transforms_updates': True,
            'trajectory_execution.allowed_start_tolerance': 0.20,
            'trajectory_execution.allowed_execution_duration_scaling': 2.0,
            'trajectory_execution.allowed_goal_duration_margin': 3.0,
        }])
    rviz = Node(package='rviz2', executable='rviz2', output='log',
        arguments=['-d', str(share / 'config/scene.rviz')], parameters=common,
        condition=IfCondition(LaunchConfiguration('launch_rviz')))
    planner = Node(package='ur3_llm_control', executable='llm_planner_node', output='screen',
        parameters=[{'use_sim_time': True, 'scene_file': scene_file,
                     'student_file': LaunchConfiguration('student_file')}])
    return [GroupAction([gz]), GroupAction([gui]), state, bridge,
            RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=[jsb, jtc, gripper_controller])),
            spawn, move_group, rviz, planner]


def generate_launch_description():
    share = Path(get_package_share_directory('ur3_llm_control'))
    partition = 'ur3_llm_' + uuid4().hex
    return LaunchDescription([
        DeclareLaunchArgument('ur_type', default_value='ur3', choices=['ur3', 'ur3e']),
        DeclareLaunchArgument('gazebo_gui', default_value='true'),
        DeclareLaunchArgument('launch_rviz', default_value='true'),
        DeclareLaunchArgument('scene_file', default_value=str(share / 'config/scene.yaml')),
        DeclareLaunchArgument('student_file', default_value=str(share / 'config/student_config.yaml')),
        SetEnvironmentVariable('IGN_PARTITION', partition),
        SetEnvironmentVariable('GZ_PARTITION', partition),
        OpaqueFunction(function=setup),
    ])
