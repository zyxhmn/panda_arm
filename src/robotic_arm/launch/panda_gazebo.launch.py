import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_share = get_package_share_directory('robotic_arm')
    
    # 路径
    gazebo_models_path = os.path.join(pkg_share, 'models')
    model_path = os.path.join(pkg_share, 'models', 'robot.urdf.xacro')
    world_path = os.path.join(pkg_share, 'worlds', 'pick_and_place.world')
    rviz_config_path = os.path.join(pkg_share, 'config', 'panda_view.rviz')
    controllers_file = os.path.join(pkg_share, 'config', 'panda_controllers.yaml')
    
    # 处理URDF文件
    robot_description_content = Command([
        FindExecutable(name='xacro'), ' ', model_path
    ])
    robot_description = {'robot_description': robot_description_content}
    
    # 启动Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        launch_arguments={'world': world_path}.items()
    )
    
    # 将机器人模型导入Gazebo
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                  '-entity', 'panda'],
                        output='screen')
    
    # 启动控制器
    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )
    
    load_arm_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'panda_arm_controller'],
        output='screen'
    )
    
    load_hand_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
              'panda_hand_controller'],
        output='screen'
    )
    
    # 启动RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen'
    )
    
    # 添加Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )
    
    # 控制器管理器
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, controllers_file],
        output='screen',
    )
    
    # 事件处理
    load_controllers = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[load_joint_state_broadcaster,
                    load_arm_controller,
                    load_hand_controller],
        )
    )
    
    # 返回启动描述
    return LaunchDescription([
        # 设置Gazebo模型路径环境变量
        ExecuteProcess(
            cmd=['bash', '-c', 'export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:' + gazebo_models_path],
            output='screen'
        ),
        # 节点
        gazebo,
        robot_state_publisher,
        spawn_entity,
        load_controllers,
        controller_manager,
        rviz_node
    ])