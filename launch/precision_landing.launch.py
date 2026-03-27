from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    package_name = 'drone_precision_landing_py'
    package_share_dir = get_package_share_directory(package_name)

    # -------------------------------------------------
    # Config files
    # -------------------------------------------------
    offboard_params = os.path.join(package_share_dir, 'config', 'offboard_params.yaml')
    mission_params = os.path.join(package_share_dir, 'config', 'mission_params.yaml')
    pad_detector_params = os.path.join(package_share_dir, 'config', 'pad_detector_params.yaml')
    ekf_params = os.path.join(package_share_dir, 'config', 'ekf_params.yaml')
    landing_params = os.path.join(package_share_dir, 'config', 'landing_params.yaml')

    # -------------------------------------------------
    # Gazebo camera topics
    # -------------------------------------------------
    image_topic = '/world/default/model/x500_mono_cam_down_0/link/camera_link/sensor/imager/image'
    camera_info_topic = '/world/default/model/x500_mono_cam_down_0/link/camera_link/sensor/imager/camera_info'

    # -------------------------------------------------
    # Gazebo -> ROS 2 bridge
    # -------------------------------------------------
    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='camera_bridge',
        output='screen',
        arguments=[
            f'{image_topic}@sensor_msgs/msg/Image@gz.msgs.Image',
            f'{camera_info_topic}@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
        ],
        remappings=[
            (image_topic, '/camera/image_raw'),
            (camera_info_topic, '/camera/camera_info'),
        ]
    )

    # -------------------------------------------------
    # Core autonomy nodes
    # -------------------------------------------------
    offboard_node = Node(
        package=package_name,
        executable='offboard_control',
        name='offboard_control_node',
        output='screen',
        parameters=[offboard_params]
    )

    mission_node = Node(
        package=package_name,
        executable='mission_manager',
        name='mission_manager_node',
        output='screen',
        parameters=[mission_params]
    )

    pad_detector_node = Node(
        package=package_name,
        executable='pad_detector',
        name='pad_detector_node',
        output='screen',
        parameters=[pad_detector_params]
    )

    ekf_node = Node(
        package=package_name,
        executable='pad_tracker_ekf',
        name='pad_tracker_ekf',
        output='screen',
        parameters=[ekf_params]
    )

    landing_node = Node(
        package=package_name,
        executable='landing_manager',
        name='landing_manager',
        output='screen',
        parameters=[landing_params]
    )

    delayed_stack = TimerAction(
        period=2.0,
        actions=[
            offboard_node,
            mission_node,
            pad_detector_node,
            ekf_node,
            landing_node,
        ]
    )

    return LaunchDescription([
        bridge_node,
        delayed_stack,
    ])