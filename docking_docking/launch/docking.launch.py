from ament_index_python import get_package_share_path
from launch import LaunchDescription
from launch_ros.actions import Node, SetUseSimTime

def generate_launch_description():
    return LaunchDescription(
        [
            SetUseSimTime(True),
            Node(
                package="opennav_docking",
                executable="opennav_docking",
                name="docking_server",
                parameters=[
                    str(
                        get_package_share_path("docking_docking")
                        / "config"
                        / "nav2_docking_server.yaml"
                    ),
                    { # NOTE: For some reason SetUseSimTime(True) doesn't work in this launch file???
                        "use_sim_time": True
                    }
                ],
                remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
            ),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='nav2_docking_lifecycle_manager',
                parameters=[
                    {
                        'autostart': True,
                        'node_names': ['docking_server']
                    },
                    {
                        "use_sim_time": True
                    }
                ],
            ),
            Node(
                package="apriltag_ros",
                executable="apriltag_node",
                parameters=[
                    str(
                        get_package_share_path("docking_docking")
                        / "config"
                        / "apriltag_node.yaml"
                    ),
                    {
                        "use_sim_time": True
                    }
                ],
                remappings={
                    "camera_info": "front_cam/color/camera_info",
                    "image_rect": "front_cam/color/image_raw",
                    "detections": "april_tags",
                }.items(),
            ),
            Node(
                # Waits for the appropriate april tag detection and publishes the pose of the detected dock.
                package="docking_docking",
                executable="apriltag_dock_detector",
                parameters=[
                    {
                        "use_sim_time": True
                    }
                ],
            ),
        ]
    )
