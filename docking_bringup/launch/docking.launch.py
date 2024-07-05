from ament_index_python import get_package_share_path
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetUseSimTime

def generate_launch_description():
    return LaunchDescription(
        [
            SetUseSimTime(True),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(
                        get_package_share_path("docking_gazebo")
                        / "launch"
                        / "simulation.launch.py"
                    )
                ),
            ),
            Node(
                name="map_to_odom",
                package="tf2_ros",
                executable="static_transform_publisher",
                arguments=["--frame-id", "map", "--child-frame-id", "odom"],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(
                        get_package_share_path("docking_navigation")
                        / "launch"
                        / "navigation.launch.py"
                    )
                ),
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=[
                    "-d",
                    str(
                        get_package_share_path("docking_bringup")
                        / "rviz"
                        / "docking.rviz"
                    ),
                ],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(
                        get_package_share_path("docking_docking")
                        / "launch"
                        / "docking.launch.py"
                    )
                ),
            ),
        ]
    )
