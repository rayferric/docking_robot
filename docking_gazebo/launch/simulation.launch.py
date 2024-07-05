from ament_index_python import get_package_share_path
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import SetUseSimTime

def generate_launch_description():
    gui_path = str(get_package_share_path("docking_gazebo") / "config" / "gazebo_gui.config")
    world_path = str(get_package_share_path("docking_gazebo") / "sdf" / "warehouse.sdf")

    return LaunchDescription(
        [
            SetUseSimTime(True),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(
                        get_package_share_path("ros_gz_sim")
                        / "launch"
                        / "gz_sim.launch.py"
                    )
                ),
                launch_arguments={
                    "gz_args": f"-r --gui-config '{gui_path}' '{world_path}'",
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(
                        get_package_share_path("panther_gazebo")
                        / "launch"
                        / "simulate_robot.launch.py"
                    )
                ),
                launch_arguments={
                    "components_config_path": str(
                        get_package_share_path("docking_gazebo")
                        / "config"
                        / "components.yaml"
                    ),
                }.items(),
            ),
        ]
    )