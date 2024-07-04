from ament_index_python import get_package_share_path
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
import yaml
import os
import copy


def recursive_dict_update(old, new):
    for k, v in new.items():
        if isinstance(v, dict):
            old[k] = recursive_dict_update(old.get(k, {}), v)
        else:
            old[k] = copy.deepcopy(v)
    return old


def load_config(config_path):
    with open(config_path) as f:
        config = yaml.load(f.read())
    return config


def render_nav2_config():
    nav2_params = load_config(str(get_package_share_path("docking_navigation") / "config" / "nav2.yaml"))
    costmap_params = load_config(str(get_package_share_path("docking_navigation") / "config" / "nav2.costmap.yaml"))

    # Overlay Nav2 local_costmap params onto common costmap params.
    local_costmap_params = nav2_params["local_costmap"]["local_costmap"][
        "ros__parameters"
    ]
    recursive_dict_update(local_costmap_params, costmap_params)
    # Set local_costmap params in core Nav2 config to the merged params
    nav2_params["local_costmap"]["local_costmap"][
        "ros__parameters"
    ] = local_costmap_params

    # Same for global costmap.
    global_costmap_params = nav2_params["global_costmap"]["global_costmap"][
        "ros__parameters"
    ]
    recursive_dict_update(global_costmap_params, costmap_params)
    nav2_params["global_costmap"]["global_costmap"][
        "ros__parameters"
    ] = global_costmap_params

    # Save final Nav2 config to a file.
    nav2_params_path = "/tmp/nav2." + str(os.getpid()) + ".yaml"
    os.makedirs(os.path.dirname(nav2_params_path), exist_ok=True)
    with open(nav2_params_path, "w") as f:
        yaml.dump(nav2_params, f)

    return nav2_params_path


def generate_launch_description():
    return LaunchDescription(
        [
            # Nav2
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(
                        get_package_share_path("nav2_bringup")
                        / "launch"
                        / "navigation_launch.py"
                    )
                ),
                launch_arguments={
                    "params_file": render_nav2_config(),
                }.items(),
            ),
        ]
    )