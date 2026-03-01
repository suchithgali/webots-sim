import os
import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.webots_launcher import WebotsLauncher


def generate_launch_description():
    pkg = get_package_share_directory('my_package')

    urdf = os.path.join(pkg, 'resource', 'mavic_webots.urdf')

    webots = WebotsLauncher(
        world=PathJoinSubstitution([pkg, 'worlds', LaunchConfiguration('world')]),
        ros2_supervisor=True,
    )

    driver = WebotsController(
        robot_name='Mavic_2_PRO',
        parameters=[{'robot_description': urdf}],
        respawn=True,
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value='wharehouse.wbt',
        ),
        webots,
        webots._supervisor,
        driver,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        ),
    ])