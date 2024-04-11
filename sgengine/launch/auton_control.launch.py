from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(package="sgengine", executable="auto", name="auton_auto"),
            Node(
                package="sgengine",
                executable="pico",
                name="auton_pico",
                arguments=["--no_duplicates"],
                respawn=True,
                respawn_delay=1,
            ),
            Node(
                package="sgengine",
                executable="odometry",
                name="auton_odometry",
                respawn=True,
                respawn_delay=1,
            ),
            Node(
                package="sgengine_cpp",
                executable="obstacle_map",
                name="auton_obstacle_map",
                respawn=True,
                respawn_delay=1,
            ),
            Node(
                package="sgengine_cpp",
                executable="grid_path_finder",
                name="auton_grid_path_finder",
                respawn=True,
                respawn_delay=1,
            ),
            Node(
                package="sgengine",
                executable="oak",
                name="auton_oak",
                respawn=True,
                respawn_delay=5,
            ),
            Node(
                package="depth_image_proc",
                executable="point_cloud_xyz_node",
                name="auton_point_cloud_xyz_node",
                remappings=[
                    ("/image_rect", "oak/depth_image"),
                    ("/camera_info", "oak/depth_camera_info"),
                    ("/points", "/full_point_cloud"),
                ],
                parameters=[{"output_frame": "camera_link"}],
            ),
        ]
    )
