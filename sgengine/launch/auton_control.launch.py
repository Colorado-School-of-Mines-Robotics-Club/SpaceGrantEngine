from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            # Node(package="sgengine", executable="auto", name="auton_auto"),
            # Node(
            #     package="sgengine",
            #     executable="pico",
            #     name="auton_pico",
            #     arguments=["--no_duplicates"],
            #     respawn=True,
            #     respawn_delay=1,
            # ),
            # Node(
            #     package="sgengine",
            #     executable="odometry",
            #     name="auton_odometry",
            #     respawn=True,
            #     respawn_delay=1,
            # ),
            # Node(
            #     package="sgengine_cpp",
            #     executable="obstacle_map",
            #     name="auton_obstacle_map",
            #     respawn=True,
            #     respawn_delay=1,
            # ),
            # Node(
            #     package="sgengine_cpp",
            #     executable="grid_path_finder",
            #     name="auton_grid_path_finder",
            #     respawn=True,
            #     respawn_delay=1,
            # ),
            Node(
                package="sgengine",
                executable="oak",
                name="auton_oak",
                respawn=True,
                respawn_delay=5,
            ),
            Node(
                package="rtabmap_odom",
                executable="stereo_odometry",
                name="stereo_odometry",
                output="screen",
                parameters=[
                    {"approx_sync": True},
                    {"queue_size": 10},
                    {"frame_id": "camera_link"},
                    {"odom_frame_id": "odom"},
                    {"subscribe_stereo": True},
                    {"left_image_topic": "oak/left_image"},
                    {"right_image_topic": "oak/right_image"},
                    {"left_camera_info_topic": "oak/left_camera_info"},
                    {"right_camera_info_topic": "oak/right_camera_info"},
                ],
                remappings=[
                    ("left/image_rect", "oak/left_image"),
                    ("right/image_rect", "oak/right_image"),
                    ("left/camera_info", "oak/left_camera_info"),
                    ("right/camera_info", "oak/right_camera_info"),
                ],
            ),
            # Node(
            #     package="depth_image_proc",
            #     executable="point_cloud_xyz_node",
            #     name="auton_point_cloud_xyz_node",
            #     remappings=[
            #         ("/image_rect", "oak/depth_image"),
            #         ("/camera_info", "oak/depth_camera_info"),
            #         ("/points", "/full_point_cloud"),
            #     ],
            #     parameters=[{"output_frame": "camera_link"}],
            # ),
        ]
    )
