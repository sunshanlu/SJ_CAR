import os
import math

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory

voc_fp = "/home/sj/Project/SJ_CAR/Vocabulary/ORBvoc.txt"  # 词袋文件路径
setting_fp = "/home/sj/Project/SJ_CAR/config/sj_slam_setting.yaml"  # ORB-SLAM2配置路径
map_fp = "/home/sj/Project/SJ_CAR/map/"  # 地图保存和加载路径

# SJ_TF参数（以m为单位）
trans_x = 0.69
trans_y = 0.0
trans_z = 0.28
rot_yaw = -math.pi / 2
rot_pitch = 0.0
rot_roll = -math.pi / 2


def generate_launch_description():
    rviz_path = os.path.join(get_package_share_directory(
        "orb_slam2"), "rviz", "visual.rviz")
    orb_slam2 = Node(package="orb_slam2",
                     executable="orb_slam2", output="screen",
                     arguments=["--build_map=false", "--track_map=true", "--show_map=false",
                                f"--voc_fp={voc_fp}", f"--setting_fp={setting_fp}", f"--map_fp={map_fp}"],
                     sigterm_timeout="20"
                     )
    camera = Node(package="camera", executable="camera", output="screen")

    pure_pursuit = Node(package="pure_pursuit", executable="pure_pursuit", output="screen")

    rviz = Node(package="rviz2",
                executable="rviz2", output="screen",
                arguments=["-d", rviz_path]
                )
    tf_base_camera = Node(package="tf2_ros", executable="static_transform_publisher",
                          output="screen", arguments=[str(trans_x), str(trans_y), str(trans_z), str(rot_yaw),
                                                      str(rot_pitch), str(rot_roll), "base_link", "camera_link"])
    tf_map_world = Node(package="tf2_ros", executable="static_transform_publisher",
                        output="screen", arguments=[str(trans_y), str(trans_z), str(-trans_x),
                                                    str(math.pi / 2), str(-math.pi / 2), str(0), "map", "world"])
    tf_camera_map = Node(package="sj_tf", executable="sj_tf", output="screen")
    base_link_msg = Node(
        package="sj_tf", executable="BaseLinkPublisher", output="screen")

    orb_slam2.cmd.pop()
    tf_base_camera.cmd.pop()
    tf_map_world.cmd.pop()
    rviz.cmd.pop()

    return LaunchDescription([
        orb_slam2,
        camera,
        rviz,
        tf_base_camera,
        tf_map_world,
        tf_camera_map,
        base_link_msg,
        pure_pursuit
    ])
