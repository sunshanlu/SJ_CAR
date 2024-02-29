import math
import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory


# ORB-SLAM2参数
voc_fp = "/home/sj/Project/SJ_CAR/Vocabulary/ORBvoc.txt"  # 词袋文件路径
setting_fp = "/home/sj/Project/SJ_CAR/config/sj_slam_setting.yaml"  # ORB-SLAM2配置路径
map_fp = "/home/sj/Project/SJ_CAR/map/"  # 地图保存和加载路径

# SJ_TF参数（以m为单位）
trans_x = 1.0
trans_y = 0.0
trans_z = 0.3
rot_yaw = -math.pi / 2
rot_pitch = 0.0
rot_roll = -math.pi / 2


def generate_launch_description():
    orb_slam2_rviz_fp = os.path.join(
        get_package_share_directory("orb_slam2"), "rviz", "visual.rviz")

    # ORB_SLAM2以构建地图方式运行时候，设置节点关闭时间为10分钟
    #! 注意，sigterm_timeout参数可以改变节点关闭的等待时间
    orb_slam2 = Node(package="orb_slam2",
                     executable="orb_slam2", output="screen",
                     arguments=["--build_map=true", "--track_map=false", "--show_map=false",
                                f"--voc_fp={voc_fp}", f"--setting_fp={setting_fp}", f"--map_fp={map_fp}"],
                     sigterm_timeout="600")
    # camera = Node(package="camera", executable="camera", output="screen")
    rviz = Node(package="rviz2", executable="rviz2", output="screen",
                arguments=["-d", orb_slam2_rviz_fp])
    tf_base_camera = Node(package="tf2_ros", executable="static_transform_publisher",
                          output="screen", arguments=[str(trans_x), str(trans_y), str(trans_z), str(rot_yaw),
                                                      str(rot_pitch), str(rot_roll), "base_link", "camera_link"])
    tf_map_world = Node(package="tf2_ros", executable="static_transform_publisher",
                        output="screen", arguments=[str(trans_y), str(trans_z), str(-trans_x), str(math.pi / 2),
                                                    str(-math.pi / 2), str(0), "map", "world"])
    tf_camera_map = Node(package="sj_tf", executable="sj_tf", output="screen")
    # base_link_msg = Node(package="sj_tf", executable="BaseLinkPublisher", output="screen", prefix=['gnome-terminal -e'])
    base_link_msg = Node(
        package="sj_tf", executable="BaseLinkPublisher", output="screen")

    orb_slam2.cmd.pop()
    tf_base_camera.cmd.pop()
    tf_map_world.cmd.pop()
    rviz.cmd.pop()
    base_link_msg.cmd.pop()

    return LaunchDescription([
        # camera,
        orb_slam2,
        rviz,
        tf_base_camera,
        tf_map_world,
        tf_camera_map,
        base_link_msg
    ])
