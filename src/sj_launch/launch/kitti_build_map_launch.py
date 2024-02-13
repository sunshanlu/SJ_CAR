from launch import LaunchDescription
from launch_ros.actions import Node

voc_fp = "/home/lu/Project/SJ_CAR/Vocabulary/ORBvoc.txt"  # 词袋文件路径
setting_fp = "/home/lu/Project/SJ_CAR/config/KITTI00-02.yaml"  # ORB-SLAM2配置路径
map_fp = "/home/lu/Project/SJ_CAR/map/"  # 地图保存和加载路径


def generate_launch_description():
    orb_slam2 = Node(package="orb_slam2",
                     executable="orb_slam2", output="screen",
                     arguments=["--build_map=true", "--track_map=false", "--show_map=true",
                                f"--voc_fp={voc_fp}", f"--setting_fp={setting_fp}", f"--map_fp={map_fp}"]
                     )
    kitti_odom = Node(package="kitti_odom",
                      executable="kitti_odom", output="screen")
    return LaunchDescription([
        orb_slam2,
        kitti_odom,
    ])
