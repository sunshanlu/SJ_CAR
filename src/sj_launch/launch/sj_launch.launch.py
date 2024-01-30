from launch_ros.actions import Node
from launch import LaunchDescription


def generate_launch_description():

    camera = Node(package="camera", executable="camera", output="screen")
    orb_slam2 = Node(package="orb_slam2",
                     executable="orb_slam2_exec", output="screen")

    return LaunchDescription([camera, orb_slam2])
