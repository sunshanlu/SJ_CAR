#include "SjTf.h"

SjTf::SjTf()
    : Node("SJ_TF") {
    mp_cameraPose = create_subscription<Pose>("camera/pose", 10, std::bind(&SjTf::pubDynamicTF, this, _1));
    mp_dynamicTfPub = std::make_shared<tf2_ros::TransformBroadcaster>(this);
}

/**
 * @brief 以camera_link为父坐标系，发布map坐标系相对于camera_link的位姿Tcm
 *
 * @param poseMsg 输出的位姿消息
 */
void SjTf::pubDynamicTF(Pose::SharedPtr poseMsg) {
    TransformStamped tfCamera2Map;
    tfCamera2Map.header.stamp = now();
    tfCamera2Map.header.frame_id = "camera_link";
    tfCamera2Map.child_frame_id = "map";

    tfCamera2Map.transform.translation.x = poseMsg->position.x;
    tfCamera2Map.transform.translation.y = poseMsg->position.y;
    tfCamera2Map.transform.translation.z = poseMsg->position.z;

    tfCamera2Map.transform.rotation.x = poseMsg->orientation.x;
    tfCamera2Map.transform.rotation.y = poseMsg->orientation.y;
    tfCamera2Map.transform.rotation.z = poseMsg->orientation.z;
    tfCamera2Map.transform.rotation.w = poseMsg->orientation.w;

    mp_dynamicTfPub->sendTransform(tfCamera2Map);
}


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    auto sj_tf = std::make_shared<SjTf>();
    rclcpp::spin(sj_tf);
    rclcpp::shutdown();
    return 0;
}
