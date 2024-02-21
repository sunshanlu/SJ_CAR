#include <iostream>

#include <eigen3/Eigen/Dense>

int main() {
    Eigen::Matrix3d A;
    Eigen::Quaterniond quat(A);
    quat.normalize();
    A << 1, 0, 0, 0, 1, 0, 0, 0, 1;
    std::cout << A << std::endl;
    std::cout << quat.x() << " " << quat.y() << " " << quat.z() << " " << quat.w() << std::endl;
    return 0;
}
