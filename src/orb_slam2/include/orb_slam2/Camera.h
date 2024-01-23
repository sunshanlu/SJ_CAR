#pragma once
#include <librealsense2/rs.hpp>
#include <memory>
#include <opencv2/opencv.hpp>

class Camera {
public:
    typedef std::shared_ptr<Camera> SharedPtr;
    static SharedPtr getInstance();

    Camera(const Camera &) = delete;

    void getFrame(cv::Mat &leftImg, cv::Mat &rightImg, double &timestamp);
    static unsigned m_times;

private:
    Camera();

    rs2::config cfg;
    rs2::pipeline pipe;
};