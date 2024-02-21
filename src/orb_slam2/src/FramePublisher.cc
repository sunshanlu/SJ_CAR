#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <rclcpp/rclcpp.hpp>

#include "FramePublisher.h"
#include "Tracking.h"

namespace ORB_SLAM2 {

FramePublisher::FramePublisher(rclcpp::Publisher<Image>::SharedPtr imagePub,
                               rclcpp::Publisher<Image>::SharedPtr imagePubRight)
    : mImagePub(imagePub)
    , mImagePubRight(imagePubRight) {
    mState = Tracking::SYSTEM_NOT_READY;
    mIm = cv::Mat(480, 640, CV_8UC3, cv::Scalar(0, 0, 0));
    mbUpdated = true;
    PublishFrame(rclcpp::Time(0));
}

void FramePublisher::SetMap(Map *pMap) { mpMap = pMap; }

void FramePublisher::Refresh(const rclcpp::Time &stamp) {
    if (mbUpdated) {
        PublishFrame(stamp);
        mbUpdated = false;
    }
}

cv::Mat FramePublisher::DrawFrame() {
    cv::Mat im;
    vector<cv::KeyPoint> vIniKeys;        // Initialization: KeyPoints in reference frame
    vector<int> vMatches;                 // Initialization: correspondeces with reference keypoints
    vector<cv::KeyPoint> vCurrentKeys;    // KeyPoints in current frame
    vector<MapPoint *> vMatchedMapPoints; // Tracked MapPoints in current frame
    int state;                            // Tracking state

    // Copy variable to be used within scoped mutex
    {
        std::unique_lock<std::mutex> lock(mMutex);
        state = mState;
        if (mState == Tracking::SYSTEM_NOT_READY)
            mState = Tracking::NO_IMAGES_YET;

        mIm.copyTo(im);

        if (mState == Tracking::NOT_INITIALIZED) {
            vIniKeys = mvIniKeys;
            vIniKeys = mvIniKeys;
            vMatches = mvIniMatches;
        } else if (mState == Tracking::OK) {
            vCurrentKeys = mvCurrentKeys;
            vMatchedMapPoints = mvpMatchedMapPoints;
        } else if (mState == Tracking::LOST) {
            vCurrentKeys = mvCurrentKeys;
        }
    }

    if (im.channels() < 3)
        cvtColor(im, im, CV_GRAY2BGR);

    // Draw
    if (state == Tracking::NOT_INITIALIZED) {
        for (unsigned int i = 0; i < vMatches.size(); i++) {
            if (vMatches[i] >= 0) {
                cv::line(im, vIniKeys[i].pt, vCurrentKeys[vMatches[i]].pt, cv::Scalar(0, 255, 0));
            }
        }
    } else if (state == Tracking::OK) // TRACKING
    {
        mnTracked = 0;
        const float r = 5;
        for (unsigned int i = 0; i < vMatchedMapPoints.size(); i++) {
            if (vMatchedMapPoints[i] || mvbOutliers[i]) {
                cv::Point2f pt1, pt2;
                pt1.x = vCurrentKeys[i].pt.x - r;
                pt1.y = vCurrentKeys[i].pt.y - r;
                pt2.x = vCurrentKeys[i].pt.x + r;
                pt2.y = vCurrentKeys[i].pt.y + r;
                if (!mvbOutliers[i]) {
                    cv::rectangle(im, pt1, pt2, cv::Scalar(0, 255, 0));
                    cv::circle(im, vCurrentKeys[i].pt, 2, cv::Scalar(0, 255, 0), -1);
                    mnTracked++;
                }
            }
        }
    }

    cv::Mat imWithInfo;
    DrawTextInfo(im, state, imWithInfo);

    return imWithInfo;
}

void FramePublisher::PublishFrame(const rclcpp::Time &stamp) {
    if (mIm.empty() || mImRight.empty()) {
        return;
    }

    cv::Mat im = DrawFrame();
    cv_bridge::CvImage rosImage, rosImageRight;
    rosImage.image = im.clone();
    rosImage.header.stamp = stamp;
    rosImage.encoding = "bgr8";

    rosImageRight.image = mImRight.clone();
    rosImageRight.header.stamp = stamp;
    rosImageRight.encoding = "mono8";

    mImagePub->publish(*rosImage.toImageMsg());
    mImagePubRight->publish(*rosImageRight.toImageMsg());
}

void FramePublisher::DrawTextInfo(cv::Mat &im, int nState, cv::Mat &imText) {
    stringstream s;
    if (nState == Tracking::NO_IMAGES_YET)
        s << "WAITING FOR IMAGES. (Topic: /camera/image_raw)";
    else if (nState == Tracking::NOT_INITIALIZED)
        s << " TRY TO INITIALING ";
    else if (nState == Tracking::OK) {
        s << " TRACKING ";
        int nKFs = mpMap->KeyFramesInMap();
        int nMPs = mpMap->MapPointsInMap();
        s << " - KFs: " << nKFs << " , MPs: " << nMPs << " , Tracked: " << mnTracked;
    } else if (nState == Tracking::LOST) {
        s << " TRACK LOST. TRYING TO RELOCALIZE ";
    } else if (nState == Tracking::SYSTEM_NOT_READY) {
        s << " LOADING ORB VOCABULARY. PLEASE WAIT...";
    }

    int baseline = 0;
    cv::Size textSize = cv::getTextSize(s.str(), cv::FONT_HERSHEY_PLAIN, 1, 1, &baseline);

    imText = cv::Mat(im.rows + textSize.height + 10, im.cols, im.type());
    im.copyTo(imText.rowRange(0, im.rows).colRange(0, im.cols));
    imText.rowRange(im.rows, imText.rows) = cv::Mat::zeros(textSize.height + 10, im.cols, im.type());
    cv::putText(imText, s.str(), cv::Point(5, imText.rows - 5), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255), 1,
                8);
}

void FramePublisher::Update(Tracking *pTracker) {
    std::unique_lock<std::mutex> lock(mMutex);
    pTracker->mImGray.copyTo(mIm);
    pTracker->mImGrayRight.copyTo(mImRight);
    mvCurrentKeys = pTracker->mCurrentFrame.mvKeys;
    mvpMatchedMapPoints = pTracker->mCurrentFrame.mvpMapPoints;
    mvbOutliers = pTracker->mCurrentFrame.mvbOutlier;
    mbOnlyTracking = pTracker->mbOnlyTracking;

    mState = static_cast<int>(pTracker->mLastProcessedState);

    mbUpdated = true;
}

} // namespace ORB_SLAM2
