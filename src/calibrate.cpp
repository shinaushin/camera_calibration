#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>

using namespace std;

void callback(const sensor_msgs::ImageConstPtr& msg) {
    try {
        ROS_INFO("Image received");
        cv::Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;
        vector<cv::Point2f> centers;
        bool found = cv::findCirclesGrid(image, cv::Size(4,11), centers, cv::CALIB_CB_ASYMMETRIC_GRID);
        ROS_INFO_STREAM(found);
        // cv::imshow("view", image);
        // cv::waitKey(30);
        if (found) {
            // cv::cornerSubPix(image, centers, cv::Size(11,11), cv::Size(-1,-1), cv::TermCriteria( cv::TermCriteria::EPS+cv::TermCriteria::COUNT, 30, 0.1 ));
            cv::drawChessboardCorners(image, cv::Size(4,11), cv::Mat(centers), found);
            cv::imshow("view", image);
            cv::waitKey(30);
        }
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("Could not convert");
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "image_listener");
    ros::NodeHandle nh;
    cv::namedWindow("view");
    cv::startWindowThread();
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("image_raw", 20, callback);

    ros::spin();
    cv::destroyWindow("view");
}

