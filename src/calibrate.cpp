#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <camera_calibration/Calibrate.h>

using namespace std;

class calibrate {

    private:
        vector<cv::Mat> images;
        ros::NodeHandle nh;
        ros::ServiceServer service;

    public:

        calibrate() {
            service = nh.advertiseService("calibrate", &calibrate::calib, this);
        }

        void callback(const sensor_msgs::ImageConstPtr& msg) {
            try {
                ROS_INFO("Image received");
        
                images.push_back(cv_bridge::toCvShare(msg, "bgr8")->image);

                if (images.size() == 20) {
                    vector<vector<cv::Point3f>> centers_allImages;
                    for (int i = 0; i < 20; i++) {
                        vector<cv::Point2f> centers;
                        cv::Mat image = images[i];
                        bool found = cv::findCirclesGrid(image, cv::Size(4,11), centers, cv::CALIB_CB_ASYMMETRIC_GRID);
                        // cv::imshow("view", image);
                        // cv::waitKey(30);
                        if(found) {
                            // cv::cornerSubPix(image, centers, cv::Size(11,11), cv::Size(-1,-1), cv::TermCriteria( cv::TermCriteria::EPS+cv::TermCriteria::COUNT, 30, 0.1 ));
                            cv::drawChessboardCorners(image, cv::Size(4,11), cv::Mat(centers), found);
                            cv::imshow("view", image);
                            cv::waitKey(30);
                        }
                    }
            
                    images.clear();
                }
            }
            catch (cv_bridge::Exception& e) {
                ROS_ERROR("Could not convert");
            }
        }  

        bool calib(camera_calibration::Calibrate::Request &req, camera_calibration::Calibrate::Response &res) {
            string mode = req.mode;
            if (mode == "calibrate") {
                image_transport::ImageTransport it(nh);
                image_transport::Subscriber sub = it.subscribe("image_raw", 20, &calibrate::callback, this);
            }
    
            return true;
        }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "image_listener");
    
    cv::namedWindow("view");
    cv::startWindowThread();

    ros::spin();
    cv::destroyWindow("view");

    return 0;
}

