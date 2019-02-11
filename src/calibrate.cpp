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
        vector<vector<cv::Point2f>> all_centers;
        ros::NodeHandle nh;
        image_transport::Subscriber sub;

    public:

        calibrate(ros::NodeHandle &n) {
            nh = n;
        }

        void callback(const sensor_msgs::ImageConstPtr& msg) {
            try {
                ROS_INFO("Image received");
                vector<cv::Point2f> centers;
                cv::Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;
                bool found = cv::findCirclesGrid(image, cv::Size(4,11), centers, cv::CALIB_CB_ASYMMETRIC_GRID);
                // ROS_INFO_STREAM(found);
                // cv::cornerSubPix(image, centers, cv::Size(11,11), cv::Size(-1,-1), cv::TermCriteria(cv::TermCriteria::EPS+cv::TermCriteria::COUNT, 30, 0.1 ));
                cv::drawChessboardCorners(image, cv::Size(4,11), cv::Mat(centers), found);
                cv::imshow("view", image);
                cv::waitKey(30);

                all_centers.push_back(centers);
                
                if (all_centers.size() == 20) {
                    conduct_calibration();     
                }
            } catch (cv_bridge::Exception& e) {
                ROS_ERROR("Could not convert");
            }
        }

        void conduct_calibration() {
            ROS_INFO("Beginning calibration");

            create_obj_pts();
            
        }

        bool calib(camera_calibration::Calibrate::Request &req, camera_calibration::Calibrate::Response &res) {
            string mode = req.mode;
            ROS_INFO_STREAM(mode);
            if (mode == "calibrate") {
                ROS_INFO("Creating listener");
                image_transport::ImageTransport it(nh);
                sub = it.subscribe("image_raw", 20, &calibrate::callback, this);
                ROS_INFO("Created");
            }
            
            return true;
        }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "calibration");
    
//    cv::namedWindow("view");
//    cv::startWindowThread();

    ros::NodeHandle nh;
    
    // ros::ServiceServer test = nh.advertiseService("cal", add);
    calibrate cal_obj(nh);
    ros::ServiceServer ss = nh.advertiseService("cal", &calibrate::calib, &cal_obj);    

    ros::spin();
//    cv::destroyWindow("view");

    return 0;
}

