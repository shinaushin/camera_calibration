#include <ros/ros.h>
#include <ros/package.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <camera_calibration/Calibrate.h>

class calibrate {

    private:
        std::vector<std::vector<cv::Point2f>> all_centers; // vector of vector of all centers found in images
        ros::NodeHandle nh;
        image_transport::Subscriber sub; // subscribe to topic where images are sent
        cv::Size s = cv::Size(4,11); //size of circle grid
        cv::Size pic_size = cv::Size(640,480); //size of images that are sent

    public:

        calibrate(ros::NodeHandle &n) {
            nh = n;
        }

        // subscriber callback
        void callback(const sensor_msgs::ImageConstPtr& msg) {
            try {
                // ROS_INFO("Image received");
                std::vector<cv::Point2f> centers;
                cv::Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;
                bool found = cv::findCirclesGrid(image, s, centers, cv::CALIB_CB_ASYMMETRIC_GRID);
                // ROS_INFO_STREAM(found);
                
                cv::Mat gray;
                cv::cvtColor(image, gray, cv::COLOR_RGB2GRAY);
                cv::cornerSubPix(gray, centers, cv::Size(11,11), cv::Size(-1,-1), cv::TermCriteria(cv::TermCriteria::EPS+cv::TermCriteria::COUNT, 30, 0.1 ));
                
                // cv::drawChessboardCorners(image, s, cv::Mat(centers), found);
                // cv::imshow("view", image);
                // cv::waitKey(30);

                all_centers.push_back(centers);
                
                // after we process all 20 images, begin calibration
                if (all_centers.size() == 20) {
                    cv::Mat camMat = cv::Mat::eye(3,3, CV_64F);
                    cv::Mat distCoeffs = cv::Mat::zeros(8,1,CV_64F);
                    std::vector<cv::Mat> rvecs;
                    std::vector<cv::Mat> tvecs;
                    bool success = conduct_calibration(camMat, distCoeffs, rvecs, tvecs);   
                    if (success) {
                        std::string path = ros::package::getPath("camera_calibration");
                        ROS_INFO_STREAM(path);
                        std::string file = path + "/intrinsics.yml";
                        cv::FileStorage fs(file, cv::FileStorage::WRITE);
                        fs << "Camera Matrix" << camMat;
                        fs << "Distortion Coefficients" << distCoeffs;
                        fs.release();
                        ROS_INFO("Finished writing to YAML file");
                    }  
                }
            } catch (cv_bridge::Exception& e) {
                ROS_ERROR("Could not convert");
            }
        }

        // set up all params to call calibratecamera function
        bool conduct_calibration(cv::Mat& camMat, cv::Mat& distCoeffs, std::vector<cv::Mat>& rvecs, std::vector<cv::Mat>& tvecs) {
            ROS_INFO("Beginning calibration");
           
            std::vector<std::vector<cv::Point3f>> obj_pts(1);
            create_obj_pts(obj_pts[0]);
            // for (int i = 0; i < all_centers[0].size(); i++) {
                // ROS_INFO_STREAM(all_centers[0][i]);
            // }
            obj_pts.resize(all_centers.size(), obj_pts[0]);

            double rms = calibrateCamera(obj_pts, all_centers, pic_size, camMat, distCoeffs, rvecs, tvecs);
            ROS_INFO_STREAM(rms);

            if (rms < 1.0) {
                ROS_INFO("Calibration complete");
                return true;
            } else {
                return false;
            } 
        }

        // creates points of where circles may be in 3d space
        void create_obj_pts(std::vector<cv::Point3f>& corners) {
            for (int i = s.height; i > 0; i--) {
                for (int j = s.width; j > 0; j--) {
                    corners.push_back(cv::Point3f( (2*(j-1) + (s.height-i+1)%2), i-1, 0)); 
                }
            }
        }

        // service callback
        bool calib(camera_calibration::Calibrate::Request &req, camera_calibration::Calibrate::Response &res) {
            std::string mode = req.mode;
            ROS_INFO_STREAM(mode);
            if (mode == "calibrate") {
                // begins listening for images
                image_transport::ImageTransport it(nh);
                sub = it.subscribe("image_raw", 20, &calibrate::callback, this);
            }
            
            return true;
        }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "calibration");
    ros::NodeHandle nh;
 
    // initialize service   
    calibrate cal_obj(nh);
    ros::ServiceServer ss = nh.advertiseService("cal", &calibrate::calib, &cal_obj);    

    ros::spin();

    return 0;
}

