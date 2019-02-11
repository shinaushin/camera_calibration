#include <ros/ros.h>
#include <camera_calibration/Calibrate.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "calibrate_client");
    ros::NodeHandle nh;
    ros::service::waitForService("cal");
    ros::ServiceClient client = nh.serviceClient<camera_calibration::Calibrate>("cal");

    camera_calibration::Calibrate srv;
    srv.request.mode = "calibrate";
    if (client.call(srv)) {
        ROS_INFO("Called service");
    } else {
        ROS_ERROR("Failed to call service");
        return 1;
    }

    return 0;
}

