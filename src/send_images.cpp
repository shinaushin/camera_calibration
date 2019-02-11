#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <ros/package.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "image_publisher");
    ros::NodeHandle nh;

    // image transport to send images to topic
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("image_raw", 20);
    
    // sleep until there's a subscriber so that we don't lose any messages
    ros::Rate rate(5);
    while(pub.getNumSubscribers() == 0) {
        rate.sleep();
    }

    int i = 0;
    int im_num = 56;
    cv::Mat image;
    sensor_msgs::ImagePtr msg;
    float resize_factor = 0.16; // resize image to 640 x 480
   
    // send 20 images from data folder
    while(nh.ok() && i < 20) {
        std::string num = std::to_string(im_num+i);
        std::string path = ros::package::getPath("camera_calibration") + "/data/IMG_17"+num+".JPG";
        // ROS_INFO_STREAM(path);
        image = cv::imread(path, CV_LOAD_IMAGE_COLOR);
        cv::resize(image, image, cv::Size(), resize_factor, resize_factor);
        msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
        pub.publish(msg);
        ROS_INFO("Image published.");
        i++;
        ros::spinOnce();
        rate.sleep();
    }
}
