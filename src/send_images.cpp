#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

using namespace std;

int main(int argc, char** argv) {
    ros::init(argc, argv, "image_publisher");
    ros::NodeHandle nh;

    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("image_raw", 20);
    
    ros::Rate rate(5);
    int i = 0;
    int im_num = 56;
    cv::Mat image;
    sensor_msgs::ImagePtr msg;
    while(nh.ok() && i < 20) {
        string num = to_string(im_num+i);
        image = cv::imread("../data/IMG_17" + num + ".jpg", CV_LOAD_IMAGE_COLOR);
        cv::waitKey(0);
        msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
        pub.publish(msg);
        ROS_INFO("Image published.");
        i++;
        ros::spinOnce();
        rate.sleep();
    }
}
