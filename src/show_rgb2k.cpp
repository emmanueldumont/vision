#include <iostream>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/highgui/highgui.hpp>

static const uint32_t MY_ROS_QUEUE_SIZE = 1000;

void imgcb1(const sensor_msgs::Image::ConstPtr& msg)
{

    try {
        cv_bridge::CvImageConstPtr cv_ptr;
        cv_ptr = cv_bridge::toCvShare(msg);

        cv::imshow("foo", cv_ptr->image);
        cv::waitKey(1);  // Update screen
    } catch (const cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}

void imgcb2(const sensor_msgs::Image::ConstPtr& msg)
{

    try {
        cv_bridge::CvImageConstPtr cv_ptr;
        cv_ptr = cv_bridge::toCvShare(msg);

        cv::imshow("bar", cv_ptr->image);
        cv::waitKey(1);  // Update screen
    } catch (const cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "foo");

    std::cout << "Oh hai there!" << std::endl;

    ros::NodeHandle nh;
    // ros::Subscriber sub = nh.subscribe("camera/rgb/image_raw", MY_ROS_QUEUE_SIZE, imgcb);
    //ros::Subscriber sub = nh.subscribe("camera/rgb/image_color", MY_ROS_QUEUE_SIZE, imgcb);
    ros::Subscriber sub1 = nh.subscribe("kinect1/rgb/image_color", MY_ROS_QUEUE_SIZE, imgcb1);
    ros::Subscriber sub2 = nh.subscribe("kinect2/rgb/image_color", MY_ROS_QUEUE_SIZE, imgcb2);

    cv::namedWindow("foo");
    cv::namedWindow("bar");
    ros::spin();
    cv::destroyWindow("foo");
    cv::destroyWindow("bar");

    std::cout << "byebye my friend" << std::endl;

    return 0;
}

