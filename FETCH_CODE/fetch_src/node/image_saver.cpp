#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include "sensor_msgs/Image.h"
#include "opencv2/opencv.hpp"

void callback(const sensor_msgs::Image::ConstPtr &msg)
{
  cv_bridge::CvImageConstPtr depth = cv_bridge::toCvCopy(msg);

  cv::Mat depth_img = depth->image;

  cv::imshow("Depth", depth_img);
  
  // ROS_INFO("Hello\n");
  // cv_bridge::CvImageConstPtr cv_ptr;
  // try {
  //   cv_bridge::CvtColorForDisplayOptions options;
  //   options.min_image_value = 0;
  //   options.max_image_value = 10;  // 10 [m]
    
  //   cv_ptr = cv_bridge::cvtColorForDisplay(cv_bridge::toCvShare(msg), "", options);
  // } catch (cv_bridge::Exception& e) {
  //   ROS_ERROR_THROTTLE(30, "Unable to convert '%s' image for display: '%s'",
  //                      msg->encoding.c_str(), e.what());
  // }
//   if (g_gui && !g_last_image.empty()) {
//     const cv::Mat &image = g_last_image;
//     cv::imshow(g_window_name, image);
//     cv::waitKey(3);
//   }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "listener");

    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("/head_camera/depth_registered/image_raw", 1, callback);

    ros::spin();

    return 0;
}
