// this code is based on http://www.ros.org/wiki/cv_bridge/Tutorials/UsingCvBridgeToConvertBetweenROSImagesAndOpenCVImages

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>

namespace enc = sensor_msgs::image_encodings;

class ImageFlip
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

public:
  ImageFlip()
    : it_(nh_)
  {
    image_pub_ = it_.advertise("image_rotated", 1);
    image_sub_ = it_.subscribe("image", 1, &ImageFlip::imageCb, this);
  }

  ~ImageFlip()
  {
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
      {
	cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
      }
    catch (cv_bridge::Exception& e)
      {
	ROS_ERROR("cv_bridge exception: %s", e.what());
	return;
      }
    cv::Mat out = cv_ptr->image.clone();

    //// opencv functions
    ////
    cv::flip(cv_ptr->image, out, 0);
    ////
    ////

    // ros publisher
    // http://answers.ros.org/question/733/how-to-convert-cvmat-to-sensor_msgsimageptr
    cv_bridge::CvImage out_msg;
    out_msg.header   = msg->header; // Same timestamp and tf frame as input image
    out_msg.encoding = sensor_msgs::image_encodings::RGB8; // Or whatever
    out_msg.image    = out; // Your cv::Mat

    image_pub_.publish(out_msg.toImageMsg());
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageFlip ic;
  ros::spin();
  return 0;
}
