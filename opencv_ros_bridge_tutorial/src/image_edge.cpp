// this code is based on http://www.ros.org/wiki/cv_bridge/Tutorials/UsingCvBridgeToConvertBetweenROSImagesAndOpenCVImages

// Written by T.Kurobobi, modified by M.Saito

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace enc = sensor_msgs::image_encodings;

static const char WINDOW_IM[] = "Image window";
static const char WINDOW_SOB[] = "Sobel window";
static const char WINDOW_LAP[] = "Laplacian window";
static const char WINDOW_CAN[] = "Canny window";

class ImageEdge
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

public:
  ImageEdge()
    : it_(nh_)
  {
    image_pub_ = it_.advertise("image_edge", 1);
    image_sub_ = it_.subscribe("image", 1, &ImageEdge::imageCb, this);

    cv::namedWindow(WINDOW_IM);
    cv::namedWindow(WINDOW_SOB);
    cv::namedWindow(WINDOW_LAP);
    cv::namedWindow(WINDOW_CAN);
  }

  ~ImageEdge()
  {
    cv::destroyWindow(WINDOW_IM);
    cv::destroyWindow(WINDOW_SOB);
    cv::destroyWindow(WINDOW_LAP);
    cv::destroyWindow(WINDOW_CAN);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    cv_bridge::CvImagePtr cv_ptr_mono;
    cv_bridge::CvImagePtr cv_ptr_tmp;
    cv_bridge::CvImagePtr cv_ptr_sobel;
    cv_bridge::CvImagePtr cv_ptr_lap;
    cv_bridge::CvImagePtr cv_ptr_can;

    try
      {
	cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
	cv_ptr_mono = cvtColor(cv_ptr, "mono8");
	cv_ptr_tmp = cv_bridge::toCvCopy(msg, enc::BGR8);
	cv_ptr_sobel = cv_bridge::toCvCopy(msg, enc::BGR8);
	cv_ptr_lap = cv_bridge::toCvCopy(msg, enc::BGR8);
	cv_ptr_can = cvtColor(cv_ptr, "mono8");
      }
    catch (cv_bridge::Exception& e)
      {
	ROS_ERROR("cv_bridge exception: %s", e.what());
	return;
      }

    // Three type edge filter
    cv::Sobel(cv_ptr->image, cv_ptr_tmp->image, CV_32F, 1, 1);
    cv::convertScaleAbs(cv_ptr_tmp->image, cv_ptr_sobel->image, 1, 0);

    cv::Laplacian(cv_ptr->image, cv_ptr_tmp->image, CV_32F, 3);
    cv::convertScaleAbs(cv_ptr_tmp->image, cv_ptr_lap->image, 1, 0);

    cv::Canny(cv_ptr_mono->image, cv_ptr_can->image, 50, 200);

    // draw circle in original image
    //  if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
    //    cv::circle(cv_ptr->image, cv::Point(30, 50), 10, CV_RGB(0,0,255));
    //  cv::threshold(cv_ptr->image, cv_ptr->image, 128, 255, 0);

    cv::imshow(WINDOW_IM, cv_ptr->image);
    cv::imshow(WINDOW_SOB, cv_ptr_sobel->image);
    cv::imshow(WINDOW_LAP, cv_ptr_lap->image);
    cv::imshow(WINDOW_CAN, cv_ptr_can->image);
    cv::waitKey(3);

    image_pub_.publish(cv_ptr->toImageMsg());
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_edge");
  ImageEdge ic;
  ros::spin();
  return 0;
}
