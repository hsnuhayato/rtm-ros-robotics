// Author:: Atsushi TSUDA
// mail:: tsuda@jsk.t.u-tokyo.ac.jp

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <sensor_msgs/image_encodings.h>

namespace enc = sensor_msgs::image_encodings;

class HistogramTracking{
private:
  ros::NodeHandle nh_;

  int i, hist_size;
  double min_val, max_val;
  CvSize dst_size;
  CvPoint min_loc, max_loc;

  cv_bridge::CvImagePtr src;
  static IplImage *src_img;
  static IplImage *tmp_img;// tmp
  IplImage *dst_img;
  IplImage *src_hsv, *tmp_hsv;
  IplImage **src_planes, *tmp_planes[3];

  static CvRect tmp_area_;
  static bool is_getting_tmp;

  // ros topic subscriber/publiser initialize
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_subscriber_;
  image_transport::Publisher image_publisher_;
  sensor_msgs::Image published_image_;

  //static bool flag;
  std::string window_name;
  bool autosize;

  // parameter
  std::string template_filename;
  
public:
  HistogramTracking() : nh_("~"), it_(nh_){
    ros::NodeHandle pnh("~");
    pnh.param("autosize", autosize, true);
    pnh.param("window_name", window_name, std::string("histogram_tracking"));
    cvNamedWindow (window_name.c_str(), autosize ? CV_WINDOW_AUTOSIZE : 0);
    cvNamedWindow ("template_image", autosize ? CV_WINDOW_AUTOSIZE : 0);

    is_getting_tmp = false;

    cvSetMouseCallback(window_name.c_str(), &HistogramTracking::mouse_cb, this);

    pnh.param("template_filename", template_filename, std::string(window_name.c_str()));
    pnh.param("histogram_size", hist_size, 10);

    image_subscriber_ = it_.subscribe("image", 1, &HistogramTracking::image_cb, this);
    image_publisher_ = it_.advertise("out_image", 1);
    ROS_INFO("initialize finish");
  }

  ~HistogramTracking(){
    cvDestroyWindow (window_name.c_str());
    cvReleaseImage (&src_img);
    cvReleaseImage (&dst_img);
    cvReleaseImage (&tmp_img);
    cvReleaseImage (&src_hsv);
    cvReleaseImage (&tmp_hsv);
    for (i = 0; i < 3; i++) {
      cvReleaseImage (&src_planes[i]);
      cvReleaseImage (&tmp_planes[i]);
    }
    cvFree (&src_planes);
  }

  void image_cb(const sensor_msgs::ImageConstPtr &img){
    ROS_INFO("image call back is called");
    IplImage buf;
    src = cv_bridge::toCvCopy(img, enc::RGB8);
    cv::imshow (window_name, src->image);    
    buf = src->image;
    src_img = &buf;
    if (is_getting_tmp) {draw_area();}

    cvWaitKey(33);

    if ((tmp_img == NULL) || (is_getting_tmp)
        || (tmp_area_.width == 0) || (tmp_area_.height == 0)){return;}

    CvHistogram *hist = 0;

    float h_ranges[] = { 0, 181 };
    float *ranges[] = { h_ranges };
    src_planes = (IplImage **) cvAlloc (sizeof (IplImage *) * 3);
    for (i = 0; i < 3; i++) {
      src_planes[i] = cvCreateImage (cvGetSize (src_img), IPL_DEPTH_8U, 1);
      tmp_planes[i] = cvCreateImage (cvGetSize (tmp_img), IPL_DEPTH_8U, 1);
    }

    // (1)2つ入力画像（探索画像，テンプレート画像）の色空間を，RGBからHSVに変換
    src_hsv = cvCreateImage (cvGetSize (src_img), IPL_DEPTH_8U, 3);
    tmp_hsv = cvCreateImage (cvGetSize (tmp_img), IPL_DEPTH_8U, 3);
    cvCvtColor (src_img, src_hsv, CV_BGR2HSV);
    cvCvtColor (tmp_img, tmp_hsv, CV_BGR2HSV);
    cvCvtPixToPlane (src_hsv, src_planes[0], src_planes[1], src_planes[2], 0);
    cvCvtPixToPlane (tmp_hsv, tmp_planes[0], tmp_planes[1], tmp_planes[2], 0);
    // (2)テンプレート画像のヒストグラムを計算
    hist = cvCreateHist (1, &hist_size, CV_HIST_ARRAY, ranges, 1);
    cvCalcHist (&tmp_planes[0], hist, 0, 0);
    // (3)探索画像全体に対して，テンプレートのヒストグラムとの距離（手法に依存）を計算
    dst_size = cvSize (src_img->width - tmp_img->width + 1, src_img->height - tmp_img->height + 1);
    dst_img = cvCreateImage (dst_size, IPL_DEPTH_32F, 1);
    cvCalcBackProjectPatch (src_planes, dst_img, cvGetSize (tmp_img), hist, CV_COMP_CORREL, 1.0);
    cvMinMaxLoc (dst_img, &min_val, &max_val, &min_loc, &max_loc, NULL);

    // (4)テンプレートに対応する位置に矩形を描画
    cvRectangle (src_img, max_loc,
                 cvPoint (max_loc.x + tmp_img->width,
                          max_loc.y + tmp_img->height),
                 CV_RGB (255, 0, 0), 3);

    image_publisher_.publish(src->toImageMsg());
  }

  static void set_template(){
    ROS_INFO("set_template is called");
    cvSetImageROI(src_img, tmp_area_);
    cvReleaseImage(&tmp_img);
    tmp_img = cvCreateImage(cvGetSize(src_img), IPL_DEPTH_8U, 3);
    cvCopy(src_img, tmp_img);
    cvResetImageROI(src_img);
    cvShowImage("template_image", tmp_img);
    cvWaitKey(1);
  }

  void draw_area (){
    ROS_INFO("draw_area is called");
    cvRectangle(src_img,
                cvPoint(tmp_area_.x, tmp_area_.y),
                cvPoint(tmp_area_.x+tmp_area_.width, tmp_area_.y+tmp_area_.height),
                cvScalar(0xff, 0x00, 0x00));
  }

  static void mouse_cb(int event, int x, int y, int flags, void *param){
    ROS_INFO("mouse call back is called");
    switch (event){
    case CV_EVENT_MOUSEMOVE:{
      if(is_getting_tmp){
        tmp_area_.width = x - tmp_area_.x;
        tmp_area_.height = y - tmp_area_.y;
      }
      break;
    }
    case CV_EVENT_LBUTTONDOWN:{
      is_getting_tmp = true;
      tmp_area_ = cvRect(x, y, 0, 0);
      break;
    }
    case CV_EVENT_LBUTTONUP:{
      is_getting_tmp = false;
      if (tmp_area_.width < 0){
        tmp_area_.x += tmp_area_.width;
        tmp_area_.width *= -1;
      }
      if (tmp_area_.height < 0){
        tmp_area_.y += tmp_area_.height;
        tmp_area_.height *= -1;
      }
        
      set_template();
      break;
    }
    }    
  } 
};

CvRect HistogramTracking::tmp_area_ = cvRect(0,0,0,0);
IplImage* HistogramTracking::src_img;
IplImage* HistogramTracking::tmp_img;
bool HistogramTracking::is_getting_tmp;

int main(int argc, char** argv){
  ros::init(argc, argv, "histogram_tracking");
  HistogramTracking ht;
  ros::spin();
  
  return 0;
}
