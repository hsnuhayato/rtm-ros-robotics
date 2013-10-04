// -*- coding: utf-8 -*-
#include <opencv2/imgproc/imgproc.hpp>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Image.h>

#include <list>
#include <boost/shared_ptr.hpp>

#include <opencv_fitting/Poses2D.h>

/// \brief ROSノード
class DetectEllipseNode
{
 public:
    virtual ~DetectEllipseNode() {}
    
    virtual void DetectEllipses(const sensor_msgs::ImageConstPtr& msg,
                                opencv_fitting::Poses2D& ellipses) = 0;
    virtual void DetectCircles(const sensor_msgs::ImageConstPtr& msg,
                               sensor_msgs::PointCloud& points) = 0;
};

typedef boost::shared_ptr<DetectEllipseNode> DetectEllipseNodePtr;

/// \brief 楕円を近似する
double fitEllipse( const cv::Mat& points, cv::RotatedRect& ellipse );

/// \brief 楕円を画像から抽出する
void fitEllipseFromImage(cv::Mat& image, std::list<cv::RotatedRect>& ellipses);

/// \brief DetectEllipseNodeの実装クラスを作成する
DetectEllipseNodePtr CreateDetectEllipseNode();
