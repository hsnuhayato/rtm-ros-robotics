#include "fitting.h"
#include <ros/node_handle.h>
#include <opencv_fitting/Point2DList.h>

class DetectEllipseNodeImpl : public DetectEllipseNode
{
    ros::NodeHandle _nh;
    ros::Publisher _pubposes2d;
    double ellipse_radius;
public:
    DetectEllipseNodeImpl() {
        // topic/service/publisher subscription
    }

    virtual void imagecb(const sensor_msgs::ImageConstPtr& msg) {
//        opencv_fitting::Point2DList mylist;
//        DetectEllipses(msg,mylist);
//        _pubposes2d.publish(mylist);
//        
//        sensor_msgs::PointCloud points;
//        DetectCircles(msg,points);
//        _pubcircles.publish(points);
    }

    virtual void DetectEllipses(const sensor_msgs::ImageConstPtr& msg, opencv_fitting::Poses2D& ellipses)
    {
        // 実装してください
    }

    virtual void DetectCircles(const sensor_msgs::ImageConstPtr& msg, sensor_msgs::PointCloud& points)
    {
        // 実装してください
    }
};

double fitEllipse( const cv::Mat& points, cv::RotatedRect& ellipse )
{
    // ここで実装してください
    return 0;
}

void fitEllipseFromImage(cv::Mat& image, std::list<cv::RotatedRect>& ellipses)
{
    // 画像からpointsを初期化する
}

DetectEllipseNodePtr CreateDetectEllipseNode()
{
    return DetectEllipseNodePtr(new DetectEllipseNodeImpl());
}
