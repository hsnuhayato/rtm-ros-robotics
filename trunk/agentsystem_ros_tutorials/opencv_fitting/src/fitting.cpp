#include "fitting.h"
#include <ros/node_handle.h>

class DetectEllipseNodeImpl : public DetectEllipseNode
{
public:
    virtual void DetectEllipses(const sensor_msgs::ImageConstPtr& msg, std::list<cv::RotatedRect>& ellipses)
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
