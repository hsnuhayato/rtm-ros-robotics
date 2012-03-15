//
//Written by Masaho Ishida (ishida@jsk.t.u-tokyo.ac.jp) 2011
//

#include <ros/ros.h>
#include <rospack/rospack.h>
#include <opencv/highgui.h>
#include <cv_bridge/CvBridge.h>
#include <sensor_msgs/Image.h>

class magnitude_converter
{
private:
    ros::NodeHandle _n;
    ros::Subscriber _sub;

public:
    magnitude_converter(){
        _sub = _n . subscribe("image", 1, &magnitude_converter::image_cb, this);
        cv::namedWindow("SrcImage", 0);
        cv::namedWindow("Histogram", 0);
    }

    virtual ~magnitude_converter(){
        _sub . shutdown();
    }


    void image_cb (const sensor_msgs::Image& msg){
        sensor_msgs::CvBridge bridge;
        bridge . fromImage (msg, "bgr8");
        IplImage* src_imgipl;
        src_imgipl = bridge . toIpl();
        cv::Mat src_img(src_imgipl);

        const int ch_width = 260;
        const int sch = src_img . channels();
        cv::Mat hist_img(cv::Size(ch_width * sch, 200), CV_8UC3, cv::Scalar::all(255));

        std::vector<cv::MatND> hist(3);
        const int hist_size = 256;
        const int hdims[] = {hist_size};
        const float hranges[] = {0,256};
        const float* ranges[] = {hranges};
        double max_val = .0;

        if(sch==1) {

            cv::calcHist(&src_img, 1, 0, cv::Mat(), hist[0], 1, hdims, ranges, true, false);
            cv::minMaxLoc(hist[0], 0, &max_val);
        } else {

            for(int i=0; i<sch; ++i) {
                cv::calcHist(&src_img, 1, &i, cv::Mat(), hist[i], 1, hdims, ranges, true, false);
                double tmp_val;
                cv::minMaxLoc(hist[i], 0, &tmp_val);
                max_val = max_val < tmp_val ? tmp_val : max_val;
            }
        }

        cv::Scalar color = cv::Scalar::all(100);
        for(int i=0; i<sch; i++) {
            if(sch==3)
                color = cv::Scalar((0xaa<<i*8)&0x0000ff,(0xaa<<i*8)&0x00ff00,(0xaa<<i*8)&0xff0000, 0);
            hist[i] . convertTo(hist[i], hist[i] . type(), max_val ? 200./max_val : 0.,0);
            //      hist[i].convertTo(hist[i], hist[i].type(), 1,0);
            for(int j=0; j<hist_size; ++j) {
                int bin_w = cv::saturate_cast<int>((double)ch_width/hist_size);
                cv::rectangle(hist_img,
                              cv::Point(j*bin_w+(i*ch_width), hist_img . rows),
                              cv::Point((j+1)*bin_w+(i*ch_width), hist_img . rows-cv::saturate_cast<int>(hist[i] . at<float>(j))),
                              color, -1);
            }
        }

        cv::imshow("SrcImage", src_img);
        cv::imshow("Histogram", hist_img);
        cv::waitKey(10);
    }
};

int main (int argc, char **argv){
    ros::init (argc, argv, "magnitude_converter");

    magnitude_converter mc;

    ros::spin();

    return 0;
}
