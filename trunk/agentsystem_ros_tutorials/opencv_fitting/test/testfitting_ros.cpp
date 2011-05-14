// 結合テスト
#include "fitting.h"
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <geometry_msgs/Pose2D.h>
#include <boost/shared_ptr.hpp>
#include <cv_bridge/cv_bridge.h>

class TestImageResults : public testing::Test
{
protected:
    boost::shared_ptr<ros::NodeHandle> nh_;
    ros::Subscriber posesub_;
    geometry_msgs::Pose2D realpose_;
    bool receivedpose_;

    virtual void SetUp() {
        receivedpose_ = false;
        nh_.reset(new ros::NodeHandle());
        posesub_ = nh_->subscribe("ellipse",1,&TestImageResults::posecb, this);
    }
    virtual void TearDown() {
        posesub_.shutdown();
        nh_.reset();
    }
    void posecb(const geometry_msgs::Pose2DConstPtr msg) {
        ASSERT_NEAR(msg->x,realpose_.x,0.5);
        ASSERT_NEAR(msg->y,realpose_.y,0.5);
        EXPECT_NEAR(msg->theta, realpose_.theta,0.2);
        receivedpose_ = true;
    }
};

TEST_F(TestImageResults, SendImage) { 
    image_transport::ImageTransport imgtrans(*nh_);
    image_transport::Publisher image_pub = imgtrans.advertise("image", 1);
    realpose_.x = 100;
    realpose_.y = 200;
    realpose_.theta = 0;
    receivedpose_ = false;
    cv_bridge::CvImagePtr cv_ptr;
    // realpose_で楕円を表示した画像を作成する（cv::Ellipse）
    image_pub.publish(cv_ptr->toImageMsg()); // 画像を送信し
    // 結果を待つ
    for(int i = 0; i < 1000; ++i) {
        if( receivedpose_ ) {
            break;
        }
        ros::spinOnce();
        usleep(1000); // １ミリ秒で待つ
    }
    ASSERT_TRUE(receivedpose_);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "testfitting");
  if( !ros::master::check() ) {
      return 1;
  }
  return RUN_ALL_TESTS();
}
