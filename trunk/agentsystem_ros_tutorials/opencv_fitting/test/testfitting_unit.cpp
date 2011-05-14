// 単体テスト
#include <gtest/gtest.h>
#include "fitting.h"

TEST(Fitting,Ellipse)
{
    cv::Mat points;
    cv::RotatedRect input, output;
    double noise = 0;
    double pixelerror = 0.5;
    // pointsのサンプリング
    double fiterror = fitEllipse(points, output);
    ASSERT_NEAR(input.center.x,output.center.x,pixelerror);
    // ..
    EXPECT_NEAR(input.angle,output.angle,pixelerror);
    ASSERT_TRUE(fiterror <= noise*1.5);
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
