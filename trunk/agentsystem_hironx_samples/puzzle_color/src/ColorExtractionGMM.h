#pragma once

#include "cv.h"
#include "ml.h"
using namespace cv;

#include <vector>
using namespace std;

class ColorExtractionGMM
{
 public:
  ColorExtractionGMM(void);
  ~ColorExtractionGMM(void);

  void setUseLab(bool flag){ UseLab = flag; };
  void setGaussN(int n) { GaussN = n; };
  void proc(Mat& source, Mat& source_mask, Mat& target);

 private:
  bool UseLab;
  int GaussN;

  void TrainGMM(CvEM& source_model, Mat& train) ;
  void makeComparedData(const CvEM model, vector<Mat> *means, vector<Mat> *icovs);
};
