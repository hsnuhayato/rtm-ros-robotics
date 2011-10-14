#include <stdio.h>
#include <iostream>
#include <limits>
#include "highgui.h"
#include "ColorExtractionGMM.h"

int main(int argc, char** argv) 
{
  if(argc < 2)
    {
      std::cerr << "usage: ./skin trainingimage inputimage" << std::endl;
      exit(1);
    }

  ColorExtractionGMM ce;
	//ce.setUseLab(true);
	ce.setGaussN(1);
	
  Mat src = imread(argv[2]);
  Mat src_mask = imread(argv[1]);
  Mat dst = src.clone();
  
  // float mat[3][3]={{1,0,0},{0,1,0},{0,0,1}};
  // Mat hoge(3,3,CV_32F,mat);
  // Mat hinv = hoge.inv();
  // for(int ii=0; ii<3; ii++)
  //   {
  //     for(int jj=0; jj<3; jj++)
  // 	printf("%f ",hinv.at<float>(ii,jj));
  //     printf("\n");
  //   }

  ce.proc(src,src_mask,dst);
  waitKey(0);

  return 1;
}
