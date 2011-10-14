#include <iostream>
#include <cv.h>
#include <highgui.h>

using namespace cv;

typedef CvPoint3D32f CPF;

bool checkBounds(int w,int h,int u,int v)
{ return ((u >= 0 && v >= 0 && u < w && v < h) ? true : false); };

bool checkPixelColor(int u,int v,Mat& img,int r,int g,int b)
{
	return ((img.at<uchar>(v,3*u  ) == r && 
					 img.at<uchar>(v,3*u+1) == g &&
					 img.at<uchar>(v,3*u+2) == b)
					? true : false);
}

void setPixelColor(int n,IplImage *img,int r,int g,int b)
{
	img->imageData[3*n  ] = r;
	img->imageData[3*n+1] = g;
	img->imageData[3*n+2] = b;
}

void proc(string mode,float *ave,float *var,Mat& _src)
{
	IplImage src = _src;
	IplImage *dst;
	if(mode == "rgb")
		dst = cvCloneImage(&src);
	else if(mode == "lab")
		{
			dst = cvCreateImage(cvGetSize(&src),IPL_DEPTH_8U,3);
			cvCvtColor(&src, dst, CV_RGB2Lab);
		}
	else
		{
			cerr << "no such mode" << endl;
			exit(1);
		}
	
	imshow("src",_src);
	//imshow("src",dst);
	cvMoveWindow("src",0,0);

	for(int i=0,k=0; i<dst->height; i++)
		for(int j=0; j<dst->width; j++,k++)
			{
				float r = (float)((uchar)dst->imageData[3*k  ]);
				float g = (float)((uchar)dst->imageData[3*k+1]);
				float b = (float)((uchar)dst->imageData[3*k+2]);
				if(fabs(r - ave[0]) > 2*var[0] ||
					 fabs(g - ave[1]) > 2*var[1] ||
					 fabs(b - ave[2]) > 2*var[2])
					setPixelColor(k,dst,0,0,0);
			}

	Mat img(dst);
	imshow("dst",img);
	cvMoveWindow("dst",640,0);
	waitKey();

	cvReleaseImage(&dst);
}

int main(int argc, char** argv) 
{
  if(argc < 9)
    {
      std::cerr << "usage: ./color inputimage mode ave1 ave2 ave3 var1 var2 var3" << std::endl;
      exit(1);
    }
	string mode = string(argv[2]);
	float ave[3] = {atof(argv[3]), atof(argv[4]), atof(argv[5])};
	float var[3] = {atof(argv[6]), atof(argv[7]), atof(argv[8])};
  Mat src = imread(argv[1]);

	proc(mode,ave,var,src);
  
  return 1;
}

