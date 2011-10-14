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

void getLabDistribution(Mat& _src,char *filename)
{
	IplImage src = _src;
  IplImage *lab = cvCreateImage(cvGetSize(&src),IPL_DEPTH_8U,3);
  cvCvtColor(&src, lab, CV_RGB2Lab);

	FILE *fp = fopen(filename,"w");

	float avel=0.0, avea=0.0, aveb=0.0, count=0.0;
	vector<CPF> pixels;
	for(int i=0,k=0; i<lab->height; i++)
		for(int j=0; j<lab->width;j++,k++)
			if(!checkPixelColor(j,i,_src,0,0,0))
				{
					CPF p = {(float)((uchar)lab->imageData[3*k  ]),
									 (float)((uchar)lab->imageData[3*k+1]),
									 (float)((uchar)lab->imageData[3*k+2])};
					fprintf(fp,"%f %f %f\n",p.x,p.y,p.z);
					pixels.push_back(p);
					avel += p.x;
					avea += p.y;
					aveb += p.z;
					count += 1.0;
				}
	fclose(fp);

	avel /= count;
	avea /= count;
	aveb /= count;
	float varl=0.0, vara=0.0, varb=0.0;
	vector<CPF>::iterator it = pixels.begin();
	for(; it != pixels.end(); it++)
		{
			varl += pow(it->x - avel, 2);
			vara += pow(it->y - avea, 2);
			varb += pow(it->z - aveb, 2);
		}
	varl /= count;
	vara /= count;
	varb /= count;
	
	// Mat img(lab);
	// imshow("hoge",img);
	// waitKey();

	printf("%f %f %f %f %f %f\n",avel,avea,aveb,sqrt(varl),sqrt(vara),sqrt(varb));

	cvReleaseImage(&lab);
}

int main(int argc, char** argv) 
{
  if(argc < 2)
    {
      std::cerr << "usage: ./lab trainingimage colorlog" << std::endl;
      exit(1);
    }

  Mat src_mask = imread(argv[1]);
	
	getLabDistribution(src_mask,argv[2]);
  
  return 1;
}

