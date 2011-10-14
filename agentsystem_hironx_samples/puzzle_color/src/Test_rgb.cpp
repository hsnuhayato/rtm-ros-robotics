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
  IplImage *rgb = cvCloneImage(&src);

	FILE *fp = fopen(filename,"w");

	float aver=0.0, aveg=0.0, aveb=0.0, count=0.0;
	vector<CPF> pixels;
	for(int i=0,k=0; i<rgb->height; i++)
		for(int j=0; j<rgb->width;j++,k++)
			if(!checkPixelColor(j,i,_src,0,0,0))
				{
					CPF p = {(float)((uchar)rgb->imageData[3*k  ]),
									 (float)((uchar)rgb->imageData[3*k+1]),
									 (float)((uchar)rgb->imageData[3*k+2])};
					fprintf(fp,"%f %f %f\n",p.x,p.y,p.z);
					pixels.push_back(p);
					aver += p.x;
					aveg += p.y;
					aveb += p.z;
					count += 1.0;
				}
	fclose(fp);

	aver /= count;
	aveg /= count;
	aveb /= count;
	float varr=0.0, varg=0.0, varb=0.0;
	vector<CPF>::iterator it = pixels.begin();
	for(; it != pixels.end(); it++)
		{
			varr += pow(it->x - aver, 2);
			varg += pow(it->y - aveg, 2);
			varb += pow(it->z - aveb, 2);
		}
	varr /= count;
	varg /= count;
	varb /= count;
	
	// Mat img(lab);
	// imshow("hoge",img);
	// waitKey();

	printf("%f %f %f %f %f %f\n",aver,aveg,aveb,sqrt(varr),sqrt(varg),sqrt(varb));

	cvReleaseImage(&rgb);
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

