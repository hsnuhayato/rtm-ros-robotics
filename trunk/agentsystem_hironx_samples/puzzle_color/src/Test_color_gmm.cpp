#include <iostream>
#include <cv.h>
#include <ml.h>
#include <highgui.h>
#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/io.hpp>
#include <colorlib.h>

//#define ONLY_AB

using namespace cv;

typedef boost::numeric::ublas::vector<double> dvector;

static const int GaussN = 3;
static int coldim = 3;

void pruneSide(Mat& src)
{
	Mat target = Mat::zeros(src.rows,src.cols,CV_8UC3);
	for(uint i=0; i<100; i++)
		for(int j=0; j<src.rows; j++)
			src.at<uchar>(j,3*i) = src.at<uchar>(j,3*i+1) = src.at<uchar>(j,3*i+2) = 0;

	for(int i=src.cols-100; i<src.cols; i++)
		for(int j=0; j<src.rows; j++)
			src.at<uchar>(j,3*i) = src.at<uchar>(j,3*i+1) = src.at<uchar>(j,3*i+2) = 0;
}

Mat proc(Mat &src,string colormode)
{
	Mat tmp = Mat::zeros(src.rows,src.cols,CV_8UC3);
	if(colormode == "rgb")		   tmp = src.clone();
	else if(colormode == "lab")	 cvtColor(src,tmp,CV_RGB2Lab);
	else{	cerr << "no such mode" << endl;	exit(1); }

	Mat target = Mat::zeros(src.rows,src.cols,CV_32FC3);
	tmp.convertTo(target,CV_32FC3, 1.0);

  for(int i=0; i<target.rows; i++)
    {
      Vec3f* row = target.ptr<Vec3f>(i);
      for(int j=0; j<target.cols; j++)
				{
					double minval = DBL_MAX;
					for(int k=0; k<GaussN; k++)
						{
							float vec[3] = {row[j].val[0], row[j].val[1], row[j].val[2]};
							//double dis = Mahalanobis(means.at(k),Mat(1,coldim,CV_32F,vec),icovs.at(k));
							Mat ave = Mat(1,coldim,CV_32F, brown_lab_ave[k]);
							Mat var = Mat(coldim,coldim,CV_32F, brown_lab_var[k]);
							double dis = Mahalanobis(ave,Mat(1,coldim,CV_32F,vec),var);
							minval = ((dis < minval) ? dis : minval);
						}

					if(minval > 1.5)
						row[j].val[0] = row[j].val[1] = row[j].val[2] = 0.0;
				}
    }

	if(colormode == "lab")
		cvtColor(target,target,CV_Lab2BGR);

	return target;
}


int main(int argc,char *argv[])
{
	if(argc < 3)
		{
			cerr << "usage: ./color imgfile trainfile colormode" << endl;
			exit(1);
		}

	Mat img = imread(argv[1]);
	pruneSide(img);

	Mat dst = proc(img,string(argv[3]));

	imshow("org",img);
	cvMoveWindow("org",0,0);
	imshow("dst",dst);
	cvMoveWindow("dst",640,0);
	waitKey();

	return 1;
}
