#include <iostream>
#include <list>
#include <cv.h>
#include <ml.h>
#include <highgui.h>
#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/io.hpp>
#include "CopyParam.h"
#include "../blob/BlobResult.h"

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

	for(int i=0; i<src.cols; i++)
		for(int j=0; j<100; j++)
			src.at<uchar>(j,3*i) = src.at<uchar>(j,3*i+1) = src.at<uchar>(j,3*i+2) = 0;
}

Mat proc(Mat &src,string targetcolor)
{
	Mat tmp = Mat::zeros(src.rows,src.cols,CV_8UC3);
	cvtColor(src,tmp,CV_RGB2Lab);

	Mat target = Mat::zeros(src.rows,src.cols,CV_32FC3);
	tmp.convertTo(target,CV_32FC3, 1.0);

	float mave[GaussN][3], mcov[GaussN][3][3];
	getTrainingParam(GaussN,targetcolor,"lab",mave,mcov);

	Mat dst = Mat::zeros(src.rows,src.cols,CV_8UC3);
  for(int i=0; i<target.rows; i++)
    {
      Vec3f* row = target.ptr<Vec3f>(i);
      for(int j=0; j<target.cols; j++)
				{
					double minval = DBL_MAX;
					for(int k=0; k<GaussN; k++)
						{
							float vec[3] = {row[j].val[0], row[j].val[1], row[j].val[2]};
							Mat ave = Mat(1,coldim,CV_32F, mave[k]);
							Mat var = Mat(coldim,coldim,CV_32F, mcov[k]);
							double dis = Mahalanobis(ave,Mat(1,coldim,CV_32F,vec),var);
							minval = ((dis < minval) ? dis : minval);
						}

					if(minval < 1.5)
						{
							dst.at<uchar>(i,3*j  ) = src.at<uchar>(i,3*j  );
							dst.at<uchar>(i,3*j+1) = src.at<uchar>(i,3*j+1);
							dst.at<uchar>(i,3*j+2) = src.at<uchar>(i,3*j+2);
						}
				}
    }

	return dst;
}

void getClusterMap(IplImage *src,CvSeq *edges,list< pair<int,int> > *points,CvScalar col)
{
  CvSeqReader reader;
  CBlob::vectorPunts vectorEdges = CBlob::vectorPunts( edges->total );
  cvStartReadSeq( edges, &reader);
  CBlob::vectorPunts::iterator itEdges1 = vectorEdges.begin();
  for(; itEdges1 != vectorEdges.end(); itEdges1++)
    CV_READ_SEQ_ELEM( (*itEdges1) ,reader);
  std::sort(vectorEdges.begin(), vectorEdges.end(), CBlob::comparaCvPoint());

  itEdges1 = vectorEdges.begin();
  CBlob::vectorPunts::iterator itEdges2 = vectorEdges.begin() + 1;
  bool dinsBlob = true;
  int yActual;
  CvPoint  pt1, pt2;
  for(int i=0; itEdges1 != (vectorEdges.end() - 1); itEdges1++,itEdges2++,i++)
    {
      yActual = (*itEdges1).y;
      if(((*itEdges1).x != (*itEdges2).x) && ((*itEdges2).y == yActual) )
				{
					if( dinsBlob )
						{
							pt1 = *itEdges1;
							pt2 = *itEdges2;
							for(int i=pt1.x,j=pt1.y; i<=pt2.x; i++)
								(*points).push_back( pair<int,int>(i,j) );
							cvLine(src,pt1,pt2,col);
						}
					dinsBlob =! dinsBlob;
				}
      if( (*(itEdges1+1)).y != yActual ) dinsBlob = true;
    }
  vectorEdges.clear();
}

void clustering(Mat &_src)
{
	Mat _gray = Mat::zeros(_src.rows,_src.cols,CV_8U);
	cvtColor(_src,_gray,CV_RGB2GRAY);

	IplImage gray = _gray;
	IplImage src  = _src;
	CBlobResult blobs = CBlobResult(&gray, NULL, 20, false);
	blobs.Filter(blobs,B_INCLUDE,CBlobGetArea(),B_INSIDE,20,10000);
	list< pair<int,int> > points;
	for(int i=0; i<blobs.GetNumBlobs(); i++)
		{
			CvScalar col = cvScalar(0,0,255);

			CBlob blob = blobs.GetBlob(i);
			getClusterMap(&src,blob.Edges(),&points,col);

			CvPoint2D32f pt[4];
			CvBox2D box = blob.GetEllipse();
			cvBoxPoints (box, pt);
      cvLine (&src, cvPointFrom32f (pt[0]), cvPointFrom32f (pt[1]), CV_RGB (0, 255, 0));
      cvLine (&src, cvPointFrom32f (pt[1]), cvPointFrom32f (pt[2]), CV_RGB (0, 255, 0));
      cvLine (&src, cvPointFrom32f (pt[2]), cvPointFrom32f (pt[3]), CV_RGB (0, 255, 0));
      cvLine (&src, cvPointFrom32f (pt[3]), cvPointFrom32f (pt[0]), CV_RGB (0, 255, 0));
		}
}

int main(int argc,char *argv[])
{
	if(argc < 3)
		{
			cerr << "usage: ./color imgfile targetcolor" << endl;
			exit(1);
		}

	Mat img = imread(argv[1]);
	pruneSide(img);

	Mat dst = proc(img,string(argv[2]));

	clustering(dst);

	imshow("org",img);
	cvMoveWindow("org",0,0);
	imshow("dst",dst);
	cvMoveWindow("dst",640,0);
	waitKey();

	return 1;
}

