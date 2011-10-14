#include <iostream>
#include <cv.h>
#include <ml.h>
#include <highgui.h>
#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/io.hpp>

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

vector<dvector> readData(string trainfile)
{
	vector<dvector> data;
	FILE *fp;
	if((fp = fopen(trainfile.c_str(), "r")) == NULL)
		{
			cerr << "no such file: " << trainfile << endl;
			exit(1);
		}
	float x,y,z;
	while(fscanf(fp,"%f %f %f\n",&x,&y,&z) != EOF)
		{
			dvector color(3);
			color(0) = x;
			color(1) = y;
			color(2) = z;
			data.push_back(color);
		}

	return data;
}

void train(string trainfile,CvEM& source_model)
{
	vector<dvector> data = readData(trainfile);

#ifndef ONLY_AB
  Mat src(data.size(),3,CV_32FC1);
	vector<dvector>::iterator it = data.begin();
  for(int i=0; it != data.end(); it++,i++)
		for(int j=0; j<3; j++)
			src.at<float>(i,j) = (*it)(j);
#else
  Mat src(data.size(),2,CV_32FC1);
	vector<dvector>::iterator it = data.begin();
  for(int i=0; it != data.end(); it++,i++)
		for(int j=0; j<2; j++)
			src.at<float>(i,j) = (*it)(j+1);
	coldim = 2;
#endif

  CvEMParams params(GaussN); //GaussN: number of gaussians

  // phase 1 (initial estimation)
  CvEM tmp_model;
  params.covs = NULL;
  params.means = NULL;
  params.weights = NULL;
  params.probs = NULL;
  params.cov_mat_type       = CvEM::COV_MAT_SPHERICAL;
  params.start_step         = CvEM::START_AUTO_STEP;
  params.term_crit.max_iter = 100;
  params.term_crit.epsilon  = 0.1;
  params.term_crit.type     = CV_TERMCRIT_ITER|CV_TERMCRIT_EPS;

  tmp_model.train(src, Mat(), params, NULL);

  // phase 2 (main estimation)
  params.cov_mat_type  = CvEM::COV_MAT_GENERIC; //COV_MAT_DIAGONAL, COV_MAT_SPHERICAL
  params.start_step    = CvEM::START_E_STEP;
  params.means = tmp_model.get_means();
  params.covs = (const CvMat**)tmp_model.get_covs();
  params.weights = tmp_model.get_weights();

  source_model.train(src, Mat(), params, NULL);
}

Mat proc(CvEM& source_model,Mat &src,string colormode)
{
	Mat tmp = Mat::zeros(src.rows,src.cols,CV_8UC3);
	if(colormode == "rgb")		   tmp = src.clone();
	else if(colormode == "lab")	 cvtColor(src,tmp,CV_RGB2Lab);
	else{	cerr << "no such mode" << endl;	exit(1); }

	Mat target = Mat::zeros(src.rows,src.cols,CV_32FC3);
	tmp.convertTo(target,CV_32FC3, 1.0);

  vector<Mat> means,icovs;
  CvEMParams p(GaussN); //GaussN: number of gaussians
  p.means = source_model.get_means();
  p.covs = (const CvMat**)source_model.get_covs();
  float means_v[GaussN][coldim], icovs_m[GaussN][coldim][coldim];
  for(int i=0; i<GaussN; i++)
    {
      CvMat *mat = cvCreateMat(coldim,coldim,CV_64F);
      cvInvert(p.covs[i],mat);
      for(int j=0; j<coldim; j++)
				{
					means_v[i][j] = cvmGet(p.means,i,j);
					for(int k=0; k<coldim; k++)
						icovs_m[i][j][k] = cvmGet(mat,j,k);
				}
      cvReleaseMat(&mat);

      means.push_back(Mat(1,coldim,CV_32F,means_v[i]));
      icovs.push_back(Mat(coldim,coldim,CV_32F,icovs_m[i]));
    }

  for(int i=0; i<target.rows; i++)
    {
      Vec3f* row = target.ptr<Vec3f>(i);
      for(int j=0; j<target.cols; j++)
				{
					double minval = DBL_MAX;
					for(int k=0; k<GaussN; k++)
						{
#ifndef ONLY_AB
							float vec[3] = {row[j].val[0], row[j].val[1], row[j].val[2]};
#else
							float vec[2] = {row[j].val[1], row[j].val[2]};
#endif
							double dis = Mahalanobis(means.at(k),Mat(1,coldim,CV_32F,vec),icovs.at(k));
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
			cerr << "usage: ./color imgfile trainfile colormode colordim" << endl;
			exit(1);
		}

	Mat img = imread(argv[1]);
	pruneSide(img);

  CvEM source_model;
	train(string(argv[2]),source_model);
	Mat dst = proc(source_model,img,string(argv[3]));

	imshow("org",img);
	cvMoveWindow("org",0,0);
	imshow("dst",dst);
	cvMoveWindow("dst",640,0);
	waitKey();

	return 1;
}
