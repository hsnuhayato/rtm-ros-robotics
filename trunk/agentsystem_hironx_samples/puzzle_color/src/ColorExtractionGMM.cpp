#include <iostream>
#include <limits>
#include <highgui.h>
#include "ColorExtractionGMM.h"
#include <float.h>

ColorExtractionGMM::ColorExtractionGMM(void)
  :UseLab(false),
   GaussN(4)
{
}

ColorExtractionGMM::~ColorExtractionGMM(void)
{
  cerr << "ColorEstractinGMM class was deleted" << endl;
}

void ColorExtractionGMM::makeComparedData(const CvEM model, 
					  vector<Mat> *means, vector<Mat> *icovs)
{
  CvEMParams p(GaussN); //GaussN: number of gaussians
  p.means = model.get_means();
  p.covs = (const CvMat**)model.get_covs();

  for(int i=0; i<GaussN; i++)
    {
      float meanv[3] = {cvmGet(p.means,i,0),cvmGet(p.means,i,1),cvmGet(p.means,i,2)};
      float covsm[3][3] = {{cvmGet(p.covs[i],0,0),cvmGet(p.covs[i],0,1),cvmGet(p.covs[i],0,2)},
			   {cvmGet(p.covs[i],1,0),cvmGet(p.covs[i],1,1),cvmGet(p.covs[i],1,2)},
			   {cvmGet(p.covs[i],2,0),cvmGet(p.covs[i],2,1),cvmGet(p.covs[i],2,2)}};
      
      Mat m = Mat(1,3,CV_32F,meanv);
      Mat c = Mat(3,3,CV_32F,covsm).inv();
      means->push_back(m);
      icovs->push_back(c);
    }
}

void ColorExtractionGMM::proc(Mat& _source, Mat& train, Mat& _target)
{
  Mat source;
  _source.convertTo(source,CV_32F,1.0/255.0);
  Mat target;
  _target.convertTo(target,CV_32F,1.0/255.0);

  if(UseLab)
    {
      cvtColor(source,source,CV_BGR2Lab);
      cvtColor(target,target,CV_BGR2Lab);
    }

  CvEM source_model;
  TrainGMM(source_model,train);

  CvEMParams p(GaussN); //GaussN: number of gaussians
  p.means = source_model.get_means();
  p.covs = (const CvMat**)source_model.get_covs();
  float means_v[GaussN][3], icovs_m[GaussN][3][3];
  vector<Mat> means,icovs;
  for(int i=0; i<GaussN; i++)
    {
      CvMat *mat = cvCreateMat(3,3,CV_64F);
      cvInvert(p.covs[i],mat);
      for(int j=0; j<3; j++)
				{
					means_v[i][j] = cvmGet(p.means,i,j);
					for(int k=0; k<3; k++)
						icovs_m[i][j][k] = cvmGet(mat,j,k);
				}
      cvReleaseMat(&mat);

      means.push_back(Mat(1,3,CV_32F,means_v[i]));
      icovs.push_back(Mat(3,3,CV_32F,icovs_m[i]));
    }

  for(int i=0; i<target.rows; i++)
    {
      Vec3f* row = target.ptr<Vec3f>(i);
      for(int j=0; j<target.cols; j++)
				{
					double minval = DBL_MAX;
					for(int k=0; k<GaussN; k++)
						{
							float vec[3] = {row[j].val[0], row[j].val[1], row[j].val[2]};
							double dis = Mahalanobis(means.at(k),Mat(1,3,CV_32F,vec),icovs.at(k));
							if(dis < minval)
								minval = dis;
						}

					if(minval > 1.5)
						row[j].val[0] = row[j].val[1] = row[j].val[2] = 0.0;
				}
    }

  if(UseLab)
    {
      cvtColor(target,target,CV_Lab2BGR);
      cvtColor(source,source,CV_Lab2BGR);
    }

  namedWindow("training image");
  imshow("training image",train);
  namedWindow("source orig");
  imshow("source orig",source);
  namedWindow("orig target");
  imshow("orig target",target);
}

void ColorExtractionGMM::TrainGMM(CvEM& source_model, Mat& train)
{
  source_model.clear();

  Mat train_gray,train_32f;
  cvtColor(train,train_gray,CV_BGR2GRAY);
  train.convertTo(train_32f,CV_32F,1.0/255.0);
  int src_samples_size = countNonZero(train_gray);
  Mat source_samples(src_samples_size,3,CV_32FC1);

  for(int y=0,sample_count=0; y<train.rows; y++)
    {
      Vec3f* row = train_32f.ptr<Vec3f>(y);
      uchar* train_row = train_gray.ptr<uchar>(y);
      for(int x=0; x<train.cols; x++)
	if(train_row[x] > 0)
	  source_samples.at<Vec3f>(sample_count++,0) = row[x];
    }
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

  tmp_model.train(source_samples, Mat(), params, NULL);

  // phase 2 (main estimation)
  params.cov_mat_type  = CvEM::COV_MAT_GENERIC;
  //params.cov_mat_type  = CvEM::COV_MAT_DIAGONAL;
  //params.cov_mat_type       = CvEM::COV_MAT_SPHERICAL;
  params.start_step    = CvEM::START_E_STEP;
  params.means = tmp_model.get_means();
  params.covs = (const CvMat**)tmp_model.get_covs();
  params.weights = tmp_model.get_weights();

  source_model.train(source_samples, Mat(), params, NULL);
}



