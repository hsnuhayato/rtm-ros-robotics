// -*- C++ -*-
/*!
 * Ryo Hanai <hanai@jsk.t.u-tokyo.ac.jp>
 */

#include <cv.h>
#include <highgui.h>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <boost/regex.hpp>
#include <boost/tuple/tuple.hpp>
#include <boost/tuple/tuple_io.hpp>
//#include <boost/multi_array.hpp>
#include <boost/foreach.hpp>
//#include <boost/numeric/ublas/vector.hpp>
//#include <boost/numeric/ublas/io.hpp>

// typedef boost::numeric::ublas::vector<double> dvector;
// typedef boost::numeric::ublas::vector<double> dvector2;

#include <piece_recog.h>

using namespace cv;
using namespace std;

const double PI = M_PIl;

string getExtension(string s) {
    boost::regex r("\\.[^\\.]+$");
    boost::smatch m;

    if (boost::regex_search(s, m, r) ) {
        // cout << "found(pos =" << m.position() << endl;
        return m.str();
    }

    string empty_string = "";
    return empty_string;
}

double redvertices[][3] = {
  {-15.000000, -15.000000, 15.000000},
  {-15.000000, 15.000000, 15.000000},
  {15.000000, -15.000000, 15.000000},
  {15.000000, 15.000000, 15.000000},
  {45.000000, -15.000000, 15.000000},
  {45.000000, 15.000000, 15.000000},
  {15.000000, 45.000000, 15.000000},
  {45.000000, 45.000000, 15.000000},
  {75.000000, 15.000000, 15.000000},
  {75.000000, 45.000000, 15.000000}
};
int rededges[][2] = {
  {0, 1},
  {0, 2},
  {1, 3},
  {2, 3},
  {2, 4},
  {4, 5},
  {3, 5},
  {3, 6},
  {6, 7},
  {5, 7},
  {5, 8},
  {7, 9},
  {8, 9}
};
double aquavertices[][3] = {
  {-15.000000, -15.000000, 15.000000},
  {15.000000, -15.000000, 15.000000},
  {-15.000000, 15.000000, 15.000000},
  {15.000000, 15.000000, 15.000000},
  {-15.000000, 45.000000, 15.000000},
  {15.000000, 45.000000, 15.000000},
  {-15.000000, 75.000000, 15.000000},
  {15.000000, 75.000000, 15.000000},
  {45.000000, 15.000000, 15.000000},
  {45.000000, 45.000000, 15.000000}
};
int aquaedges[][2] = {
  {0, 1},
  {0, 2},
  {1, 3},
  {2, 3},
  {2, 4},
  {4, 5},
  {3, 5},
  {4, 6},
  {6, 7},
  {5, 7},
  {3, 8},
  {8, 9},
  {9, 5}
};
double yellowvertices[][3] = {
  {-15.000000, 15.000000, 15.000000},
  {15.000000, 15.000000, 15.000000},
  {45.000000, 15.000000, 15.000000},
  {-15.000000, -15.000000, 15.000000},
  {15.000000, -15.000000, 15.000000},
  {45.000000, -15.000000, 15.000000},
  {-15.000000, -45.000000, 15.000000},
  {15.000000, -45.000000, 15.000000},
  {-15.000000, -75.000000, 15.000000},
  {15.000000, -75.000000, 15.000000}
};
int yellowedges[][2] = {
  {0, 1},
  {1, 2},
  {0, 3},
  {1, 4},
  {2, 5},
  {3, 4},
  {4, 5},
  {3, 6},
  {4, 7},
  {6, 7},
  {6, 8},
  {7, 9},
  {8, 9}
};
double yellowgreenvertices[][3] = {
  {-15.000000, -15.000000, 15.000000},
  {15.000000, -15.000000, 15.000000},
  {45.000000, -15.000000, 15.000000},
  {-15.000000, 15.000000, 15.000000},
  {15.000000, 15.000000, 15.000000},
  {45.000000, 15.000000, 15.000000},
  {-15.000000, 45.000000, 15.000000},
  {15.000000, 45.000000, 15.000000}
};
int yellowgreenedges[][2] = {
  {0, 1},
  {1, 2},
  {3, 4},
  {4, 5},
  {6, 7},
  {0, 3},
  {1, 4},
  {2, 5},
  {3, 6},
  {4, 7}
};
double brownvertices[][3] = {
  {-15.000000, -15.000000, 15.000000},
  {15.000000, -15.000000, 15.000000},
  {45.000000, -15.000000, 15.000000},
  {-15.000000, 15.000000, 15.000000},
  {15.000000, 15.000000, 15.000000},
  {45.000000, 15.000000, 15.000000},
  {-15.000000, 45.000000, 15.000000},
  {15.000000, 45.000000, 15.000000},
  {45.000000, 45.000000, 15.000000}
};
int brownedges[][2] = {
  {0, 1},
  {1, 2},
  {3, 4},
  {4, 5},
  {6, 7},
  {7, 8},
  {0, 3},
  {3, 6},
  {1, 4},
  {4, 7},
  {2, 5},
  {5, 8}
};

class State {
public:
    State (double x, double y, double theta, double scale) {
	x_ = x; y_ = y; theta_ = theta; scale_ = scale;
    }
    State () { State (0.0, 0.0, 0.0, 0.0); }

    double x_;
    double y_;
    double theta_;
    double scale_;
};

typedef boost::tuple<double,double,double> vertice;
typedef boost::tuple<int,int> edge;

class Model {
public:
    Model() { }

    void load(double (*vertices)[3], int nvertices, int (*edges)[2], int nedges) {
	for (int i = 0; i < nvertices; i++) {
	    vertices_.push_back(boost::make_tuple(vertices[i][0],
						  vertices[i][1],
						  vertices[i][2]));
	}
	for (int i = 0; i < nedges; i++) {
	    edges_.push_back(boost::make_tuple(edges[i][0],
					       edges[i][1]));
	}
    }

    std::vector<vertice> vertices_;
    std::vector<edge> edges_;
};

static inline void trans(Point& p, State& st, Point& q) {
    double l = st.scale_;
    double theta = st.theta_;
    double c = cos(theta);
    double s = sin(theta);
    q.x = st.x_ + l*c*p.x - l*s*p.y;
    q.y = st.y_ + l*s*p.x + l*c*p.y;
}

static inline double eval_edge(Point& p0, Point& p1, Mat& dtf) {
    double d = 0.0;
    int n = 10;
    int c = dtf.channels();

    for (int i = 0; i < n; i++) {
	double x = p0.x + (p1.x-p0.x)*i/n;
	double y = p0.y + (p1.y-p0.y)*i/n;
	double v = dtf.data[dtf.step * (int)y + (int)x * c];
	d -= v;
    }
    
    return d/n;
}

double eval_likelihood(State& st, Mat& observation, Model& model,
		       Mat& cameraMat, Mat& distCoeffs) {
    double lk = 0.0;
    Point p0, p1, q0, q1;

    vector<vertice> vertices = model.vertices_;
    vector<edge> edges = model.edges_;

    for (vector<edge>::iterator it = edges.begin(); 
	 it != edges.end(); it++) {

	vertice v0 = vertices[it->get<0>()];
	vertice v1 = vertices[it->get<1>()];
	Point p0, p1;
	p0.x = v0.get<0>(); p0.y = v0.get<1>();
	p1.x = v1.get<0>(); p1.y = v1.get<1>();

	// vector<Point3f> objectPoints;
	// Mat rvec;
	// Mat tvec;
	// vector<Point2f> imagePoints;

	// rvec[0] = 0; rvec[1] = 0; rvec[2] = 0;
	// projectPoints(objectPoints, rvec, tvec, cameraMat, distCoeffs, imagePoints);

	trans(p0, st, q0);
	trans(p1, st, q1);

	lk += eval_edge(q0, q1, observation);
    }

    // for (unsigned int i = 0; i < sizeof(edges)/(sizeof(int)*2); i++) {
    // 	double* v0 = vertices[edges[i][0]];
    // 	double* v1 = vertices[edges[i][1]];
    // 	Point p0, p1;
    // 	p0.x = v0[0]; p0.y = v0[1];
    // 	p1.x = v1[0]; p1.y = v1[1];

    // 	trans(p0, st, q0);
    // 	trans(p1, st, q1);

    // 	lk += eval_edge(q0, q1, observation);
    // }
    
    return lk;
}

void draw_result(Mat& img, State& st, Model& model) {
    Point p0, p1, q0, q1;

    vector<vertice> vertices = model.vertices_;
    vector<edge> edges = model.edges_;

    for (vector<edge>::iterator it = edges.begin(); 
	 it != edges.end(); it++) {

	vertice v0 = vertices[it->get<0>()];
	vertice v1 = vertices[it->get<1>()];
	Point p0, p1;
	p0.x = v0.get<0>(); p0.y = v0.get<1>();
	p1.x = v1.get<0>(); p1.y = v1.get<1>();
	trans(p0, st, q0);
	trans(p1, st, q1);
	line(img, q0, q1, Scalar(0, 255, 0), 2);
    }
}

void loadCameraParameter(string calibFile, Mat& cameraMat, Mat& distCoeffs) {
    int width, height;
    FileStorage cvfs(calibFile, CV_STORAGE_READ);
    FileNode node(cvfs.fs, NULL);
    width = node["image_width"];
    height = node["image_height"];
    FileNode fn = node[string("camera_matrix")];
    read(fn[0], cameraMat);
    read(fn[1], distCoeffs);
}

void recognize (string imgfile, string piece, bool debug) {
    Mat cameraMat;
    Mat distCoeffs;
    loadCameraParameter ("data/camera_calib_rhand.yaml", cameraMat, distCoeffs);

    Mat observation = imread(imgfile, 3);
    Mat hsvimg;
    cvtColor(observation, hsvimg, CV_BGR2HSV);
    std::vector<cv::Mat> planes;
    cv::split(hsvimg, planes);
    
    // Mat hedge;
    // adaptiveThreshold(planes[0], hedge, 255,
    // 		      ADAPTIVE_THRESH_GAUSSIAN_C,
    // 		      THRESH_BINARY, 5, 5);
    // Mat iedge;
    // adaptiveThreshold(planes[2], iedge, 255,
    // 		      ADAPTIVE_THRESH_GAUSSIAN_C,
    // 		      THRESH_BINARY, 15, 7);

    // int hedgeThresh = 10;    
    // Mat chedge;
    // blur(planes[0], chedge, Size(3,3));
    // Canny(chedge, chedge, hedgeThresh, hedgeThresh*3, 3);
    // bitwise_not(chedge, chedge);

    int edgeThresh = 22;
    Mat ciedge;
    blur(planes[2], ciedge, Size(3,3));
    Canny(ciedge, ciedge, edgeThresh, edgeThresh*3, 3);
    bitwise_not(ciedge, ciedge);

    Mat dtf, dtf_norm;
    distanceTransform(ciedge, dtf, CV_DIST_L2, 5);
    normalize(dtf, dtf_norm, 0.0, 255.0, CV_MINMAX, NULL);

    // GaussianBlur(ciedge, dtf_norm, Size(19,19), 19);
    // for (int y = 0; y < ciedge.rows; y++) {
    // 	for (int x = 0; x < ciedge.cols; x++) {
    // 	    double val1 = dtf_norm.data[dtf_norm.step*y + x];
    // 	    double val2 = ciedge.data[ciedge.step*y + x];
    // 	    dtf_norm.data[dtf_norm.step*y +x] = (val1+val2)/2;
    // 	}
    // }
    //add(dtf, ciedge, dtf_norm);
    //normalize(dtf_norm, dtf_norm, 0.0, 255.0, CV_MINMAX, NULL);

    State st(140,350,-PI/6.0,2.1);
    State stmax;
    Model mdl;

    if (piece == "red") {
	mdl.load(redvertices, sizeof(redvertices)/(sizeof(double)*3),
		 rededges, sizeof(rededges)/(sizeof(int)*2));
    } else if (piece == "aqua") {
	mdl.load(aquavertices, sizeof(aquavertices)/(sizeof(double)*3),
		 aquaedges, sizeof(aquaedges)/(sizeof(int)*2));
    } else if (piece == "yellow") {
	mdl.load(yellowvertices, sizeof(yellowvertices)/(sizeof(double)*3),
		 yellowedges, sizeof(yellowedges)/(sizeof(int)*2));
    } else if (piece == "yellowgreen") {
	mdl.load(yellowgreenvertices, sizeof(yellowgreenvertices)/(sizeof(double)*3),
		 yellowgreenedges, sizeof(yellowgreenedges)/(sizeof(int)*2));
    } else if (piece == "brown") {
	mdl.load(brownvertices, sizeof(brownvertices)/(sizeof(double)*3),
		 brownedges, sizeof(brownedges)/(sizeof(int)*2));
    } else {
	return;
    }

    double lkmax = -DBL_MAX;
    int m = 30;
    int n = 72;
    int o = 3;
    int cx = 240;
    int cy = 320;
    int dx = 4;
    int dy = 4;
    for (int i = -m; i <= m; i++) {
	st.x_ = cx + i*dx;
	for (int j = -m; j <= m; j++) {
	    st.y_ = cy + j*dy;
	    for (int k = 0; k < n; k++) {
		st.theta_ = 2.0*PI*k/n;
		for (int l = -o; l <= o; l++) {
		    //st.scale_ = 2.1 * expf(0.03*l);
		    st.scale_ = 2.6 * expf(0.05*l);
		    double lk = eval_likelihood(st, dtf_norm, mdl,
						cameraMat, distCoeffs);
		    if (lk > lkmax) {
			lkmax = lk; stmax = st;
		    }

		    // cout << "LK:" << st.theta_ << ", " << lk << endl;
		    // Mat tmp;
		    // observation.copyTo(tmp);
		    // draw_result(tmp, st, mdl);
		    // imshow("likelihood", tmp);
		    // waitKey(0);
		}
	    }
	}
    }

    cout << "MAX" << endl;
    cout << stmax.x_ << "," << stmax.y_ << ","
	 << stmax.theta_ << ", " << stmax.scale_ << "," << lkmax << endl;

    draw_result(observation, stmax, mdl);
    draw_result(dtf_norm, stmax, mdl);

    if (debug) {
	imshow("canny intensity edge", ciedge);
	imshow("distance transform", dtf_norm);
	//imshow("intensity edge", iedge);
	//imshow("hedge", hedge);
	// imshow("canny hue edge", chedge);
    }

    imshow("observation", observation);
    waitKey(0);
}

#define BOOST_PYTHON_STATIC_LIB
#include <boost/python.hpp>

BOOST_PYTHON_MODULE( libpiece_recog )
{
    using namespace boost::python;
    def("recognize", &recognize);
}

