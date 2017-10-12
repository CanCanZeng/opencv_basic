#include <iostream>
#include <fstream>

#include <math.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/calib3d.hpp>

#include <Eigen/Core>
#include <chrono>

#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"
#include "g2o/types/slam3d/vertex_se3.h"
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_binary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/types/sba/types_six_dof_expmap.h>

using namespace std;
using namespace cv;

class VertexPointUV : public g2o::BaseVertex<2, Eigen::Vector2d>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    VertexPointUV() {}
    virtual bool read(istream &is) {return false;}
    virtual bool write(ostream &os) const {return false;}
    virtual void setToOriginImpl(){}
    virtual void oplusImpl(const double* update) override {
        Eigen::Vector2d::ConstMapType v(update);
        _estimate += v;
    }
};


class EdgeProjectUV2XYZ : public g2o::BaseBinaryEdge<3, Eigen::Vector3d, VertexPointUV,g2o::VertexSE3Expmap>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    EdgeProjectUV2XYZ() {}

    virtual void computeError() override
    {
        const VertexPointUV* ppt2d = static_cast<const VertexPointUV*>(_vertices[0]);
//        VertexPointUV pt2d = _vertices[0];
        Eigen::Vector2d uv = ppt2d->estimate();
        double x = uv[0];
        double y = uv[1];
        double X = _measurement[0];
        double Y = _measurement[1];
        double Z = _measurement[2];
        double cos_theta = (x*X +y*Y + Z) / sqrt((X*X+Y*Y+Z*Z)*(x*x+y*y+1));
        _error[0] = sqrt((X*X+Y*Y+Z*Z)*(1-cos_theta*cos_theta));
        _error[1] = 0;
        _error[2] = 0;
    }



    bool read(istream &is){}
    bool write(ostream &os) const {}
};

// 定义了一个 9 维的顶点，同时优化位姿和内参数
class VertexCameraBAL : public g2o::BaseVertex<9, Eigen::VectorXd>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    VertexCameraBAL() {}
    virtual bool read(istream &is) {return false;}
    virtual bool write(ostream &os) const {return false;}
    virtual void setToOriginImpl() {}

    virtual void oplusImpl(const double* update) override
    {
        Eigen::VectorXd::ConstMapType v(update, VertexCameraBAL::Dimension);
        _estimate += v;
    }
};

// 二维的投影点
class VertexPointBAL : public g2o::BaseVertex<3, Eigen::Vector3d>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    VertexPointBAL() {}
    virtual bool read(istream &is) {}
    virtual bool write(ostream &os) const {return false;}
    virtual void setToOriginImpl() {}

    virtual void oplusImpl(const double* update) override
    {
        Eigen::Vector3d::ConstMapType v(update);
        _estimate += v;
    }
};

// 重投影误差边， 用 ceres 自动求导
class EdgeObservationBAL : public g2o::BaseBinaryEdge<2, Eigen::Vector2d, VertexCameraBAL, VertexPointBAL>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    EdgeObservationBAL() {}
    virtual bool read(istream &is) {return false;}
    virtual bool write(ostream &os) const {return false;}

    virtual void computeError() override
    {
        const VertexCameraBAL* cam = static_cast<const VertexCameraBAL*>(vertex(0));
        const VertexPointBAL*  point = static_cast<const VertexPointBAL*>(vertex(1));

        (*this)(cam->estimate().data(), point->estimate().data(), _error.data());
    }

    // 为了使用 Ceres 求导功能而定义的函数，让本类成为拟函数类
    template<typename T>
    bool operator() (const T* camera, const T* point, const T* residuals) const
    {
        T predictions[2];
        CamProjectionWithDistortion(camera, point, predictions);
        residuals[0] = predictions[0] - T(measurement()[0]);
        residuals[1] = predictions[1] - T(measurement()[1]);
        return true;
    }
};

void PoseEstimationWithBA(cv::Mat obj_points, cv::Mat im_points,
                          cv::Mat K, cv::Mat dist_coff,
                          cv::Mat& R, cv::Mat t)
{
    cv::Mat r;
    cv::solvePnP(obj_points, im_points, K, dist_coff, r, t, false, SOLVEPNP_EPNP);
    cv::Rodrigues(r, R);

    // 初始化 g2o
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<6,2> > Block; // pose 维度为6, 待优化的点维度为 2
    Block::LinearSolverType* linear_solver = new g2o::LinearSolverCSparse<Block::PoseMatrixType>();
    Block* solver_ptr = new Block(linear_solver);
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);

    // 顶点
    g2o::VertexSE3Expmap* pose = new g2o::VertexSE3Expmap();  // camera pose
    Eigen::Matrix3d R_mat;
    R_mat <<
             R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2),
            R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2),
            R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2)
            ;
    pose->setId(0);
    pose->setEstimate(g2o::SE3Quat(R_mat, Eigen::Vector3d(t.at<double>(0,0),t.at<double>(1,0), t.at<double>(2,0))));
    optimizer.addVertex(pose);

    for(int i=0; i<im_points.rows; i++)
    {
        VertexPointUV* point = new VertexPointUV();
        point->setId(1+i);
        point->setEstimate(Eigen::Vector2d(im_points.at<cv::Vec2d>(i,0)[0], im_points.at<cv::Vec2d>(i,0)[1]));
        point->setMarginalized(true);
        optimizer.addVertex(point);
    }

    // pamarameter: camera intrinsics
    g2o::CameraParameters* camera = new g2o::CameraParameters(
                K.at<double>(0,0), Eigen::Vector2d(K.at<double>(0,2), K.at<double>(1,2)), 0);
    camera->setId(0);
    optimizer.addParameter(camera);

    // edges
    for(int i=0; i<obj_points.rows; i++)
    {
        EdgeProjectUV2XYZ* edge = new EdgeProjectUV2XYZ();
        edge->setId(i+1);
        edge->setVertex(0, dynamic_cast<VertexPointUV*>(optimizer.vertex(i+1)));
        edge->setVertex(1, pose);
        edge->setMeasurement((Eigen::Vector3d(obj_points.at<double>(i,0),
                                     obj_points.at<double>(i,1),
                                     obj_points.at<double>(i,2)
                                     )));
        edge->setParameterId(0,0);
        edge->setInformation(Eigen::Matrix2d::Identity());
        optimizer.addEdge(edge);
    }

    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    optimizer.setVerbose(true);
    optimizer.initializeOptimization();
    optimizer.optimize(100);
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double> >(t2-t1);
    cout << "optimization costs time: " << time_used.count() << " seconds." << endl;

    cout << endl << "after optimization: " << endl;
    cout << "T = " << endl << Eigen::Isometry3d(pose->estimate()).matrix() << endl;
}

int main(int argc, char** argv)
{

    cv::Mat obj_points, im_points, K, dist, R, t;
    obj_points = cv::Mat(563, 3, CV_64FC1);
    im_points = cv::Mat(563, 1, CV_64FC2);
    K = (cv::Mat_<double>(3, 3) << 640, 0, 320, 0, 640, 240, 0, 0, 1);
    dist = cv::Mat(4, 1, CV_64FC1, Scalar(0));

    ifstream infile;
    infile.open("MapP3D.txt");
    assert(infile.is_open());
    for(int i=0; i<563; i++)
    {
        double x, y, z;
        infile >> x >> y >> z;
        obj_points.at<double>(i,0) = x;
        obj_points.at<double>(i,1) = y;
        obj_points.at<double>(i,2) = z;
    }
    infile.close();

//    ifstream infile;
    infile.open("MapP2D.txt");
    assert(infile.is_open());
    for(int i=0; i<563; i++)
    {
        double u, v;
        infile >> u >> v;
        im_points.at<cv::Vec2d>(i,0)[0] = u;
        im_points.at<cv::Vec2d>(i,0)[1] = v;
    }
    infile.close();
    std::cout << K << endl;
//    cv::Mat r;
//    cv::solvePnP(obj_points, im_points, K, dist, r, t, false, 0);

//    cv::Rodrigues(r, R);
//    std::cout << R << endl;
//    std::cout << t << endl;


//    cv::solvePnP(obj_points, im_points, K, dist, r, t, false, SOLVEPNP_EPNP);
//    cv::Rodrigues(r, R);
//    std::cout <<R << endl;
//    std::cout << t << endl;


    PoseEstimationWithBA(obj_points, im_points, K, dist, R, t);

    return 0;
}
