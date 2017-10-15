
#ifndef DEEPAR_G2O_KALMANFILTER_H
#define DEEPAR_G2O_KALMANFILTER_H

#include <iostream>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/viz.hpp>
using namespace cv;

// for Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>
using Eigen::Vector2d;
using Eigen::Vector3d;

// for Sophus
#include <sophus/se3.h>
using Sophus::SE3;

// for g2o
//#include <g2o/core/base_vertex.h>
//#include <g2o/core/base_binary_edge.h>
//#include <g2o/core/block_solver.h>
//#include <g2o/core/optimization_algorithm_levenberg.h>
//#include <g2o/solvers/dense/linear_solver_dense.h>
//#include <g2o/core/robust_kernel.h>
//#include <g2o/types/sba/types_six_dof_expmap.h>
//using namespace g2o;

using namespace std;

#endif  // DEEPAR_G2O_KALMANFILTER_H
