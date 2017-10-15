#include "deepar_g2o_kalmanfilter.h"
#include "opencv2/viz.hpp"

int OptimizePoseAndPoint3d();
//float AverageReprojectError(vector<Vector3d> points_3d, vector<Vector2d> points_2d, Matrix3d R, Vector3d t, cv::Mat K);

//------------------------------ main 函数--------------------------------------------------

int main(int argc, char** argv)
{
    cout << "start" << endl;
    OptimizePoseAndPoint3d();

    return 0;


}

int OptimizePoseAndPoint3d()
{
    stringstream ss;
    string file_name;
    cv::FileStorage file_storage;
    cv::Mat image1, image2;
    Eigen::Matrix3d R1, R2;
    Eigen::Vector3d t1, t2;

    // 用 opencv_viz 模块显示位姿
    cv::viz::Viz3d pose_visualize("Pose");
    pose_visualize.showWidget("Coordinate Widget", cv::viz::WCoordinateSystem());
    // construct a cube widget
    cv::viz::WCube cube_widget(Point3f(-10.0,-10.0, -10.0), Point3f(10.0, 10.0, 10.0), true, cv::viz::Color::blue());
    cube_widget.setRenderingProperty(cv::viz::LINE_WIDTH, 4.0);
    pose_visualize.showWidget("Cube Widget", cube_widget);

    // 初始化 卡尔曼滤波器
    RNG rng;
    const int status_num = 12;
    const int measure_num = 6;
    KalmanFilter kalmanfilter(status_num, measure_num, 0);
    cv::Mat translation_matrix(12, 12, CV_32F);
    cv::setIdentity(translation_matrix);
    translation_matrix.at<float>(0,6) = 1;
    translation_matrix.at<float>(1,7) = 1;
    translation_matrix.at<float>(2,8) = 1;
    translation_matrix.at<float>(3,9) = 1;
    translation_matrix.at<float>(4,10) = 1;
    translation_matrix.at<float>(5,11) = 1;
    kalmanfilter.transitionMatrix = translation_matrix.clone();

    setIdentity(kalmanfilter.measurementMatrix);  // H
    setIdentity(kalmanfilter.processNoiseCov, Scalar::all(1e-5));  // Q
    setIdentity(kalmanfilter.measurementNoiseCov, Scalar::all(1e-1));   // R
    setIdentity(kalmanfilter.errorCovPost, Scalar::all(1));   // P

    rng.fill(kalmanfilter.statePost, RNG::UNIFORM, 0, 1);

    Mat measurement = Mat::zeros(measure_num, 1, CV_32F);
    Mat updated_status = Mat::zeros(status_num, 1, CV_32F);

    for(int i=1; i<40; i++)
    {
        ss << "/home/zcc/projects/qtProjects/monoDirectVO/bin/frame_" << i << "_data.yaml";
        ss >> file_name;
        ss.clear();
        ss.str("");
        cout << file_name << endl;
        file_storage.open(file_name, cv::FileStorage::READ);
        if(!file_storage.isOpened())
            continue;  // 没打开就下一个
        cout << "成功读到数据" << endl;
        cv::Mat image, points2d, pts2d, pts3d, pose;
        file_storage["img"] >> image;
        file_storage["pt2d"] >> points2d;
        file_storage["pt3d"] >> pts3d;
        file_storage["pose"] >> pose;

        cv::imshow("image", image/255.0);
        cv::waitKey(10);

        // 为了使用 EPnP,必须把 pt2d 转换成双通道的
        pts2d = cv::Mat(points2d.rows, 1, CV_32FC2);
        for(int i=0; i<points2d.rows; i++)
        {
            pts2d.at<cv::Vec2f>(i,0)[0] = points2d.at<float>(i,0);
            pts2d.at<cv::Vec2f>(i,0)[1] = points2d.at<float>(i,1);
        }

        cv::Mat R, t, K = (cv::Mat_<float>(3, 3) << 640, 0, 320, 0, 640, 240, 0, 0, 1);

        // EPnP求解位姿初值
        cv::Mat r, dist_coff = cv::Mat(4, 1, CV_32FC1, cv::Scalar(0));
        //cv::solvePnP(pt3d, pt2d, K, dist_coff, r, t, false, SOLVEPNP_EPNP);
        cv::solvePnP(pts3d, points2d, K, dist_coff, r, t, false);
        cv::Rodrigues(r, R);

        std::cout << "r= " << endl << r << endl;
        std::cout << "t= " << endl << t << endl;

        measurement.at<float>(0,0) = r.at<double>(0,0);
        measurement.at<float>(1,0) = r.at<double>(1,0);
        measurement.at<float>(2,0) = r.at<double>(2,0);
        measurement.at<float>(3,0) = t.at<double>(0,0);
        measurement.at<float>(4,0) = t.at<double>(1,0);
        measurement.at<float>(5,0) = t.at<double>(2,0);

        // 更新卡尔曼滤波器
        kalmanfilter.predict();
        kalmanfilter.correct(measurement);
        updated_status = kalmanfilter.statePost;


        // 显示当前位姿
        r.at<double>(0,0) = updated_status.at<float>(0,0);
        r.at<double>(1,0) = updated_status.at<float>(1,0);
        r.at<double>(2,0) = updated_status.at<float>(2,0);
        t.at<double>(0,0) = updated_status.at<float>(3,0);
        t.at<double>(1,0) = updated_status.at<float>(4,0);
        t.at<double>(2,0) = updated_status.at<float>(5,0);

        cv::Rodrigues(r, R);

        std::cout << "经过滤波后的姿态：" << endl;
        std::cout << "r= " << endl << r << endl;
        std::cout << "t= " << endl << t << endl;

        cv::Affine3d pose_aff(R, cv::Vec3d(t2[0],t2[1],t2[2]));
        pose_visualize.setWidgetPose("Cube Widget", pose_aff);
        pose_visualize.spinOnce(30,true);
    }

    return 0;
}



