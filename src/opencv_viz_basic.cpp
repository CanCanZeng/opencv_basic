#include "opencv_viz_basic.h"
#include <opencv2/video/tracking.hpp>

int ReadData(string full_file_name, vector<cv::Mat>& R_vec, vector<cv::Vec3f>& t_vec);

int main(int argc, char** argv)
{
    if(argc != 2)
        cerr << "Usage: opencv_viz_basic full_file_name!" << endl;

    string full_file_name(argv[1]);
    vector<cv::Mat> R_vec;
    vector<cv::Vec3f> t_vec;
    ReadData(full_file_name, R_vec, t_vec);

    // create a window
    cv::viz::Viz3d pose_visualize("Pose");
    pose_visualize.showWidget("Coordinate Widget", cv::viz::WCoordinateSystem());

    // 创建一个圆柱体
    cv::viz::WCylinder cylinder_widget(cv::Point3f(0,-5.0,62), cv::Point3d(0,5.0,62), 3);
    pose_visualize.showWidget("cylinder", cylinder_widget);

//    RNG rng;
//    const int status_num = 12;
//    const int measure_num = 6;
//    cv::KalmanFilter kalmanfilter(status_num, measure_num, 0);
//    cv::Mat translation_matrix(12, 12, CV_32F);
//    cv::setIdentity(translation_matrix);
//    translation_matrix.at<float>(0,6) = 1;
//    translation_matrix.at<float>(1,7) = 1;
//    translation_matrix.at<float>(2,8) = 1;
//    translation_matrix.at<float>(3,9) = 1;
//    translation_matrix.at<float>(4,10) = 1;
//    translation_matrix.at<float>(5,11) = 1;
//    kalmanfilter.transitionMatrix = translation_matrix.clone();

//    setIdentity(kalmanfilter.measurementMatrix);  // H
//    setIdentity(kalmanfilter.processNoiseCov, Scalar::all(1e-3));  // Q
//    setIdentity(kalmanfilter.processNoiseCov(cv::Rect(3,3,3,3)), Scalar::all(2));
//    setIdentity(kalmanfilter.measurementNoiseCov, Scalar::all(3e-2));   // R
//    setIdentity(kalmanfilter.measurementNoiseCov(cv::Rect(3,3,3,3)), Scalar::all(2.0));
//    setIdentity(kalmanfilter.errorCovPost, Scalar::all(1));   // P
//    rng.fill(kalmanfilter.statePost, RNG::NORMAL, 0.0, 1.0);

//    cv::Mat measurement(6,1, CV_32F);

    for(int i=0; i<R_vec.size(); i++)
    {
        cv::Mat R;
        cv::Vec3f t;
        R = R_vec[i];
        t = t_vec[i];

//        cv::Mat r;
//        cv::Rodrigues(R, r);
//        measurement.at<float>(0,0) = r.at<float>(0,0);
//        measurement.at<float>(1,0) = r.at<float>(1,0);
//        measurement.at<float>(2,0) = r.at<float>(2,0);
//        measurement.at<float>(3,0) = t[0];
//        measurement.at<float>(4,0) = t[1];
//        measurement.at<float>(5,0) = t[2];

//        // 更新卡尔曼滤波器
//        kalmanfilter.predict();
//        kalmanfilter.correct(measurement);
//        cv::Mat updated_status = kalmanfilter.statePost;
//        r.at<float>(0,0) = updated_status.at<float>(0,0);
//        r.at<float>(1,0) = updated_status.at<float>(1,0);
//        r.at<float>(2,0) = updated_status.at<float>(2,0);
//        t[0] = updated_status.at<float>(3,0);
//        t[1] = updated_status.at<float>(4,0);
//        t[2] = updated_status.at<float>(5,0);
//        cv::Rodrigues(r, R);

        cv::Mat Rwc = R.t();
        cv::Mat tcw = (cv::Mat_<float>(3,1)<< t[0], t[1], t[2]);
        cv::Mat twc = -R * tcw;
        cv::Vec3f t_wc = cv::Vec3f(twc.at<float>(0,0), twc.at<float>(1,0), twc.at<float>(2,0));

        cv::Affine3f cam_pose = cv::Affine3f(Rwc, t_wc);
        pose_visualize.setViewerPose(cam_pose);
        pose_visualize.spinOnce(30, false);
    }

    return 0;
}


int ReadData(string full_file_name, vector<cv::Mat>& R_vec, vector<Vec3f> &t_vec)
{
    ifstream fin(full_file_name);
    if(!fin)
        cerr << "Cannot find file: " << full_file_name << endl;
    stringstream ss;
    string temp1, temp2;
    while(!fin.eof())
    {
        int index;
        float timestampt;
        float R_data[9];
        float t_data[3];

        // 数据格式：序号 时间戳 [R] [t]
        fin >> index;
        fin >> timestampt;

        // 处理第一个中括号
        fin >> temp1;
        temp2 = temp1.substr(1, temp1.size()-1);
        ss << temp2;

        ss >> R_data[0];
        ss.clear();
        ss.str("");

        for(int i=1; i<8; i++)
            fin >> R_data[i];

        // 处理第二个中括号
        fin >> temp1;
        temp2 = temp1.substr(0, temp1.size()-1);
        ss << temp2;
        ss >> R_data[8];
        ss.clear();
        ss.str("");

        //处理第三个中括号
        fin >> temp1;
        temp2 = temp1.substr(1, temp1.size()-1);
        ss << temp2;
        ss >> t_data[0];
        ss.clear();
        ss.str("");

        fin >> t_data[1];

        // 处理第四个中括号
        fin >> temp1;
        temp2 = temp1.substr(0, temp1.size()-1);
        ss << temp2;
        ss >> t_data[2];
        ss.clear();
        ss.str("");

        fin >> temp1; // 最后一个多余的字符


        cv::Mat R = (cv::Mat_<float>(3,3) <<
                     R_data[0], R_data[1], R_data[2],
                     R_data[3], R_data[4], R_data[5],
                     R_data[6], R_data[7], R_data[8]
                );
        cv::Vec3f t = cv::Vec3f(t_data[0], t_data[1], t_data[2]);
        R_vec.push_back(R);
        t_vec.push_back(t);

        if(!fin.good()) break;
    }

    ofstream outfile("x_vec_huawei.txt", ios_base::out);
    for(int i=0; i<t_vec.size(); i++)
    {
        outfile << t_vec[i][0] << endl;
    }
    outfile.close();

    fin.close();
}
