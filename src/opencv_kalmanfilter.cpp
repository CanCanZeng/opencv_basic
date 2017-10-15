#include "opencv_kalmanfilter.h"

#include <stdio.h>

const int win_height = 600;
const int win_width = 800;

Point mouse_position = Point(win_width >> 1, win_height >> 1);


// mouse event callback
void MouseEvent(int event, int x, int y, int flag, void *parm)
{
    if(event == CV_EVENT_MOUSEMOVE)
    {
        mouse_position = Point(x,y);
    }
}

int main(void)
{
    RNG rng;
    // 1. kalman filter setup
    const int state_num = 4;                // 状态值 4x1
    const int measure_num = 2;              // c测量值 2x1
    KalmanFilter kalmanfilter(state_num, measure_num, 0);

    kalmanfilter.transitionMatrix = (Mat_<float>(4,4) <<
                                     1.0f, 0.0f, 1.0f, 0.0f,
                                     0.0f, 1.0f, 0.0f, 1.0f,
                                     0.0f, 0.0f, 1.0f, 0.0f,
                                     0.0f, 0.0f, 0.0f, 1.0f
                                     );

    setIdentity(kalmanfilter.measurementMatrix);            // 测量矩阵 H
    cout << kalmanfilter.measurementMatrix << endl;
    setIdentity(kalmanfilter.processNoiseCov, Scalar::all(1e-5));   // 系统噪声协方差矩阵 Q
    setIdentity(kalmanfilter.processNoiseCov(cv::Rect(2,2,2,2)), Scalar::all(1e-1));
    setIdentity(kalmanfilter.measurementNoiseCov, Scalar::all(2));   // 测量噪声协方差矩阵 R
    setIdentity(kalmanfilter.errorCovPost, Scalar::all(1));             // 后验误差估计协方差矩阵
    // 初始状态值
    rng.fill(kalmanfilter.statePost, RNG::UNIFORM, 0, win_height > win_width ? win_width : win_height);
    Mat measurement = Mat::zeros(measure_num, 1, CV_32F);       // 初始测量值 x'(0), 因为后面要更新这个值，所以必须先定义

    namedWindow("kalman");
    setMouseCallback("kalman", MouseEvent);

    Mat image(win_height, win_width, CV_8UC3, Scalar(0));

    while(1)
    {
        // 2. kalman prediction
        Mat prediction = kalmanfilter.predict();
        Point predict_pt = Point(prediction.at<float>(0), prediction.at<float>(1));

        // 3. update measurement
        randn(measurement, 0, 2);
        measurement.at<float>(0) += (float)mouse_position.x;
        measurement.at<float>(1) += (float)mouse_position.y;
        Point meas_pt = Point(measurement.at<float>(0), measurement.at<float>(1));

        // 4. update
        kalmanfilter.correct(measurement);
        Mat updated_status = kalmanfilter.statePost;
        Point updated_pt = Point(updated_status.at<float>(0), prediction.at<float>(1));

        cout << "系统状态向量：" << endl << kalmanfilter.statePost << endl;

        // draw
        image.setTo(Scalar(255, 255, 255, 0));
        circle(image, updated_pt, 5,Scalar(0, 255, 0), 3);      // updated point with green
        circle(image, meas_pt, 5, Scalar(255, 0, 0), 3); // current position with red

        char buf[255];
        snprintf(buf, 256, "updated position: (%3d, %3d)", updated_pt.x, updated_pt.y);
        putText(image, buf, Point(10,30), CV_FONT_HERSHEY_SCRIPT_COMPLEX, 1, Scalar(0,0,0), 1, 8);
        snprintf(buf, 256, "measured position: (%3d, %3d)", meas_pt.x, meas_pt.y);
        putText(image, buf, Point(10,60), CV_FONT_HERSHEY_SCRIPT_COMPLEX, 1, Scalar(0,0,0), 1, 8);

        imshow("kalman", image);
        int key = waitKey(3);
        if(key == 27 || key == 'q' || key == 'Q')
            break;
    }
}
