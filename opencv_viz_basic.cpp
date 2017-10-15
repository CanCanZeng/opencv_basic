#include "opencv_viz_basic.h"

int main(int argc, char** argv)
{
    // create a window
    cv::viz::Viz3d my_window("Viz Demo");

    // add coordinate axes
    my_window.showWidget("Coordinate Widget", cv::viz::WCoordinateSystem());

    // add line to represent (1,1,1) axis
    cv::viz::WLine axis(Point3f(-1.0f, -1.0f, -1.0f), Point3f(1.0f, 1.0f, 1.0f));
    axis.setRenderingProperty(cv::viz::LINE_WIDTH, 4.0);
    my_window.showWidget("Line Widget", axis);

    // construct a cube widget
    cv::viz::WCube cube_widget(Point3f(0.5, 0.5, 0.5), Point3f(0.0, 0.0, -0.5), true, cv::viz::Color::blue());
    cube_widget.setRenderingProperty(cv::viz::LINE_WIDTH, 4.0);
    // display widget (update if already displayed)
    my_window.showWidget("Cube Widget", cube_widget);

    // Rodrigues vector
   cv::Mat rot_vec =  cv::Mat::zeros(1, 3, CV_32F);
   float translation_phase = 0.0, translation = 0.0;
   while(!my_window.wasStopped())
   {
       // rotation using rodrigues
       // rotate around (1,1,1)
       rot_vec.at<float>(0,0) += CV_PI * 0.01f;
       rot_vec.at<float>(0,1) += CV_PI * 0.01f;
       rot_vec.at<float>(0,2) += CV_PI * 0.01f;
       // shift on (1,1,1)
       translation_phase += CV_PI * 0.01f;
       translation        = sin(translation_phase);
       Mat rot_mat;
       cv::Rodrigues(rot_vec, rot_mat);

       // construct pose
       cv::Affine3f pose(rot_mat, cv::Vec3f(translation, translation, translation));
       my_window.setWidgetPose("Cube Widget", pose);
       my_window.spinOnce(1, true);
   }


    return 0;
}
