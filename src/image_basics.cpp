#include <iostream>
#include <chrono>

using namespace std;

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

int main(int argc, char** argv)
{
    // 读取 argv[1] 指定的图像
    cv::Mat image;
    image = cv::imread( argv[1] );

    // 判断图像是否正确读取
    if( image.data == nullptr )
    {
        cerr << "文件" << argv[1] << "不存在" << endl;
        return 0;
    }

    // 文件顺利读取，首先输出一些基本信息
    cout << "图像宽度为：" << image.cols << "高度为：" << image.rows
         << "通道数为：" << image.channels() << endl;
    cv::imshow("image",image);
    cv::waitKey(0); // 暂停程序，等待按键输入

    // 判断 image 的类型
    if( image.type() != CV_8UC1 && image.type() != CV_8UC3)
    {
            // 图像类型不符合要求
        cout << "请输入一张彩色图或者灰度图。" << endl;
    }

    // 遍历图像
    // 使用 std::chrono 来给算法计时
    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    for(size_t y = 0; y < image.rows; y++)
    {
        for(size_t x = 0; x < image.cols; x++)
        {
            // 访问位于 x, y 处的像素
            // 用 cv::Mat::ptr 获得行指针
            unsigned char* row_ptr = image.ptr<unsigned char>(y);
            unsigned char* data_ptr = &row_ptr[x*image.channels()];
            // 输出该像素的每个通道，如果是灰度图就只有一个通道
            for(int c= 0; c!=image.channels(); c++)
            {
                unsigned char data = data_ptr[c];
            }
        }
    }
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2-t1);
    cout << "遍历图像用时" << time_used.count() << " 秒" << endl;
    cout << "ok" << endl;
	return 0;
}
