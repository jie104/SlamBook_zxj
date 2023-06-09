//
// Created by zxj on 2023/5/25.
//

#include <iostream>
#include <chrono>

//#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

int main(int argc,char **argv){
    //读取argv[1]指定的图像

    cv::Mat image;
    image=cv::imread(argv[1]);  //cv::imread函数读取指定路径下的图像

    //判断图像文件是否正确读取
    if(image.data== nullptr){   //数据不存在，可能是文件不存在
        std::cerr << "文件" << argv[1] << "不存在." << std::endl;
    }

    //文件顺利读取，首先输出一些基本信息
    std::cout << "图像宽为" << image.cols << ",高为" << image.rows
            << ",通道数为" << image.channels() << std::endl;

    cv::imshow("image",image);  //用cv::imshow显示图像
    cv::waitKey(0); //暂停程序，等待一个按键输入

    //判断image的类型
    if(image.type()!=CV_8UC1 && image.type()!=CV_8UC3){
        //图像类型不符合要求
        std::cout << "请输入一张彩色图或灰度图 " << std::endl;
        return 0;
    }

    //遍历图像，请注意以下遍历方式也可使用与随机像素访问
    //使用std::chrono给算法计时
    std::chrono::steady_clock::time_point  t1=std::chrono::steady_clock::now();
    for (std::size_t y=0;y< image.rows;y++){
        //使用cv::Mat::ptr获得图像的行指针
        unsigned char *row_ptr=image.ptr<unsigned char>(y); //row_ptr是第y行的头指针
        for(std::size_t x=0;x<image.cols;x++){
            //访问位于x,y处的像素
            unsigned char* data_ptr=&row_ptr[x*image.channels()];    //data_ptr指向待访问的像素数据
            //输出该像素的每个通道，如果是灰度图就只有一个通道
            for(int c=0;c!=image.channels();c++){
                unsigned char data=data_ptr[c]; //data为I(x,y)第c个通道的值
            }
        }
    }
    std::chrono::steady_clock::time_point t2=std::chrono::steady_clock::now();
    std::chrono::duration<double> time_used=std::chrono::
            duration_cast<std::chrono::duration<double>>(t2-t1);
    std::cout << "遍历图像用时： " << time_used.count() << "秒。" << std::endl;

    //关于cv::Mat的拷贝
    //直接赋值并不会拷贝数据
    cv::Mat image_another=image;
    //修改image_another会导致image发生变化
    image_another(cv::Rect(0,0,100,100)).setTo(0);  //将左上角100*100的块置0
    cv::imshow("image",image);
    cv::waitKey(0);

    //使用clone拷贝
    cv::Mat image_clone=image.clone();
    image_clone(cv::Rect(0,0,100,100)).setTo(255);
    cv::imshow("image",image);
    cv::imshow("image_clone",image_clone);
    cv::waitKey(0);

    cv::destroyAllWindows();
    return 0;
}