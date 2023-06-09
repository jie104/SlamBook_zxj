//
// Created by zxj on 2023/5/26.
//
#include <opencv2/opencv.hpp>
#include <string>
#include <cmath>

std::string image_file="/home/zxj/桌面/Study/SlamBook_zxj/ch5/imageBasic/distorted.png";    //确保路径正确

int main(int argc,char **argv){
    //本程序实现去畸变部分的代码，尽管可以调用Opencv去畸变
    //畸变参数
    double k1=-0.28340811,k2=0.0739,p1=0.00019359,p2=1.76187114e-05;
    //内参
    double fx=458.654,fy=457.296,cx=367.215,cy=248.375;

    cv::Mat image=cv::imread(image_file,0); //图像是灰度图，CV_8UC1
    int rows=image.rows,cols=image.cols;
    cv::Mat image_undistort=cv::Mat(rows,cols,CV_8UC1); //去畸变后的图

    //计算去畸变后图像的内容
    for(int v=0;v<rows;v++){
        for (int u=0;u<cols;u++){
            //按照公式，计算(u,v)对应到畸变图像中的坐标(u_distorted,v_distorted)
            double x=(u-cx)/fx,y=(v-cy)/fy;
            double r=std::sqrt(x*x+y*y);
            double x_distorted=x*(1+k1*r*r+k2*r*r*r*r)+2*p1*x*y+
                    p2*(r*r+2*x*x);
            double y_distorted=y*(1+k1*r*r+k2*r*r*r*r)+p1*(r*r+2*y*y)
                    +2*p2*x*y;
            double u_distorted=fx*x_distorted+cx;
            double v_distorted=fy*y_distorted+cy;
//            p2*(r*r+2*x*x);

            //赋值（最近邻插值）
            if(u_distorted >=0 && v_distorted >=0 && u_distorted < cols
                && v_distorted <rows){
                image_undistort.at<u_char >(v,u)=image.at<u_char >((int) v_distorted,
                                                                   (int)u_distorted);
            }else{
                image_undistort.at<u_char >(v,u)=0;
            }

        }
    }

    //画出去畸变后的图像
    cv::imshow("distorted",image);
    cv::imshow("undistorted",image_undistort);
    cv::waitKey();
    return 0;

}

