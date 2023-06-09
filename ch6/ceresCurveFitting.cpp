//
// Created by zxj on 2023/6/5.
//

#include <iostream>
#include <opencv2/core/core.hpp>
#include <ceres/ceres.h>
#include<chrono>

///代价函数的计算模型

struct CURVE_FITTING_COST{
    CURVE_FITTING_COST(double x,double y):x_(x),y_(y){

    }

    //残差计算
    template<class T>
    bool operator() (
        const T* const abc, //参数模型，有3维
        T* residual) const {
        //y-exp(ax^2+bx+x)
        residual[0]=T(y_)-ceres::exp(abc[0]*T(x_)*T(x_)+abc[1]*T(x_)+abc[2]);
//        LOG(INFO) << "已调用运算符!!! ";
        return true;
    }

    const double x_,y_; //x,y数据

};


int main(int argc,char **){
    double ar=1.0,br=2.0,cr=1.0;    //真实参数
    double ae=2.0,be=-1.0,ce=5.0;   //估计参数值,相当于初值
    int N=100;  //数据点
    double w_sigma=1.0; //噪声sigma值
    double inv_sigma=1.0/w_sigma;
    cv::RNG rng;    //Opencv随机产生器

    std::vector<double> x_data,y_data;  //数据
    for(int i=0;i<N;i++){
        double x=i/100.0;
        x_data.push_back(x);
        double y= exp(ar*x*x+br*x+cr)+rng.gaussian(w_sigma*w_sigma);
        y_data.push_back(y);
    }

    double abc[3]={ae,be,ce};

    //构建最小二问题
    ceres::Problem problem;
    for (int i=0;i<N;i++){
        problem.AddResidualBlock(   //向问题添加误差项
        //使用自动求导，模版参数：误差类型、输出维度、输入维度，维数要与之前struct一致
        new ceres::AutoDiffCostFunction<CURVE_FITTING_COST,1,3>(
                new CURVE_FITTING_COST(x_data[i],y_data[i])
                ),
            nullptr,        //核函数，这里不使用，为空
            abc                         //待估计参数
        );
    }

    //配置求解器
    ceres::Solver::Options options; //这里有很多配置项可以填
    options.linear_solver_type=ceres::DENSE_NORMAL_CHOLESKY;      //增量方程如何求解
    options.minimizer_progress_to_stdout= true; //输出到cout

    ceres::Solver::Summary summary;     //优化信息
    std::chrono::steady_clock::time_point t1=std::chrono::steady_clock::now();
    ceres::Solve(options,&problem,&summary);   //开始优化
    std::chrono::steady_clock::time_point t2=std::chrono::steady_clock::now();
    std::chrono::duration<double> time_used=
            std::chrono::duration_cast<std::chrono::duration<double>>(t2-t1);
    std::cout << "solve time cost = " << time_used.count() << " seconds. " << std::endl;

    //输出结果
    std::cout << summary.BriefReport() << std::endl;
    std::cout << "estimated a,b,c = ";
    for (auto a:abc) std::cout << a << " ";
    std::cout << std::endl;

    return 0;



}
