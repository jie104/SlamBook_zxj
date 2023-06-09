//
// Created by zxj on 2023/6/6.
//

#include <iostream>
#include <g2o/core/g2o_core_api.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_dogleg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <Eigen/Core>
#include <opencv2/core/core.hpp>
#include <cmath>
#include <chrono>


//曲线模型的顶点，模版参数：优化变量维度和数据维度
class CurveFittingVertex:public g2o::BaseVertex<3,Eigen::Vector3d>{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    //顶点重置函数，这是平凡的，只要将估计值置0即可
    virtual void setToOriginImpl() override{
        _estimate <<0,0,0;
    }

    //更新,处理增量delta_x的计算
    virtual void oplusImpl(const double *update) override{
        _estimate+=Eigen::Vector3d(update);
    }

    //存盘和读盘：留空
    virtual bool read(std::istream &in){}
    virtual bool write(std::ostream &out) const{}
private:

};

//误差模型 模版参数：观测值维度，类型，连接顶点类型
class CurveFittingEdge:public g2o::BaseUnaryEdge<1,double,CurveFittingVertex>{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    CurveFittingEdge(double x):BaseUnaryEdge(),_x(x){}

    //计算曲线模型误差
    virtual void computeError() override{
        const CurveFittingVertex *v=static_cast<const CurveFittingVertex *>
            (_vertices[0]);
        const Eigen::Vector3d abc=v->estimate();
        _error(0,0)=_measurement-
                std::exp(abc(0,0)*_x*_x+abc(1,0)*_x+abc(2,0));

    }

    //计算雅可比矩阵
    virtual void linearizeOplus() override{
        const CurveFittingVertex *v=static_cast<const CurveFittingVertex *>(_vertices[0]);
        const Eigen::Vector3d abc=v->estimate();
        double y=exp(abc[0]*_x*_x+abc[1]*_x+abc[2]);
        _jacobianOplusXi[0]=-_x*_x*y;
        _jacobianOplusXi[1]=-_x*y;
        _jacobianOplusXi[2]=-y;
    }

    virtual bool read(std::istream &in){}
    virtual bool write(std::ostream &out) const {}

public:
    double _x;  //x值，y值为_measurement

};

int main(int argc,char **argv){
    double ar=1.0,br=2.0,cr=1.0;    //真实参数
    double ae=2.0,be=-1.0,ce=5.0;   //估计参数值
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

    //构建图优化，先设定g2o
    //每个误差项优化变量维度为3，误差值维度为1
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<3,1>> BlockSolverType;

    //线性求解器类型
    typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType> LinearSolverType;

    //梯度下降方法，可以从GN、LM、DogLeg中选
    auto solver=new g2o::OptimizationAlgorithmGaussNewton(
            g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));
    g2o::SparseOptimizer optimizer; //图模型
    optimizer.setAlgorithm(solver); //设置求解器
    optimizer.setVerbose(true); //  打开调试输出

    //往图中增加顶点
    CurveFittingVertex *v=new CurveFittingVertex();
    v->setEstimate(Eigen::Vector3d(ae,be,ce));
    v->setId(0);
    optimizer.addVertex(v);

    //往图中增加边
    for(int i=0;i<N;i++){
        CurveFittingEdge *edge=new CurveFittingEdge(x_data[i]);
        edge->setId(i);
        edge->setVertex(0,v);   //设置连接的顶点
        edge->setMeasurement(y_data[i]);    //观测数据
        edge->setInformation(Eigen::Matrix<double,1,1>::Identity()*
                         1/(w_sigma*w_sigma));   //信息矩阵：协方差矩阵之逆
         optimizer.addEdge(edge);
    }

    //执行优化
    std::cout << "start optimization" << std::endl;
    std::chrono::steady_clock::time_point t1=std::chrono::steady_clock::now();
    optimizer.initializeOptimization();
    optimizer.optimize(10);
    std::chrono::steady_clock::time_point t2=std::chrono::steady_clock::now();
    std::chrono::duration<double> time_used=
            std::chrono::duration_cast<std::chrono::duration<double>>(t2-t1);
    std::cout << "solve time cost = " << time_used.count() <<" seconds. " << std::endl;

    //输出优化值
    Eigen::Vector3d abc_estimate =v->estimate();
    std::cout << "estimated model: " << abc_estimate.transpose() << std::endl;
}