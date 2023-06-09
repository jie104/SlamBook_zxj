//
// Created by zxj on 2023/5/9.
//

#include <iostream>
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry>

//本程序演示Eigen几何模块的使用方法

int main(int argc,char **argv){
    //Eigen/Geometry模块提供了各种旋转和平移的表示
    //3D旋转矩阵直接使用Matrix3d或Matrix3f
    Eigen::Matrix3d rotation_matrix=Eigen::Matrix3d::Identity();
    //旋转向量使用AngleAxis，它底层不直接是Matrix，但运算可以当做矩阵
    Eigen::AngleAxisd rotation_vector(M_PI/4,Eigen::Vector3d(0,0,1));   //沿着z轴旋转45°
    std::cout.precision(3);
    //用matrix()转换成矩阵
    std::cout << "rotation matrix =\n" <<rotation_vector.matrix() << std::endl;
    std::cout << "matrix3d =\n" <<rotation_matrix << std::endl;

    rotation_matrix=rotation_vector.toRotationMatrix();
    std::cout << "rotation matrix =\n" <<rotation_matrix << std::endl;

    //用AngleAxis可以进行坐标变换
    Eigen::Vector3d v(1,0,0);
    Eigen::Vector3d v_rotated=rotation_vector*v;
    std::cout << "(1,0,0) after rotation (by angle axis) = "
                << v_rotated.transpose() << std::endl;

    //或者用旋转矩阵
    v_rotated=rotation_matrix*v;
    std::cout << "(1,0,0) after rotation (by matrix) = "
                << v_rotated.transpose() << std::endl;

    //欧拉角：可以用旋转矩阵直接转换成欧拉角
    //ZYX顺序，即roll pitch yaw 顺序
    Eigen::Vector3d euler_angles=rotation_matrix.eulerAngles(2,1,0);
    std::cout << "yaw pitch roll = " << euler_angles.transpose() << std::endl;


    //欧氏变换矩阵使用Eigen::Isometry
    Eigen::Isometry3d T=Eigen::Isometry3d::Identity();  //虽然称为3d,实质上是4*4矩阵
    T.rotate(rotation_vector);  //按照rotatio_vector进行旋转
    T.pretranslate(Eigen::Vector3d(1,3,4)); //把平移向量设成（1,3,4）
    std::cout << "Transform matrix = \n" << T.matrix() << std::endl;

    //用变换矩阵进行坐标变换
    Eigen::Vector3d v_transformed=T*v;
    std::cout <<"v transform = " << v_transformed.transpose() << std::endl;

    //对于仿射变换和射影变换，使用Eigen::Affine3d和Eigen::Projective3d即可

    //四元数
    //可以直接把AngleAxis赋值给四元数，反之亦然
    Eigen::Quaterniond q=Eigen::Quaterniond(rotation_vector);
    std::cout << "quaternion from rotation vector = " << q.coeffs().transpose() << std::endl;

    //请注意coeffs的顺序是（x,y,z）,w为实部，前三者为虚部，也可以把旋转矩阵赋予它
    q=Eigen::Quaterniond(rotation_matrix);
    std::cout << "quaternion from rotation vector = " << q.coeffs().transpose() << std::endl;

    //使用四元数旋转一个向量，使用重载的乘法即可
    v_rotated=q*v;  //注意数学上是qvq^{-1}
    std::cout << "(1,0,0) after rotation = " << v_rotated.transpose() << std::endl;

    //用常规向量乘法表示，则计算如下
    std::cout <<"should be equal to " <<(q*Eigen::Quaterniond(0,1,0,0)*q.inverse()).coeffs().transpose()
            << std::endl;
}