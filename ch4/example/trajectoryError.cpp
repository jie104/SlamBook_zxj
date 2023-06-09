//
// Created by zxj on 2023/5/18.
//

#include <iostream>
#include <fstream>
#include <unistd.h>
#include <pangolin/pangolin.h>
#include <sophus/se3.h>

std::string groundtruth_file="/home/zxj/桌面/Study/SlamBook_zxj/ch4/example/groundtruth.txt";
std::string estimated_file="/home/zxj/桌面/Study/SlamBook_zxj/ch4/example/estimated.txt";

typedef std::vector<Sophus::SE3,Eigen::aligned_allocator<Sophus::SE3>> TrajectoryType;

void DrawTrajectory(const TrajectoryType &gt,const TrajectoryType &esti);
//void DrawTrajectory(TrajectoryType &poses);

TrajectoryType ReadTrajectory(const std::string &path);

int main(int argc,char **argv){
    TrajectoryType groundtruth= ReadTrajectory(groundtruth_file);
    TrajectoryType estimated= ReadTrajectory(estimated_file);
    assert(!groundtruth.empty() && !estimated.empty());
    assert(groundtruth.size()==estimated.size());

    //compute rmse
    double rmse=0;
    for (size_t i=0;i<estimated.size();i++){
        Sophus::SE3 p1=estimated[i],p2=groundtruth[i];
        double error=(p2.inverse()*p1).log().norm();
        rmse+=error*error;
    }

    rmse=rmse/double(estimated.size());
    rmse=std::sqrt(rmse);

    std::cout << "RMSE = " << rmse << std::endl;

    DrawTrajectory(groundtruth,estimated);
//    DrawTrajectory(estimated);


    return 0;
}

TrajectoryType ReadTrajectory(const std::string &path) {
    std::ifstream fin(path);
    TrajectoryType trajectory;
    if (!fin) {
        std::cerr << "trajectory " << path << " not found." << std::endl;
        return trajectory;
    }

    while (!fin.eof()) {
        double time, tx, ty, tz, qx, qy, qz, qw;
        fin >> time >> tx >> ty >> tz >> qx >> qy >> qz >> qw;
        Sophus::SE3 p1(Eigen::Quaterniond(qx, qy, qz, qw), Eigen::Vector3d(tx, ty, tz));
        trajectory.push_back(p1);
    }

}

void DrawTrajectory(const TrajectoryType &poses,const TrajectoryType &poses1) {
    //create pangolin window and plot the tracjectory
    pangolin::CreateWindowAndBind("Trajectory Viewer", 1024, 768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC0_ALPHA, GL_ONE_MINUS_CONSTANT_ALPHA);

    pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
            pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)
    );

    pangolin::View &d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, 0.0, 1.0, -1024.0f / 768.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));

    while (pangolin::ShouldQuit() == false) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        d_cam.Activate(s_cam);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
        glLineWidth(2);
        for (size_t i = 0; i < poses.size(); i++) {
            //画出每个位姿的三个坐标轴
            Eigen::Vector3d Ow = poses[i].translation();
            Eigen::Vector3d Xw = poses[i] * (0.1 * Eigen::Vector3d(1, 0, 0));
            Eigen::Vector3d Yw = poses[i] * (0.1 * Eigen::Vector3d(0, 1, 0));
            Eigen::Vector3d Zw = poses[i] * (0.1 * Eigen::Vector3d(0, 0, 1));
            glBegin(GL_LINES);
            glColor3f(1.0, 0.0, 0.0);
            glVertex3d(Ow[0], Ow[1], Ow[2]);
            glVertex3d(Xw[0], Xw[1], Xw[2]);
            glColor3f(0.0, 1.0, 0.0);
            glVertex3d(Ow[0], Ow[1], Ow[2]);
            glVertex3d(Yw[0], Yw[1], Yw[2]);
            glColor3f(0.0, 0.0, 1.0);
            glVertex3d(Ow[0], Ow[1], Ow[2]);
            glVertex3d(Zw[0], Zw[1], Zw[2]);
            glEnd();
        }

        //画出连线
        for (size_t i = 0; i < poses.size(); i++) {
            glColor3f(0.0, 0.0, 0.0);
            glBegin(GL_LINES);
            auto p1 = poses[i], p2 = poses[i + 1];
            glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
            glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
            glEnd();

        }

        for (size_t i = 0; i < poses1.size(); i++) {
            //画出每个位姿的三个坐标轴
            Eigen::Vector3d Ow = poses1[i].translation();
            Eigen::Vector3d Xw = poses1[i] * (0.1 * Eigen::Vector3d(1, 0, 0));
            Eigen::Vector3d Yw = poses1[i] * (0.1 * Eigen::Vector3d(0, 1, 0));
            Eigen::Vector3d Zw = poses1[i] * (0.1 * Eigen::Vector3d(0, 0, 1));
            glBegin(GL_LINES);
            glColor3f(1.0, 0.0, 0.0);
            glVertex3d(Ow[0], Ow[1], Ow[2]);
            glVertex3d(Xw[0], Xw[1], Xw[2]);
            glColor3f(0.0, 1.0, 0.0);
            glVertex3d(Ow[0], Ow[1], Ow[2]);
            glVertex3d(Yw[0], Yw[1], Yw[2]);
            glColor3f(0.0, 0.0, 1.0);
            glVertex3d(Ow[0], Ow[1], Ow[2]);
            glVertex3d(Zw[0], Zw[1], Zw[2]);
            glEnd();
        }

        //画出连线
        for (size_t i = 0; i < poses1.size(); i++) {
            glColor3f(0.0, 0.0, 0.0);
            glBegin(GL_LINES);
            auto p1 = poses1[i], p2 = poses1[i + 1];
            glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
            glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
            glEnd();

        }


        pangolin::FinishFrame();
        usleep(5000);   //sleep 5 ms



        }
    }

