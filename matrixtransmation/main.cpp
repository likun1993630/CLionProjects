#include <iostream>

#include <cmath>
using namespace std;
#include <Eigen/Core>
#include <Eigen/Geometry>
int main( int argc, char** argv )
{
    Eigen::Vector3d P0;
    Eigen::Vector3d P1(0.5, -0.1, 0.2);
    Eigen::Vector3d P2;
    Eigen::Isometry3d Tc1w = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d Tc2w = Eigen::Isometry3d::Identity();
    Eigen::Quaterniond q1(0.55, 0.30, 0.20, 0.2);
    Eigen::Vector3d t1(0.7, 1.1, 0.2);
    Eigen::Quaterniond q2(-0.1, 0.3, -0.7, 0.2);
    Eigen::Vector3d t2(-0.1, 0.4, 0.8);
    //归一化
    cout << "q1=\n" << q1.coeffs().transpose() << "\n" << endl;
    cout << "q2=\n" << q2.coeffs().transpose() << "\n" << endl;
    Eigen::Quaterniond q1n = q1.normalized();
    cout << "q1归一化, q1n=\n" << q1n.coeffs().transpose() << "\n" << endl;
    Eigen::Quaterniond q2n = q2.normalized();
    cout << "q2归一化, q2n=\n" << q2n.coeffs().transpose()<< "\n" << endl;
    //求一号萝卜变换矩阵
    Eigen::Matrix3d q1_m = q1n.toRotationMatrix();
    cout << "q1对应的旋转矩阵q1_m=\n" << q1_m << "\n" << endl;
    Tc1w.rotate(q1_m);
    Tc1w.pretranslate(t1);
    cout << "变换矩阵Tc1w=\n" << Tc1w.matrix() << "\n" << endl;
    //求二号萝卜变换矩阵
    Eigen::Matrix3d q2_m = q2n.toRotationMatrix();
    cout << "q2对应的旋转矩阵q2_m=\n" << q2_m << "\n" << endl;
    Tc2w.rotate(q2_m);
    Tc2w.pretranslate(t2);
    cout << "变换矩阵Tc2w=\n" << Tc2w.matrix() << "\n" << endl;
    //求二号萝卜观察该点的位置
    P0 = Tc1w.inverse()*P1;
    cout<< "该点在世界坐标系的坐标位置P0为:\n"<<P0.transpose()<<"\n"<<endl;
    P2 = Tc2w*P0;
    cout<< "该点在二号萝卜的坐标位置P2为:\n"<<P2.transpose()<<"\n"<<endl;
    return 0;
}