//
// Created by xin on 28/10/2020.
//

#ifndef SWING_TRAJECTORY_SWING_TRAJECTORY_H
#define SWING_TRAJECTORY_SWING_TRAJECTORY_H

#include "iostream"
#include <cstdio>
#include <cstdlib>
#include <vector>
#include "spline.h"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>


void test_spline(){
    std::vector<double> X {0.0, 0.4, 1.2, 1.8, 2.0};
    std::vector<double> Y {0.1, 0.7, 0.6, 1.1, 0.9};

    tk::spline spline;
    spline.set_boundary(tk::spline::first_deriv, 0.0,tk::spline::first_deriv, 0.5, false);
    spline.set_points(X,Y);    // currently it is required that X is already sorted

    double x=0.5;
    printf("spline at %f is %f\n", x, spline(x));
    printf("spline first deviv at %f is %f\n", x, spline.deriv(1,x));
    printf("spline second deviv at %f is %f\n", x, spline.deriv(2,x));

}

class CubicSpline1D{
private:
    tk::spline spline_;
public:
    CubicSpline1D(const std::vector<double>& t, const std::vector<double>& x, const double left_first_deriv=0.0, const double right_first_deriv=0.0){
        spline_.set_boundary(tk::spline::first_deriv, left_first_deriv,tk::spline::first_deriv, right_first_deriv, false);
        spline_.set_points(t, x);
    }

    double pos(double t){
        return spline_(t);
    }

    double vel(double t){
        return spline_.deriv(1,t);
    }

    double acc(double t){
        return spline_.deriv(2,t);
    }
};





class SwingTrajectory3D{
private:
    Eigen::Isometry3d ini_pose_;
    Eigen::Isometry3d fin_pose_;
    Eigen::Vector3d ini_pos_;
    Eigen::Vector3d fin_pos_;
    Eigen::Vector3d ini_vel_;
    Eigen::Vector3d fin_vel_;
    Eigen::Matrix3d ini_rot_;
    Eigen::Matrix3d fin_rot_;

    double swing_duration_;
    double swing_height_;

    tk::spline spline_x_;
    tk::spline spline_y_;
    tk::spline spline_z_;


public:
    SwingTrajectory3D(){};

    SwingTrajectory3D(const Eigen::Vector3d ini_pos, const Eigen::Vector3d fin_pos, const double swing_duration, const double swing_height, const Eigen::Vector3d ini_vel=Eigen::Vector3d(0,0,0), const Eigen::Vector3d fin_vel=Eigen::Vector3d(0,0,0)){
        ini_pos_ = ini_pos;
        fin_pos_ = fin_pos;
        ini_vel_ = ini_vel;
        fin_vel_ = fin_vel;
        ini_rot_ = Eigen::Matrix3d::Identity();
        fin_rot_ = Eigen::Matrix3d::Identity();
        swing_duration_ = swing_duration;
        swing_height_ = swing_height;

        Eigen::Vector3d mid_pos = (ini_pos_ + fin_pos_)/2.0;
        mid_pos.z() += swing_height_;

        double ini_time = 0.0;
        double mid_time = swing_duration_/2.0;
        double fin_time = swing_duration_;

        std::vector<double> t={ini_time, mid_time, fin_time};
        std::vector<double> x={ini_pos_(0), mid_pos(0), fin_pos_(0)};
        std::vector<double> y={ini_pos_(1), mid_pos(1), fin_pos_(1)};
        std::vector<double> z={ini_pos_(2), mid_pos(2), fin_pos_(2)};

        spline_x_.set_boundary(tk::spline::first_deriv, ini_vel_(0),tk::spline::first_deriv, fin_vel_(0), false);
        spline_y_.set_boundary(tk::spline::first_deriv, ini_vel_(1),tk::spline::first_deriv, fin_vel_(1), false);
        spline_z_.set_boundary(tk::spline::first_deriv, ini_vel_(2),tk::spline::first_deriv, fin_vel_(2), false);
        spline_x_.set_points(t, x);
        spline_y_.set_points(t, y);
        spline_z_.set_points(t, z);
    }

    SwingTrajectory3D(const Eigen::Isometry3d ini_pose, const Eigen::Isometry3d fin_pose, const double swing_duration, double swing_height, const Eigen::Vector3d ini_vel=Eigen::Vector3d(0,0,0), const Eigen::Vector3d fin_vel=Eigen::Vector3d(0,0,0)){
        ini_pose_ = ini_pose;
        fin_pose_ = fin_pose;
        ini_pos_ = ini_pose_.translation();
        fin_pos_ = fin_pose_.translation();
        ini_vel_ = ini_vel;
        fin_vel_ = fin_vel;
        ini_rot_ = ini_pose_.rotation();
        fin_rot_ = fin_pose_.rotation();
        swing_duration_ = swing_duration;
        swing_height_ = swing_height;

//        Eigen::Vector3d mid_pos = (ini_pos_ + fin_pos_)/2.0;
        Eigen::Vector3d mid1_pos = ini_pos_;
        mid1_pos.z() += swing_height_;
        Eigen::Vector3d mid2_pos = (ini_pos_ + fin_pos_)/2.0;;
        mid2_pos.z() += swing_height_;

        double ini_time = 0.0;
        double mid1_time = swing_duration_/3.0;
        double mid2_time = 2.0*swing_duration_/3.0;
        double fin_time = swing_duration_;

        std::vector<double> t={ini_time, mid1_time, mid2_time, fin_time};
        std::vector<double> x={ini_pos_(0), mid1_pos(0), mid2_pos(0), fin_pos_(0)};
        std::vector<double> y={ini_pos_(1), mid1_pos(1), mid2_pos(1), fin_pos_(1)};
        std::vector<double> z={ini_pos_(2), mid1_pos(2), mid2_pos(2), fin_pos_(2)};

        spline_x_.set_boundary(tk::spline::first_deriv, ini_vel_(0),tk::spline::first_deriv, fin_vel_(0), false);
        spline_y_.set_boundary(tk::spline::first_deriv, ini_vel_(1),tk::spline::first_deriv, fin_vel_(1), false);
        spline_z_.set_boundary(tk::spline::first_deriv, ini_vel_(2),tk::spline::first_deriv, fin_vel_(2), false);
        spline_x_.set_points(t, x);
        spline_y_.set_points(t, y);
        spline_z_.set_points(t, z);

    }

    Eigen::Vector3d pos(double t){
        double px = spline_x_(t);
        double py = spline_y_(t);
        double pz = spline_z_(t);
        return Eigen::Vector3d(px,py,pz);
    }

    Eigen::Vector3d vel(double t){
        double vx = spline_x_.deriv(1, t);
        double vy = spline_y_.deriv(1, t);
        double vz = spline_z_.deriv(1, t);
        return Eigen::Vector3d(vx,vy,vz);
    }

    Eigen::Vector3d acc(double t){
        double ax = spline_x_.deriv(2, t);
        double ay = spline_y_.deriv(2, t);
        double az = spline_z_.deriv(2, t);
        return Eigen::Vector3d(ax,ay,az);
    }

    Eigen::Quaterniond quat(double t){
        return Eigen::Quaterniond(ini_rot_).slerp(t/swing_duration_, Eigen::Quaterniond(fin_rot_));
    }

    Eigen::Matrix3d rot(double t){
        return Eigen::Quaterniond(ini_rot_).slerp(t/swing_duration_, Eigen::Quaterniond(fin_rot_)).matrix();
    }

    Eigen::Isometry3d pose(double t){
        return creatTransformationMatrix(quat(t), pos(t));
    }

    Eigen::VectorXd std2eigen(std::vector<double> std_vec){
        Eigen::VectorXd eigen_vec = Eigen::Map<Eigen::VectorXd>(std_vec.data(), std_vec.size());
        return eigen_vec;
    }

    std::vector<double> eigen2std(Eigen::VectorXd eigen_vec){
        std::vector<double> std_vec(eigen_vec.data(), eigen_vec.data() + eigen_vec.rows() * eigen_vec.cols());
        return std_vec;
    }

    inline Eigen::Isometry3d creatTransformationMatrix(Eigen::Quaterniond const &quat, Eigen::Vector3d const &trans)
    {
        Eigen::Isometry3d T;
        T.setIdentity();
        T = (Eigen::AngleAxisd(quat));
        T.translation() = trans;
        return T;
    }


};

#endif //SWING_TRAJECTORY_SWING_TRAJECTORY_H
