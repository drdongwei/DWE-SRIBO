#ifndef _SRIBO_H
#define _SRIBO_H

#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <Eigen/Core>
#include <ros/ros.h>
#include <math.h>
#include <algorithm>
#include <vector>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <nlink_parser/LinktrackNodeframe3.h>
#include <mavros_msgs/OpticalFlowRad.h>
#include <mavros_msgs/Altitude.h>
#include <sensor_msgs/Imu.h>
#include <string>
#include <fstream>
#include <iostream>

using namespace Eigen;
using namespace std;

class dwe{
    private:
        //private member 
        Eigen::Matrix<double, 6, 6> A;
        Eigen::Matrix<double, 6, 3> B;
        Eigen::Matrix<double, 6, 6> AA,P,Q;
        Eigen::Matrix<double, 4, 4> R;
        Eigen::Matrix<double, 1, 3> Qp,Qv,Pp,Pv;
        Eigen::Matrix<double, 1, 6> tP,tQ;
        Eigen::Matrix<double, 1, 4> tR;

    public:
        dwe(){}
        dwe(int delta,int lopt,int k, Eigen::Vector3d& kappa,double paramP,double paramQ,double paramR,int order,bool enable_flow);
        ~dwe();

        // param in sliding window
        int delta_;
        int lopt_;
        int k_;
        double paramP_;
        double paramQ_;
        double paramR_;

        int order_;
        int count_j_;
        float dt_ = 0.0;
        int LOOP_num_;
        int flow_quality_;

        Eigen::Matrix<double, 1, 3> kappa_;

        bool flag_init_imu_; 
        bool flag_init_ok_ ; 
        bool hasTakenOff_ ; 
        bool enable_flow_;

        //param in estimation
        MatrixXd MHE_height_;
        MatrixXd MHE_flow_;
        MatrixXd MHE_imu_;
        MatrixXd MHE_imu_att_;
        MatrixXd MHE_uwb_;

        MatrixXd VICON_pos01_;
        MatrixXd VICON_pos02_;
        MatrixXd VICON_pos_;
        MatrixXd VICON_vel01_;
        MatrixXd VICON_vel02_;
        MatrixXd VICON_vel_;
        MatrixXd VICON_imu_;
        MatrixXd VICON_uwb_;
        
        double previousTime_ = 0.0;
        double currentTime_ = 0.0;
        MatrixXd VICON_inital_;
        MatrixXd deltaTime_;
        MatrixXd xt_;
        MatrixXd xt_real_;
        MatrixXd x1_;

        int num_state_;
        MatrixXd Ex,Eu,W,t_matrix,ur,xtt;
        MatrixXd ar,xxt_new,xxe,Ex_new;

    public:
        // declaration for functions

        // define the input data form uav
        void flow_cb(const mavros_msgs::OpticalFlowRad::ConstPtr &msg);
        void uwb_cb(const nlink_parser::LinktrackNodeframe3::ConstPtr &msg);
        void imu_new_cb(const sensor_msgs::Imu::ConstPtr &msg);

        // define the input data form vicon
        void pos_pp01_cb(const geometry_msgs::PoseStamped::ConstPtr &msg);
        void pos_pp02_cb(const geometry_msgs::PoseStamped::ConstPtr &msg);
        void vel_pp01_cb(const nav_msgs::Odometry::ConstPtr &msg);
        void vel_pp02_cb(const nav_msgs::Odometry::ConstPtr &msg);

        MatrixXd qua2mat();
        MatrixXd sgolayfilt(Eigen::MatrixXd data);
        MatrixXd meanFilter(Eigen::MatrixXd data);
        void Initialdataxt();
        void solveonce();

};

#endif