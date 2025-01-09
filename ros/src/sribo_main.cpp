#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <Eigen/Core>
#include <vector>
#include <ros/ros.h>
#include <math.h>
#include <vector>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <nlink_parser/LinktrackNodeframe3.h>
#include <mavros_msgs/OpticalFlowRad.h>
#include <mavros_msgs/Altitude.h>
#include <sensor_msgs/Imu.h>
#include <string>
#include "sribo.h"

using namespace Eigen;
using namespace std;    

//Param definition, seen in file "param/sribo_param.yaml"
int delta ;
int lopt;
int k;
double paramP;
double paramQ;
double paramR;
int order, rate;
string dataType;
Eigen::Vector3d kappa;
bool enable_flow;

// output data
ros::Publisher estimate_pose_pub;
ros::Publisher estimate_vel_pub;
ros::Publisher estimate_orientation_pub;

// used data 
ros::Publisher estimate_vicon01_pub;
ros::Publisher estimate_vicon02_pub;
ros::Publisher estimate_uwb_pub;
ros::Publisher estimate_flow_pub;
ros::Publisher estimate_imu_pub;
ros::Publisher estimate_dt_pub;

ros::Publisher real_pose_pub;
ros::Publisher real_vel_pub;
ros::Publisher real_xtest_pub;

// check data
ros::Publisher estimate_height_pub;

/**
 * @brief Read param from "param/dwe_param.yaml"
 */
template<typename T>
void readParam(ros::NodeHandle &nh, std::string param_name, T& loaded_param) {
    // template to read param from roslaunch
    const string& node_name = ros::this_node::getName();
    param_name = "sribo_main/" + param_name;
    nh.getParam(param_name, loaded_param);
    if (!nh.getParam(param_name, loaded_param))
    {
        ROS_ERROR_STREAM("Failed to load " << param_name << ", use default value");
    }
    else{
        ROS_INFO_STREAM("Load " << param_name << " success");
    }
}

void loadRosParams(ros::NodeHandle &nh)
{
    readParam<int>(nh, "delta", delta);
    readParam<int>(nh, "lopt", lopt);
    readParam<int>(nh, "k", k);
    readParam<int>(nh, "rate", rate);

    readParam<bool>(nh, "enable_flow", enable_flow);

    readParam<double>(nh, "paramP", paramP);
    readParam<double>(nh, "paramQ", paramQ);
    readParam<double>(nh, "paramR", paramR);

    readParam<int>(nh, "order", order);
    readParam<string>(nh, "dataType", dataType);

    vector<double> kappa_vec(3);
    readParam<vector<double>>(nh, "kappa", kappa_vec);
    for (int i = 0; i < 3; i++) {
        kappa(i) = kappa_vec[i];
    }
}

/**
 * @brief pub estimated pose and vel
 */
// estimated xt_real
void PubPoseXt(MatrixXd _estimate_pose ,dwe *filter){
    geometry_msgs::PoseStamped xt_pose_msg;
    xt_pose_msg.header.stamp = ros::Time::now();  
    xt_pose_msg.pose.position.x = _estimate_pose(0,filter->LOOP_num_-1);
    xt_pose_msg.pose.position.y = _estimate_pose(1,filter->LOOP_num_-1);
    xt_pose_msg.pose.position.z = _estimate_pose(2,filter->LOOP_num_-1);
    estimate_pose_pub.publish(xt_pose_msg); 
}

// estimated vel_real
void PubVelXt(MatrixXd _estimate_vel ,dwe *filter){
    geometry_msgs::PoseStamped xt_vel_msg;
    xt_vel_msg.header.stamp = ros::Time::now(); 
    xt_vel_msg.pose.position.x = _estimate_vel(3,filter->LOOP_num_-1);
    xt_vel_msg.pose.position.y = _estimate_vel(4,filter->LOOP_num_-1);
    xt_vel_msg.pose.position.z = _estimate_vel(5,filter->LOOP_num_-1);
    estimate_vel_pub.publish(xt_vel_msg); 
} // vel_msg = xt_real_


// pub data for px4: IMU_orientation
void PubOrientationXt(MatrixXd _estimate_orientation ,dwe *filter){
    geometry_msgs::PoseStamped xt_orientation_msg;
    xt_orientation_msg.header.stamp = ros::Time::now(); 
    xt_orientation_msg.pose.orientation.w = _estimate_orientation(0,filter->lopt_);
    xt_orientation_msg.pose.orientation.x = _estimate_orientation(1,filter->lopt_);
    xt_orientation_msg.pose.orientation.y = _estimate_orientation(2,filter->lopt_);
    xt_orientation_msg.pose.orientation.z = _estimate_orientation(3,filter->lopt_);
    estimate_orientation_pub.publish(xt_orientation_msg); 
}

/**
 * @brief pub MHE data
 */
// used data : vicon_pos_01 : dynamic uav
void PubVicon01Xt(MatrixXd _estimate_vicon01 ,dwe *filter){
    geometry_msgs::PoseStamped xt_vicon01_msg;
    xt_vicon01_msg.header.stamp = ros::Time::now();  
    xt_vicon01_msg.pose.position.x = _estimate_vicon01(0,filter->lopt_);
    xt_vicon01_msg.pose.position.y = _estimate_vicon01(1,filter->lopt_);
    xt_vicon01_msg.pose.position.z = _estimate_vicon01(2,filter->lopt_);
    estimate_vicon01_pub.publish(xt_vicon01_msg); 
}

// used data : vicon_pos_02 : static uav
void PubVicon02Xt(MatrixXd _estimate_vicon02 ,dwe *filter){
    geometry_msgs::PoseStamped xt_vicon02_msg;
    xt_vicon02_msg.header.stamp = ros::Time::now();  
    xt_vicon02_msg.pose.position.x = _estimate_vicon02(0,filter->lopt_);
    xt_vicon02_msg.pose.position.y = _estimate_vicon02(1,filter->lopt_);
    xt_vicon02_msg.pose.position.z = _estimate_vicon02(2,filter->lopt_);
    estimate_vicon02_pub.publish(xt_vicon02_msg); 
}

// used data : MHE_uwb
void Pubuwb(MatrixXd _estimate_uwb ,dwe *filter){
    geometry_msgs::PoseStamped uwb_msg;
    uwb_msg.header.stamp = ros::Time::now(); 
    uwb_msg.pose.position.x = _estimate_uwb(0,filter->lopt_);
    estimate_uwb_pub.publish(uwb_msg); 
}

// used data : MHE_flow
void Pubflow(MatrixXd _estimate_flow ,dwe *filter){
    geometry_msgs::PoseStamped flow_msg;
    flow_msg.header.stamp = ros::Time::now(); 
    flow_msg.pose.position.x = _estimate_flow(0,filter->lopt_);
    flow_msg.pose.position.y = _estimate_flow(1,filter->lopt_);
    flow_msg.pose.position.z = _estimate_flow(2,filter->lopt_);
    estimate_flow_pub.publish(flow_msg); 
}

// used data : MHE_imu
void Pubimu(MatrixXd _estimate_imu ,dwe *filter){
    geometry_msgs::PoseStamped imu_msg;
    imu_msg.header.stamp = ros::Time::now(); 
    imu_msg.pose.position.x = _estimate_imu(0,filter->lopt_-1);
    imu_msg.pose.position.y = _estimate_imu(1,filter->lopt_-1);
    imu_msg.pose.position.z = _estimate_imu(2,filter->lopt_-1);
    estimate_imu_pub.publish(imu_msg); 
}

// used data : MHE_dt
void Pubdt(float _estimate_dt,float _estimate_dt_solveonce ,dwe *filter){
    geometry_msgs::PoseStamped dt_msg;
    dt_msg.header.stamp = ros::Time::now(); 
    dt_msg.pose.position.x = _estimate_dt;
    dt_msg.pose.position.y = _estimate_dt_solveonce;
    estimate_dt_pub.publish(dt_msg); 
}

/**
 * @brief pub relative data
 */
// relative data : VICON_pos
void PubRelativePos(MatrixXd _relative_pos ,dwe *filter){
    geometry_msgs::PoseStamped relativePos_msg;
    relativePos_msg.header.stamp = ros::Time::now(); 
    relativePos_msg.pose.position.x = _relative_pos(0,filter->lopt_);
    relativePos_msg.pose.position.y = _relative_pos(1,filter->lopt_);
    relativePos_msg.pose.position.z = _relative_pos(2,filter->lopt_);
    real_pose_pub.publish(relativePos_msg); 
}

// relative data : VICON_vel
void PubRelativeVel(MatrixXd _relative_vel ,dwe *filter){
    geometry_msgs::PoseStamped relativeVel_msg;
    relativeVel_msg.header.stamp = ros::Time::now(); 
    relativeVel_msg.pose.position.x = _relative_vel(0,filter->lopt_);
    relativeVel_msg.pose.position.y = _relative_vel(1,filter->lopt_);
    relativeVel_msg.pose.position.z = _relative_vel(2,filter->lopt_);
    real_vel_pub.publish(relativeVel_msg); 
}

// relative data : xt
void PubRelativeXt(MatrixXd _relative_xt ,dwe *filter){
    geometry_msgs::PoseStamped relativeXt_msg;
    relativeXt_msg.header.stamp = ros::Time::now(); 
    relativeXt_msg.pose.position.x = _relative_xt(0,filter->LOOP_num_-1);
    relativeXt_msg.pose.position.y = _relative_xt(1,filter->LOOP_num_-1);
    relativeXt_msg.pose.position.z = _relative_xt(2,filter->LOOP_num_-1);
    real_xtest_pub.publish(relativeXt_msg); 
}

/**
 * @brief pub check data
 */
// check data : flow_distance
void Pubheight(MatrixXd _estimate_height ,dwe *filter){
    geometry_msgs::PoseStamped height_msg;
    height_msg.header.stamp = ros::Time::now(); 
    height_msg.pose.position.x = _estimate_height(0,filter->lopt_);
    estimate_height_pub.publish(height_msg); 
}


int main (int argc, char** argv) 
{
    // ros init
    ros::init(argc, argv, "dwe", ros::init_options::AnonymousName);
    ros::NodeHandle nh;

    // load param
    loadRosParams(nh);
    ROS_INFO("**********START ROS NODE**********");
    // define dwe_filter and load param
    dwe dwe_filter(delta,lopt,k,kappa,paramP,paramQ,paramR,order,enable_flow);
    
    // data callback from uav
    ros::Subscriber flow_sub = nh.subscribe<mavros_msgs::OpticalFlowRad>("/mavros/px4flow/raw/optical_flow_rad",1,boost::bind(&dwe::flow_cb, &dwe_filter, _1));
    ros::Subscriber uwb_sub = nh.subscribe<nlink_parser::LinktrackNodeframe3>("/nlink_linktrack_nodeframe3",1,boost::bind(&dwe::uwb_cb, &dwe_filter, _1));
    ros::Subscriber imu_new_sub = nh.subscribe<sensor_msgs::Imu>("/IMU_data",1,boost::bind(&dwe::imu_new_cb, &dwe_filter, _1));
    
    // data callback from vicon, 01 is static anchor
    ros::Subscriber pos_pp01_sub = nh.subscribe<geometry_msgs::PoseStamped>("/vicon_UAV/CSJ01/pose",1,boost::bind(&dwe::pos_pp01_cb, &dwe_filter, _1));
    ros::Subscriber pos_pp02_sub = nh.subscribe<geometry_msgs::PoseStamped>("/vicon_UAV/CSJ02/pose",1,boost::bind(&dwe::pos_pp02_cb, &dwe_filter, _1));
    ros::Subscriber vel_pp01_sub = nh.subscribe<nav_msgs::Odometry>("/vicon_UAV/CSJ01/odom",1,boost::bind(&dwe::vel_pp01_cb, &dwe_filter, _1));
    ros::Subscriber vel_pp02_sub = nh.subscribe<nav_msgs::Odometry>("/vicon_UAV/CSJ02/odom",1,boost::bind(&dwe::vel_pp02_cb, &dwe_filter, _1));

    // pub : check -- use for real exp and check
    estimate_pose_pub = nh.advertise<geometry_msgs::PoseStamped>(dataType + "/xt/pose", 1);
    estimate_vel_pub = nh.advertise<geometry_msgs::PoseStamped>(dataType + "/xt/vel", 1);
    estimate_orientation_pub = nh.advertise<geometry_msgs::PoseStamped>(dataType + "/mavros/vision_pose/pose", 1);
    // estimate_orientation_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 1);

    // pub : MHE data
    estimate_vicon01_pub = nh.advertise<geometry_msgs::PoseStamped>(dataType + "/MHEdata/vicon01", 1);
    estimate_vicon02_pub = nh.advertise<geometry_msgs::PoseStamped>(dataType + "/MHEdata/vicon02", 1);
    estimate_flow_pub = nh.advertise<geometry_msgs::PoseStamped>(dataType + "/MHEdata/flow", 1);
    estimate_imu_pub = nh.advertise<geometry_msgs::PoseStamped>(dataType + "/MHEdata/imu", 1);
    estimate_uwb_pub = nh.advertise<geometry_msgs::PoseStamped>(dataType + "/MHEdata/uwb", 1);
    estimate_dt_pub = nh.advertise<geometry_msgs::PoseStamped>(dataType + "/MHEdata/dt", 1);
    estimate_height_pub = nh.advertise<geometry_msgs::PoseStamped>(dataType + "/MHEdata/height", 1);

    // pub : real data
    real_pose_pub = nh.advertise<geometry_msgs::PoseStamped>(dataType + "/Realdata/gtd", 1);
    real_vel_pub = nh.advertise<geometry_msgs::PoseStamped>(dataType + "/Realdata/vel", 1);
    real_xtest_pub = nh.advertise<geometry_msgs::PoseStamped>(dataType + "/Realdata/xtest", 1);

    ros::Rate loop_rate(rate);

    // main loop
    while(ros::ok()){

        ros::spinOnce();

        // ensure imu msg is obtained
        if (!(dwe_filter.flag_init_imu_)){
            continue;
        }

        // LOOP_num: current timestep
        ROS_INFO("----------ENTER ROS LOOP----------");
        ROS_INFO("LOOP: %d", dwe_filter.LOOP_num_);

        // show the data from uav
        ROS_INFO("DATA MHE_imu_: ux:%f uy:%f uz:%f",dwe_filter.MHE_imu_(0,dwe_filter.lopt_-1), 
                                                    dwe_filter.MHE_imu_(1,dwe_filter.lopt_-1),
                                                    dwe_filter.MHE_imu_(2,dwe_filter.lopt_-1));

        ROS_INFO("DATA MHE_uwb_: uwb:%f ",dwe_filter.MHE_uwb_(0,dwe_filter.lopt_));
        ROS_INFO("DATA MHE_flow_: vx:%f vy:%f vz:%f",dwe_filter.MHE_flow_(0,dwe_filter.lopt_), 
                                                     dwe_filter.MHE_flow_(1,dwe_filter.lopt_),
                                                     dwe_filter.MHE_flow_(2,dwe_filter.lopt_)); 
        ROS_INFO("DATA MHE_height_: %f", dwe_filter.MHE_height_(0,dwe_filter.lopt_));

        // get initial data xt_ until takeoff
        if(!dwe_filter.flag_init_ok_){

            // pub : output
            PubPoseXt(dwe_filter.xt_real_,&dwe_filter);
            PubVelXt(dwe_filter.xt_real_,&dwe_filter);
            PubOrientationXt(dwe_filter.MHE_imu_att_,&dwe_filter);

            // pub : MHE data
            PubVicon01Xt(dwe_filter.VICON_pos01_,&dwe_filter);
            PubVicon02Xt(dwe_filter.VICON_pos02_,&dwe_filter);
            Pubuwb(dwe_filter.MHE_uwb_,&dwe_filter);
            Pubflow(dwe_filter.MHE_flow_,&dwe_filter);
            Pubimu(dwe_filter.MHE_imu_,&dwe_filter);
            Pubdt(dwe_filter.dt_,0,&dwe_filter);

            PubRelativePos(dwe_filter.VICON_pos_,&dwe_filter);
            PubRelativeVel(dwe_filter.VICON_vel_,&dwe_filter);
            PubRelativeXt(dwe_filter.xt_,&dwe_filter);

            // pub : check data
            Pubheight(dwe_filter.MHE_height_,&dwe_filter);

            dwe_filter.LOOP_num_ = dwe_filter.LOOP_num_+1;
            dwe_filter.flag_init_imu_ = false; 
            // loop_rate.sleep();
            continue;
        }
        
        // estimated initial value
        double time_start = ros::Time::now().toSec();  

        dwe_filter.solveonce();  

        double time_end = ros::Time::now().toSec();
        ROS_INFO("consumptiom dt: %.4lf seconds", time_end - time_start);

        ROS_INFO("Estimated   data: x:%f y:%f z:%f",dwe_filter.xt_real_(0,dwe_filter.LOOP_num_-1), 
                                                    dwe_filter.xt_real_(1,dwe_filter.LOOP_num_-1),
                                                    dwe_filter.xt_real_(2,dwe_filter.LOOP_num_-1));
        ROS_INFO("Error real  data: x:%f y:%f z:%f",(dwe_filter.VICON_pos01_(0,dwe_filter.lopt_)-dwe_filter.xt_real_(0,dwe_filter.LOOP_num_-1)), 
        (dwe_filter.VICON_pos01_(1,dwe_filter.lopt_)-dwe_filter.xt_real_(1,dwe_filter.LOOP_num_-1)),
        (dwe_filter.VICON_pos01_(2,dwe_filter.lopt_)-dwe_filter.xt_real_(2,dwe_filter.LOOP_num_-1)));

        // pub : output
        PubPoseXt(dwe_filter.xt_real_,&dwe_filter);
        PubVelXt(dwe_filter.xt_real_,&dwe_filter);
        PubOrientationXt(dwe_filter.MHE_imu_att_,&dwe_filter);

        // pub : MHE data
        PubVicon01Xt(dwe_filter.VICON_pos01_,&dwe_filter);
        PubVicon02Xt(dwe_filter.VICON_pos02_,&dwe_filter);
        Pubuwb(dwe_filter.MHE_uwb_,&dwe_filter);
        Pubflow(dwe_filter.MHE_flow_,&dwe_filter);
        Pubimu(dwe_filter.MHE_imu_,&dwe_filter);
        Pubdt(dwe_filter.dt_,time_end - time_start,&dwe_filter);

        PubRelativePos(dwe_filter.VICON_pos_,&dwe_filter);
        PubRelativeVel(dwe_filter.VICON_vel_,&dwe_filter);
        PubRelativeXt(dwe_filter.xt_,&dwe_filter);

        // pub : check data
        Pubheight(dwe_filter.MHE_height_,&dwe_filter);

        dwe_filter.LOOP_num_ = dwe_filter.LOOP_num_+1;
        dwe_filter.flag_init_imu_ = false; 
    }
   
    ROS_INFO("----------FINISH ROS LOOP----------");
    return 0;
}
