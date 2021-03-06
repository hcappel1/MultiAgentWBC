#define _USE_MATH_DEFINES

#include <math.h>
#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <cmath>
#include <iostream>
#include <Eigen/Dense>
#include <control_node.hpp>
#include <ros/console.h>

using namespace std;
using namespace Eigen;

// void Jacobian(vector<double> theta_){

// }

Kinematics::Kinematics(){
    std::cout << "[Kinematics constructed]" << std::endl;
}

Kinematics::~Kinematics(){
    std::cout << "[Kinematics destructed]" << std::endl;
}

//Transformation matrices
MatrixXd Kinematics::T_1_0(double th1, double rho1){
    MatrixXd T_1_0(4,4);
    T_1_0(0,0) = cos(th1);
    T_1_0(0,1) = -sin(th1);
    T_1_0(0,2) = 0;
    T_1_0(0,3) = rho1*sin(th1);
    T_1_0(1,0) = sin(th1);
    T_1_0(1,1) = cos(th1);
    T_1_0(1,2) = 0;
    T_1_0(1,3) = rho1*sin(th1);
    T_1_0(2,0) = 0;
    T_1_0(2,1) = 0;
    T_1_0(2,2) = 1.0;
    T_1_0(2,3) = 0;
    T_1_0(3,0) = 0;
    T_1_0(3,1) = 0;
    T_1_0(3,2) = 0;
    T_1_0(3,3) = 1.0;

    return T_1_0;
}

MatrixXd Kinematics::T_2_1(double th2, double rho2){
    MatrixXd T_2_1(4,4);
    T_2_1(0,0) = cos(th2);
    T_2_1(0,1) = -sin(th2);
    T_2_1(0,2) = 0;
    T_2_1(0,3) = rho2*sin(th2);
    T_2_1(1,0) = sin(th2);
    T_2_1(1,1) = cos(th2);
    T_2_1(1,2) = 0;
    T_2_1(1,3) = rho2*sin(th2);
    T_2_1(2,0) = 0;
    T_2_1(2,1) = 0;
    T_2_1(2,2) = 1.0;
    T_2_1(2,3) = 0;
    T_2_1(3,0) = 0;
    T_2_1(3,1) = 0;
    T_2_1(3,2) = 0;
    T_2_1(3,3) = 1.0;

    return T_2_1;
}

MatrixXd Kinematics::T_3_2(double th3, double rho3){
    MatrixXd T_3_2(4,4);
    T_3_2(0,0) = cos(th3);
    T_3_2(0,1) = -sin(th3);
    T_3_2(0,2) = 0;
    T_3_2(0,3) = rho3*sin(th3);
    T_3_2(1,0) = sin(th3);
    T_3_2(1,1) = cos(th3);
    T_3_2(1,2) = 0;
    T_3_2(1,3) = rho3*sin(th3);
    T_3_2(2,0) = 0;
    T_3_2(2,1) = 0;
    T_3_2(2,2) = 1.0;
    T_3_2(2,3) = 0;
    T_3_2(3,0) = 0;
    T_3_2(3,1) = 0;
    T_3_2(3,2) = 0;
    T_3_2(3,3) = 1.0;

    return T_3_2;
}

MatrixXd Kinematics::Jacobian(VectorXd robot_state){
    MatrixXd J_(6,4);
    double th1 = robot_state(0);
    double th2 = robot_state(1);
    double th3 = robot_state(2);
    double rho1 = 0.6;
    double rho2 = 0.6;
    double rho3 = 0.6;

    double sig1 = rho2*sin(th2)+rho3*cos(th2)*sin(th3)+rho3*cos(th3)*sin(th2);
    double sig2 = rho2*cos(th2)+rho3*cos(th2)-rho3*sin(th2)*sin(th3);

    J_(0,0) = 1;
    J_(0,1) = -cos(th1)*sig1-sin(th1)*sig2-rho1*sin(th1);
    J_(0,2) = -cos(th1)*sig1-sin(th1)*sig2;
    J_(0,3) = rho2*cos(th1)*sin(th2)-sin(th1)*sig2-cos(th1)*sig1+rho2*cos(th2)*sin(th1);
    J_(1,0) = 0;
    J_(1,1) = cos(th1)*sig2-sin(th1)*sig1+rho1*cos(th1);
    J_(1,2) = cos(th1)*sig2-sin(th1)*sig1;
    J_(1,3) = cos(th1)*sig2-sin(th1)*sig1-rho2*cos(th1)*cos(th2)+rho2*sin(th1)*sin(th2);
    J_(2,0) = 0;
    J_(2,1) = 0;
    J_(2,2) = 0;
    J_(2,3) = 0;
    J_(3,0) = 0;
    J_(3,1) = 0;
    J_(3,2) = 0;
    J_(3,3) = 0;
    J_(4,0) = 0;
    J_(4,1) = 0;
    J_(4,2) = 0;
    J_(4,3) = 0;
    J_(5,0) = 0;
    J_(5,1) = 1;
    J_(5,2) = 1;
    J_(5,3) = 1;

    return J_;

}

bool Kinematics::Feasibility(VectorXd joint){
    double th1 = joint(0);
    double th2 = joint(1);
    double th3 = joint(2);
    joint_min = 3*M_PI/2;
    joint_max = M_PI/2;

    if (th1 > joint_max and th1 < joint_min){
        return false;
    }

    else if (th2 > joint_max and th2 < joint_min){
        return false;
    }

    else if (th3 > joint_max and th3 < joint_min){
        return false;
    }

    else{
        return true;
    }
}

VectorXd Kinematics::InvKin(VectorXd end_effector){
    VectorXd joint_state(3);

    double ee_x = end_effector(0);
    double ee_y = end_effector(1);
    double ee_z = end_effector(2);
    double ee_phi = end_effector(3);
    double rho1 = 0.6;
    double rho2 = 0.6;
    double rho3 = 0.6;

    double xp = ee_x - rho3*sin(ee_phi);
    double zp = ee_z - rho3*cos(ee_phi);
    double P = -2*rho1*zp;
    double Q = -2*rho1*xp;
    double R = pow(xp,2) + pow(zp,2) + pow(rho1,2) - pow(rho2,2);
    double D = sqrt(pow(P,2)+pow(Q,2));
    double gamma = atan2(Q/D,P/D);

    double th1 = gamma - acos(-R/D);
    double th2 = atan2((xp-rho1*sin(th1))/rho2,(zp-rho1*cos(th1))/rho2) - th1;
    double th3 = ee_phi-th1-th2;

    joint_state(0) = th1;
    joint_state(1) = th2;
    joint_state(2) = th3;

    return joint_state;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "state_publisher");

    VectorXd end_effector(4);
    end_effector(0) = -1.6;
    end_effector(1) = 0;
    end_effector(2) = 0.6;
    end_effector(3) = -M_PI/4;

    Kinematics kinematics;

    VectorXd joint = kinematics.InvKin(end_effector);
    if (isnan(joint(0))){
        ROS_ERROR("joint state infeasible");
    }
    else if (kinematics.Feasibility(joint)==false){
        ROS_ERROR("joint state infeasible");
    }
    else{
        ROS_INFO("calculated joint state");
        ROS_INFO("theta1: %f", joint(0));
        ROS_INFO("theta2: %f", joint(1));
        ROS_INFO("theta3: %f", joint(2));
    } 

    ros::NodeHandle n;
    ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("/control_node_joint_msg", 1);
    tf::TransformBroadcaster broadcaster;
    ros::Rate loop_rate(30);

    const double degree = M_PI/180;

    //robot state
    double tilt = 0, tinc = degree, swivel=0, angle=0, height=0, hinc=0.005;
    //double angle = 0;

    // message declarations
    geometry_msgs::TransformStamped odom_trans;
    sensor_msgs::JointState joint_state;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "axis";

    while (ros::ok()) {
        //update joint_state
        joint_state.header.stamp = ros::Time::now();
        joint_state.name.resize(4);
        joint_state.position.resize(4);
        joint_state.name[0] ="base_joint";
        joint_state.position[0] = 0.0;
        joint_state.name[1] ="base_to_bar1";
        joint_state.position[1] = joint(0);
        joint_state.name[2] ="bar1_to_bar2";
        joint_state.position[2] = joint(1);
        joint_state.name[3] ="bar2_to_bar3";
        joint_state.position[3] = joint(2);


        // update transform
        // (moving in a circle with radius=2)
        odom_trans.header.stamp = ros::Time::now();
        odom_trans.transform.translation.x = cos(angle)*2;
        odom_trans.transform.translation.y = sin(angle)*2;
        odom_trans.transform.translation.z = .7;
        odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(angle+M_PI/2);

        //send the joint state and transform
        joint_pub.publish(joint_state);
        broadcaster.sendTransform(odom_trans);

        // Create new robot state
        tilt += tinc;
        if (tilt<-.5 || tilt>0) tinc *= -1;
        height += hinc;
        if (height>.2 || height<0) hinc *= -1;
        swivel += degree;
        angle += degree/4;

        // This will adjust as needed per iteration
        loop_rate.sleep();
    }


    return 0;
}