#ifndef CONTROL_NODE_H
#define CONTROL_NODE_H

#include <cmath>
#include <iostream>
#include <Eigen/Dense>

using namespace Eigen;



class Kinematics{
public:
	Kinematics();
	virtual ~Kinematics();

	typedef Matrix<double, Dynamic, Dynamic> MatrixXd;
	typedef Matrix<double, Dynamic, 1> VectorXd;

	MatrixXd J;
	VectorXd robot_state;
	VectorXd end_effector;
	double joint_max;
	double joint_min;

	MatrixXd Jacobian(VectorXd robot_state);
	MatrixXd T_1_0(double th1, double rho1);
	MatrixXd T_2_1(double th2, double rho2);
	MatrixXd T_3_2(double th3, double rho3);

	VectorXd InvKin(VectorXd end_effector);
	bool Feasibility(VectorXd joint);
};



#endif