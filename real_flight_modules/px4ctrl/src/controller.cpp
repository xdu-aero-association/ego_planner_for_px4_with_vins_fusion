#include "controller.h"
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <uav_utils/converters.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <boost/format.hpp>

using namespace Eigen;
using std::cout;
using std::endl;
using namespace uav_utils;

Controller::Controller(Parameter_t& param_):
	param(param_)
{
	is_configured = false;
	int_e_v.setZero();
}

void Controller::config()
{
	config_gain(param.hover_gain);
	is_configured = true;
}

void Controller::config_gain(const Parameter_t::Gain& gain)
{
	Kp.setZero();
	Kv.setZero();
	Ka.setZero();
	Kp(0,0) = gain.Kp0;
	Kp(1,1) = gain.Kp1;
	Kp(2,2) = gain.Kp2;
	Kv(0,0) = gain.Kv0;
	Kv(1,1) = gain.Kv1;
	Kv(2,2) = gain.Kv2;
	Kvi(0,0) = gain.Kvi0;
	Kvi(1,1) = gain.Kvi1;
	Kvi(2,2) = gain.Kvi2;
	Ka(0,0) = gain.Ka0;
	Ka(1,1) = gain.Ka1;
	Ka(2,2) = gain.Ka2;
	Kyaw = gain.Kyaw;
}

void Controller::update(
	const Desired_State_t& des, 
	const Odom_Data_t& odom, 
	Controller_Output_t& u, 
	SO3_Controller_Output_t& u_so3
)
{
	ROS_ASSERT_MSG(is_configured, "Gains for controller might not be initialized!");
	std::string constraint_info("");
	Vector3d e_p, e_v, F_des;
	double e_yaw = 0.0;


	if (des.v(0) != 0.0 || des.v(1) != 0.0 || des.v(2) != 0.0) {
		// ROS_INFO("Reset integration");
		int_e_v.setZero();
	}

	double yaw_curr = get_yaw_from_quaternion(odom.q);
	double	yaw_des = des.yaw;
	Matrix3d wRc = rotz(yaw_curr);
	Matrix3d cRw = wRc.transpose();

	e_p = des.p - odom.p;
	Eigen::Vector3d u_p = wRc * Kp * cRw * e_p;
	
	u.des_v_real = des.v + u_p; // For estimating hover percent
	e_v = des.v + u_p - odom.v;

	const std::vector<double> integration_enable_limits = {0.1, 0.1, 0.1};
	for (size_t k = 0; k < 3; ++k) {
		if (std::fabs(e_v(k)) < 0.2) {
			int_e_v(k) += e_v(k) * 1.0 / 50.0;
		}
	}

	Eigen::Vector3d u_v_p = wRc * Kv * cRw * e_v;
	const std::vector<double> integration_output_limits = {0.4, 0.4, 0.4};
	Eigen::Vector3d u_v_i = wRc * Kvi * cRw * int_e_v;
	for (size_t k = 0; k < 3; ++k) {
		if (std::fabs(u_v_i(k)) > integration_output_limits[k]) {
			uav_utils::limit_range(u_v_i(k), integration_output_limits[k]);
			ROS_INFO("Integration saturate for axis %zu, value=%.3f", k, u_v_i(k));
		}
	}

	Eigen::Vector3d u_v = u_v_p + u_v_i;

	e_yaw = yaw_des - yaw_curr;

	while(e_yaw > M_PI) e_yaw -= (2 * M_PI);
	while(e_yaw < -M_PI) e_yaw += (2 * M_PI);

	double u_yaw = Kyaw * e_yaw;
	
	
	F_des = u_v * param.mass + 
		Vector3d(0, 0, param.mass * param.gra) + Ka * param.mass * des.a;
	
	if (F_des(2) < 0.5 * param.mass * param.gra)
	{
		constraint_info = boost::str(
			boost::format("thrust too low F_des(2)=%.3f; ")% F_des(2));
		F_des = F_des / F_des(2) * (0.5 * param.mass * param.gra);
	}
	else if (F_des(2) > 2 * param.mass * param.gra)
	{
		constraint_info = boost::str(
			boost::format("thrust too high F_des(2)=%.3f; ")% F_des(2));
		F_des = F_des / F_des(2) * (2 * param.mass * param.gra);
	}

	if (std::fabs(F_des(0)/F_des(2)) > std::tan(toRad(50.0)))
	{
		constraint_info += boost::str(boost::format("x(%f) too tilt; ")
			% toDeg(std::atan2(F_des(0),F_des(2))));
		F_des(0) = F_des(0)/std::fabs(F_des(0)) * F_des(2) * std::tan(toRad(30.0));
	}

	if (std::fabs(F_des(1)/F_des(2)) > std::tan(toRad(50.0)))
	{
		constraint_info += boost::str(boost::format("y(%f) too tilt; ")
			% toDeg(std::atan2(F_des(1),F_des(2))));
		F_des(1) = F_des(1)/std::fabs(F_des(1)) * F_des(2) * std::tan(toRad(30.0));	
	}

	Vector3d z_b_des = F_des / F_des.norm();
	
	/////////////////////////////////////////////////
	// Z-X-Y Rotation Sequence                
	// Vector3d x_c_des = Vector3d(std::cos(yaw_des), sin(yaw_des), 0.0);
	// Vector3d y_b_des = z_b_des.cross(x_c_des) / z_b_des.cross(x_c_des).norm();
	// Vector3d x_b_des = y_b_des.cross(z_b_des);
	/////////////////////////////////////////////////

	/////////////////////////////////////////////////
	// Z-Y-X Rotation Sequence                
	Vector3d y_c_des = Vector3d(-std::sin(yaw_des), std::cos(yaw_des), 0.0);
	Vector3d x_b_des = y_c_des.cross(z_b_des) / y_c_des.cross(z_b_des).norm();
	Vector3d y_b_des = z_b_des.cross(x_b_des);
	///////////////////////////////////////////////// 

	Matrix3d R_des1; // it's wRb
	R_des1 << x_b_des, y_b_des, z_b_des;
	
	Matrix3d R_des2; // it's wRb
	R_des2 << -x_b_des, -y_b_des, z_b_des;
	
	Vector3d e1 = R_to_ypr(R_des1.transpose() * odom.q.toRotationMatrix());
	Vector3d e2 = R_to_ypr(R_des2.transpose() * odom.q.toRotationMatrix());

	Matrix3d R_des; // it's wRb

	if (e1.norm() < e2.norm())
	{
		R_des = R_des1;
	}
	else
	{
		R_des = R_des2;
	}

	{	
		Vector3d F_c = wRc.transpose() * F_des;
		Matrix3d wRb_odom = odom.q.toRotationMatrix();
		Vector3d z_b_curr = wRb_odom.col(2);
		double u1 = F_des.dot(z_b_curr);
		double fx = F_c(0);
		double fy = F_c(1);
		double fz = F_c(2);
		u.roll  = std::atan2(-fy, fz);
		u.pitch = std::atan2( fx, fz);
		u.thrust = u1 / param.full_thrust;
		if(param.use_yaw_rate_ctrl){
			u.yaw_mode = Controller_Output_t::CTRL_YAW_RATE;
			u.yaw = u_yaw;
		}
		else{
			u.yaw_mode = Controller_Output_t::CTRL_YAW;
			u.yaw = des.yaw;
		}
	}

};

void Controller::publish_ctrl(const Controller_Output_t& u, const ros::Time& stamp)
{
	mavros_msgs::AttitudeTarget msg;

	msg.header.stamp = stamp;
	msg.header.frame_id = std::string("FCU");

	msg.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ROLL_RATE  | 
					mavros_msgs::AttitudeTarget::IGNORE_PITCH_RATE | 
					mavros_msgs::AttitudeTarget::IGNORE_YAW_RATE;

	Eigen::Vector3d uAngle(u.yaw,u.pitch,u.roll);
	Eigen::Quaterniond quaternion = ypr_to_quaternion(uAngle);
	// Eigen::Quaterniond quaternion;
	// quaternion = Eigen::AngleAxisd(AngleAxisd(uAngle(0),Vector3d::UnitZ())) * 
	// 			 Eigen::AngleAxisd(AngleAxisd(uAngle(1),Vector3d::UnitY())) * 
	// 			 Eigen::AngleAxisd(AngleAxisd(uAngle(2),Vector3d::UnitX()));
	msg.orientation.x = quaternion.x();
	msg.orientation.y = quaternion.y();
	msg.orientation.z = quaternion.z();
	msg.orientation.w = quaternion.w();

	msg.thrust = u.thrust;
	
    ctrl_FCU_pub.publish(msg);
}

void Controller::publish_zero_ctrl(const ros::Time& stamp)
{
	mavros_msgs::AttitudeTarget msg;

	msg.header.stamp = stamp;
	msg.header.frame_id = std::string("FCU");

	msg.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ROLL_RATE  | 
					mavros_msgs::AttitudeTarget::IGNORE_PITCH_RATE | 
					mavros_msgs::AttitudeTarget::IGNORE_YAW_RATE;

	Eigen::Vector3d uAngle(0.0, 0.0, 0.0);
	Eigen::Quaterniond quaternion = ypr_to_quaternion(uAngle);
	// Eigen::Quaterniond quaternion;
	// quaternion = Eigen::AngleAxisd(AngleAxisd(uAngle(0),Vector3d::UnitZ())) * 
	// 			 Eigen::AngleAxisd(AngleAxisd(uAngle(1),Vector3d::UnitY())) * 
	// 			 Eigen::AngleAxisd(AngleAxisd(uAngle(2),Vector3d::UnitX()));
	msg.orientation.x = quaternion.x();
	msg.orientation.y = quaternion.y();
	msg.orientation.z = quaternion.z();
	msg.orientation.w = quaternion.w();
	
	msg.thrust = param.hov_percent*0.8;
	
    ctrl_FCU_pub.publish(msg);
}
