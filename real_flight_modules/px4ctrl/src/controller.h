#ifndef __CONTROLLER_H
#define __CONTROLLER_H

#include <sensor_msgs/Imu.h>
#include <mavros_msgs/AttitudeTarget.h>

#include "input.h"

struct Desired_State_t
{
	Eigen::Vector3d p;
	Eigen::Vector3d v;
	double yaw;
	Eigen::Quaterniond q;
	Eigen::Vector3d a;
};

struct Controller_Output_t
{
    static constexpr double CTRL_YAW_RATE = 1.0;
    static constexpr double CTRL_YAW = 0.0;

	double roll;
	double pitch;
	double yaw;
	double thrust;
	double yaw_mode; // if yaw_mode > 0, CTRL_YAW;
				// if yaw_mode < 0, CTRL_YAW_RATE

	Eigen::Vector3d des_v_real;
};

struct SO3_Controller_Output_t
{
	Eigen::Matrix3d Rdes;
	Eigen::Vector3d Fdes;
	double net_force;
};

class Controller
{
public:
	Parameter_t& param;

	ros::Publisher ctrl_FCU_pub;
	ros::ServiceClient set_FCU_mode;

	Eigen::Matrix3d Kp;
	Eigen::Matrix3d Kv;
	Eigen::Matrix3d Kvi;
	Eigen::Matrix3d Ka;
	double Kyaw;

	Eigen::Vector3d int_e_v;

	Controller(Parameter_t&);
	void config_gain(const Parameter_t::Gain& gain);
	void config();
	void update(const Desired_State_t& des, const Odom_Data_t& odom, 
		Controller_Output_t& u, SO3_Controller_Output_t& u_so3
	);
	
	void publish_ctrl(const Controller_Output_t& u, const ros::Time& stamp);
	void publish_zero_ctrl(const ros::Time& stamp);

private:
	bool is_configured;
};

#endif
