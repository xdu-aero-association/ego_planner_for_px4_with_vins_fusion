#include "input.h"

RC_Data_t::RC_Data_t() {
    rcv_stamp = ros::Time(0);

    last_mode = -1.0;
    last_gear = -1.0;

    is_command_mode = false;
    enter_command_mode = false;
    is_api_mode = false;
    enter_api_mode = false;
}

void RC_Data_t::feed(mavros_msgs::RCInConstPtr pMsg) {
    msg = *pMsg;
    rcv_stamp = ros::Time::now();

    mode = ((double)msg.channels[4]-1000.0)/1000.0;
    gear = ((double)msg.channels[5]-1000.0)/1000.0;

    if ( !have_init_last_mode )
    {
        have_init_last_mode = true;
        last_mode = mode;
    } 
    if ( !have_init_last_gear )
    {
        have_init_last_gear = true;
        last_gear = gear;
    } 

    check_validity();

    if (last_mode < API_MODE_THRESHOLD_VALUE && mode > API_MODE_THRESHOLD_VALUE)
        enter_api_mode = true;
    else
        enter_api_mode = false;

    if (mode > API_MODE_THRESHOLD_VALUE)
        is_api_mode = true;
    else
        is_api_mode = false;

    if (is_api_mode && last_gear < GEAR_SHIFT_VALUE && gear > GEAR_SHIFT_VALUE) {
        enter_command_mode = true;
    } else if (gear < GEAR_SHIFT_VALUE) {
        enter_command_mode = false;
    }

    if (gear > GEAR_SHIFT_VALUE)
        is_command_mode = true;
    else
        is_command_mode = false;

    last_mode = mode;
    last_gear = gear;
}

void RC_Data_t::check_validity() {
    if ( mode >= -1.1 && mode <= 1.1 && gear >= -1.1 && gear <= 1.1) {
        // pass
    } else {
        ROS_ERROR("RC data validity check fail. mode=%f, gear=%f", mode, gear);
    }
}

Odom_Data_t::Odom_Data_t() {
    rcv_stamp = ros::Time(0);
    q.setIdentity();
};

void Odom_Data_t::feed(nav_msgs::OdometryConstPtr pMsg) {
    msg = *pMsg;
    rcv_stamp = ros::Time::now();

    uav_utils::extract_odometry(pMsg, p, v, q, w);

//#define IN_SIM
#ifdef IN_SIM   /* Set to 1 if use px4 HITL in gazebo. Because We found a bug in mavros(=1.1.0-1bionic.20200406.133946)
         odom output that velocity is relative to current body frame, not to world frame.*/  // zxzx
    Eigen::Quaternion<double> wRb_q(msg.pose.pose.orientation.w, msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z);
    Eigen::Matrix3d wRb = wRb_q.matrix();
    v = wRb * v;
    
    static int count = 0;
    if ( count++ % 100 == 0 )
        ROS_WARN("In simulation state!!!");
#endif

}

Imu_Data_t::Imu_Data_t() {
    rcv_stamp = ros::Time(0);
}

void Imu_Data_t::feed(sensor_msgs::ImuConstPtr pMsg) {
    msg = *pMsg;
    rcv_stamp = ros::Time::now();

    w(0) = msg.angular_velocity.x;
    w(1) = msg.angular_velocity.y;
    w(2) = msg.angular_velocity.z;

    a(0) = msg.linear_acceleration.x;
    a(1) = msg.linear_acceleration.y;
    a(2) = msg.linear_acceleration.z;

    q.x() = msg.orientation.x;
    q.y() = msg.orientation.y;
    q.z() = msg.orientation.z;
    q.w() = msg.orientation.w;
}

State_Data_t::State_Data_t() {
}

void State_Data_t::feed(mavros_msgs::StateConstPtr pMsg) {
    
    current_state = *pMsg; 
}

Command_Data_t::Command_Data_t() {
    rcv_stamp = ros::Time(0);
}

void Command_Data_t::feed(quadrotor_msgs::PositionCommandConstPtr pMsg) {

    msg = *pMsg;
    rcv_stamp = ros::Time::now();

    p(0) = msg.position.x;
    p(1) = msg.position.y;
    p(2) = msg.position.z;

    v(0) = msg.velocity.x;
    v(1) = msg.velocity.y;
    v(2) = msg.velocity.z;

    a(0) = msg.acceleration.x;
    a(1) = msg.acceleration.y;
    a(2) = msg.acceleration.z;
    
    yaw = uav_utils::normalize_angle(msg.yaw);
}
