#include <ros/ros.h>
#include "PX4CtrlFSM.h"

//#include <quadrotor_msgs/SO3Command.h>
#include <geometry_msgs/PoseStamped.h>
#include <quadrotor_msgs/PositionCommand.h>
//#include <geometry_msgs/PoseWithCovarianceStamped.h>
//#include <std_msgs/Header.h>
//#include <geometry_msgs/Vector3Stamped.h>
#include <signal.h>

PX4CtrlFSM* pFSM;

void mySigintHandler(int sig) {
    ROS_INFO("[PX4Ctrl] exit...");
    ros::shutdown();
}

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "px4ctrl");
    ros::NodeHandle nh("~");

    signal(SIGINT, mySigintHandler);
    ros::Duration(1.0).sleep();

    Parameter_t param;

    Controller controller(param);
    HovThrKF hov_thr_kf(param);
    PX4CtrlFSM fsm(param, controller, hov_thr_kf);
    pFSM = &fsm;

    param.config_from_ros_handle(nh);
    param.init();
    fsm.hov_thr_kf.init();
    fsm.hov_thr_kf.set_hov_thr(param.hov_percent);
    
    ROS_INFO("Initial value for hov_thr set to %.2f/%.2f",
             fsm.hov_thr_kf.get_hov_thr(),
             param.mass * param.gra / param.full_thrust);
    ROS_INFO("Hovering thrust kalman filter is %s.",
             param.hover.use_hov_percent_kf ? "used" : "NOT used");

    fsm.controller.config();

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state", 
                                                                 10,
                                                                 boost::bind(&State_Data_t::feed, &fsm.state_data, _1));

    ros::Subscriber odom_sub =
        nh.subscribe<nav_msgs::Odometry>("odom",
                                         100,
                                         boost::bind(&Odom_Data_t::feed, &fsm.odom_data, _1),
                                         ros::VoidConstPtr(),
                                         ros::TransportHints().tcpNoDelay());

    ros::Subscriber cmd_sub = 
        nh.subscribe<quadrotor_msgs::PositionCommand>("cmd",
                                                      100,
                                                      boost::bind(&Command_Data_t::feed, &fsm.cmd_data, _1),
                                                      ros::VoidConstPtr(),
                                                      ros::TransportHints().tcpNoDelay());

    ros::Subscriber imu_sub =
        nh.subscribe<sensor_msgs::Imu>("/mavros/imu/data",
                                         100,
                                         boost::bind(&Imu_Data_t::feed, &fsm.imu_data, _1),
                                         ros::VoidConstPtr(),
                                         ros::TransportHints().tcpNoDelay());
    ros::Subscriber rc_sub =
        nh.subscribe<mavros_msgs::RCIn>("/mavros/rc/in",
                                         10,
                                         boost::bind(&RC_Data_t::feed, &fsm.rc_data, _1));
        
    fsm.controller.ctrl_FCU_pub = nh.advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude", 10);
    fsm.traj_start_trigger_pub = nh.advertise<geometry_msgs::PoseStamped>("/traj_start_trigger", 10);

    fsm.controller.set_FCU_mode = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

    ros::Duration(0.5).sleep();

    ROS_INFO("PX4CTRL] Waiting for rc");
    while (ros::ok()) {
        ros::spinOnce();
        if (fsm.rc_is_received(ros::Time::now())) {
            ROS_INFO("[PX4CTRL] rc received.");
            break;
        }
        ros::Duration(0.1).sleep();
    }

    int trials = 0;
    while(ros::ok() && !fsm.state_data.current_state.connected)
    {
        ros::spinOnce();
        ros::Duration(1.0).sleep();
        if ( trials ++ > 5 )
            ROS_ERROR("Unable to connnect to PX4!!!");
    }

    ros::Rate r(param.ctrl_rate);
    // ---- process ----
    while (ros::ok()) {
        r.sleep();
        ros::spinOnce();
        fsm.process();
    }

    return 0;
}
