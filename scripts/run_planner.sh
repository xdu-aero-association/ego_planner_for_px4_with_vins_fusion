sudo chmod 777 /dev/ttyTHS0
#sudo cpufreq-set -g performance
sudo sh ../../../max_cpu_freq.sh
sudo sh ../../../max_gpu_freq.sh
sudo jetson_clocks
roslaunch mavros px4.launch & sleep 3;

roslaunch realsense2_camera rs_camera.launch & sleep 7;

rosrun vins vins_node /home/nv/zxzx/ego_planner_ws/src/real_flight_modules/VINS-Fusion-gpu/config/px4/euroc_stereo_imu_config.yaml & sleep 5;
#roslaunch vins_estimator euroc.launch & sleep 3;

roslaunch ego_planner run_in_exp.launch & sleep 3;

roslaunch px4ctrl ctrl_md.launch;

wait;
