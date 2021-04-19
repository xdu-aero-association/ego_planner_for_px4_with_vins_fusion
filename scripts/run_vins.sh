sudo chmod 777 /dev/ttyTHS0
#sudo cpufreq-set -g performance
sudo sh ~/max_cpu_freq.sh
sudo sh ~/max_gpu_freq.sh
sudo jetson_clocks
roslaunch mavros px4.launch & sleep 3;

roslaunch realsense2_camera rs_camera.launch & sleep 7;

roslaunch realsense2_camera rs_t265.launch & sleep 5;

wait;
