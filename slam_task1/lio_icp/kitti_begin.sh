#!/bin/bash

# Source ROS setup
source /home/siai/Desktop/EIE_WORK/slam_task1/lio_icp/devel/setup.bash

# 启动 roscore
echo "Starting roscore..."
roscore &
ROSCORE_PID=$!
sleep 5

# 启动节点的 launch 文件
echo "Launching front_end.launch..."
roslaunch lio_ndt front_end.launch &
LAUNCH_PID=$!
sleep 5

# 启动 KITTI 数据的 bag 文件
echo "Playing KITTI bag file..."
gnome-terminal -- bash -c "rosbag play -r 0.5 /home/siai/Desktop/data.bag; exec bash" &

# 捕捉 Ctrl+C 信号以便停止所有进程
trap "echo 'Stopping...'; kill $ROSCORE_PID $LAUNCH_PID; exit" SIGINT

# 等待所有进程结束
wait $ROSCORE_PID $LAUNCH_PID