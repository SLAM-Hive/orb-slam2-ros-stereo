#!/bin/bash
# this shell is used to wait until the txt files finished writing!
#traj1='flock -x -n /home/ORB_SLAM2/Examples/ROS/ORB_SLAM2/KeyFrameTrajectory_TUM_Format.txt -c "echo ok"'
#traj2='flock -x -n /home/ORB_SLAM2/Examples/ROS/ORB_SLAM2/FrameTrajectory_TUM_Format.txt -c "echo ok"'
traj1='flock -x -n /SLAM-Hive/slam_hive_algos/orb-slam2-ros-stereo/slamhive/1.txt -c "echo ok"'
traj2='flock -x -n /SLAM-Hive/slam_hive_algos/orb-slam2-ros-stereo/slamhive/2d.txt -c "echo ok"'
echo "aaa"
while true
do
    if [ "$traj1" = "ok" ];then
        if [ "$traj2" = "ok" ];then
            break
        fi
    fi
done
echo "txt files are all finished writing!"

