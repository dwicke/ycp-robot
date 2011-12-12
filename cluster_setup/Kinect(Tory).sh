echo Configuring this machine for the Kinect setup.
echo

echo Setting ROS parameters...
export ROS_MASTER_URI=http://192.168.2.201:11311
export ROS_IP=192.168.2.203
echo Using $ROS_IP for this machine and it will connect to $ROS_MASTER_URI as server.
echo

. common.sh
