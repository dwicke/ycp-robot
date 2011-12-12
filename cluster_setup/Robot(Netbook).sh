echo Configuring this machine for the Robot/Serial setup.
echo

echo Setting ROS parameters...
export ROS_MASTER_URI=http://192.168.2.201:11311
export ROS_IP=192.168.2.204
echo Using $ROS_IP for this machine and it will connect to $ROS_MASTER_URI as server.
echo

echo Checking to see if the environment is sane...
if [ -z "$ROS_IP" ]; then
	echo "Don't run this directly.";
	return;
fi;
export ROSVERSION=`rosversion -d`
if [ $ROSVERSION != 'diamondback' ]; then
	echo "It isnt. Run setup.sh first! ($ROSVERSION)";
	return;
fi;
echo Environment checks out.
echo

echo Configuring the wifi interface...
sudo ifconfig -v eth0 down
sudo ifconfig -v wlan0 up $ROS_IP

echo Done!
echo

echo "(If it doesnt work, make sure you ran it as '. [machine].sh')"
echo

