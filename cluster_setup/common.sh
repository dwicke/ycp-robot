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

echo Configuring the interface...
sudo ifconfig -v eth0 up $ROS_IP

echo Done!
echo

echo "(If it doesnt work, make sure you ran it as '. [machine].sh')"
echo