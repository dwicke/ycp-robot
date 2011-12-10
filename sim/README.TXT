To install dependencies:
	./install_GL_dependencies.sh

To make:
	rosmake --pre-clean sim

To run:
	rosrun sim sim
	Press H to toggle on-screen command help.

Known bugs:
	* When window is resized, may report bogus sensor data for 1-3 messages.
	* Memory leak on re-init (will be fixed when sensors are made to load dynamically).
	* Pointcloud/depthmap values need calibrated- may be off-scale or distorted.
	* Need more to-scale room environment, current 20x20 meter one maxes out Kinect range.
	* Doesn't currently compute and publish depthmap_float data.
