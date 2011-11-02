To install dependencies:
	./install_GL_dependencies.sh

To make:
	rosmake --pre-clean sim

To run:
	roscd sim && rosrun sim sim

Known bugs:
	* When window is resized, may report bogus sensor data for 1-3 messages.
	* Virtual Kinect sensor doesn't render at fixed resolution (runs very slow).
	* Convergence functions being called before window is initialized (fixed?).
	* Memory leak on re-init (will be fixed when sensors are made to load dynamically).
