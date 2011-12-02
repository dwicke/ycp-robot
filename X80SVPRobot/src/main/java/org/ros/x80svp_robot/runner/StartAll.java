package main.java.org.ros.x80svp_robot.runner;

import org.ros.address.InetAddressFactory;
import org.ros.node.DefaultNodeFactory;
import org.ros.node.DefaultNodeRunner;
import org.ros.node.Node;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMain;
import org.ros.node.NodeRunner;

import main.java.org.ros.x80svp_robot.navigation.*;
import main.java.org.ros.x80svp_robot.robot.*;
import main.java.org.ros.x80svp_robot.sensorfilters.*;

import com.google.common.base.Preconditions;
/**
 * This is the NodeRunner for the Robot package.  It starts
 * all nodes in this package. 
 * 
 */
public class StartAll implements NodeMain{

	private Node node;

	@Override
	public void main(NodeConfiguration configuration) {

		Preconditions.checkState(node == null);
		Preconditions.checkNotNull(configuration);

		try {
			node = new DefaultNodeFactory().newNode("start_all", configuration);


			NodeRunner nodeRunner = DefaultNodeRunner.newDefault();

			// I have tried copying configuration, i have tried newPrivate rather than public
			// and each time errors occur due to the need for publshers and subscribers needing
			// the network.  So, the device that this code runs on must have a network functionality
			// available for java to use.
			SensorListener senseListen = new SensorListener();
			NodeConfiguration nodeConfig = NodeConfiguration.newPublic(InetAddressFactory.newNonLoopback().getHostName());
			nodeConfig.setNodeName("sensor_listener");


			UltrasonicSensorFilter ultraSen = new UltrasonicSensorFilter();
			NodeConfiguration ultraConfig = NodeConfiguration.newPublic(InetAddressFactory.newNonLoopback().getHostName());
			ultraConfig.setNodeName("ultrasonic_sensor");

			IRSensorFilter irSen= new IRSensorFilter();
			NodeConfiguration irConfig = NodeConfiguration.newPublic(InetAddressFactory.newNonLoopback().getHostName());
			irConfig.setNodeName("infrared_sensor");
/*
			PyroelectricSensorFilter pyroSen = new PyroelectricSensorFilter();
			NodeConfiguration pyroConfig = NodeConfiguration.newPublic(InetAddressFactory.newNonLoopback().getHostName());
			pyroConfig.setNodeName("human_sensor");
*/


			ObstacleAvoidance obsAvoid = new ObstacleAvoidance();
			NodeConfiguration obsConfig = NodeConfiguration.newPublic(InetAddressFactory.newNonLoopback().getHostName());
			obsConfig.setNodeName("brait_obstacle_avoidance");



			MotorControl mtrCtr= new MotorControl();
			NodeConfiguration mtrCtrConfig = NodeConfiguration.newPublic(InetAddressFactory.newNonLoopback().getHostName());
			mtrCtrConfig.setNodeName("motor_control");


			BraitenburgAvoid infraAvoid = new BraitenburgAvoid();
			NodeConfiguration infraAvoidConfig = NodeConfiguration.newPublic(InetAddressFactory.newNonLoopback().getHostName());
			infraAvoidConfig.setNodeName("brait_infrared_avoid");



			BraitenburgAvoid ultraAvoid = new BraitenburgAvoid();
			NodeConfiguration ultraAvoidConfig = NodeConfiguration.newPublic(InetAddressFactory.newNonLoopback().getHostName());
			ultraAvoidConfig.setNodeName("brait_ultrasonic_avoid");


/*
			HeatTrack track = new HeatTrack();
			NodeConfiguration trackConfig = NodeConfiguration.newPublic(InetAddressFactory.newNonLoopback().getHostName());
			trackConfig.setNodeName("heat_track");


			HeatSearch heat_search = new HeatSearch();
			NodeConfiguration heatSearchConfig = NodeConfiguration.newPublic(InetAddressFactory.newNonLoopback().getHostName());
			heatSearchConfig.setNodeName("heat_search");

			*/


			nodeRunner.run(obsAvoid, obsConfig);
			nodeRunner.run(mtrCtr, mtrCtrConfig);


			nodeRunner.run(infraAvoid, infraAvoidConfig);
			nodeRunner.run(ultraAvoid, ultraAvoidConfig);
			//nodeRunner.run(heat_search, heatSearchConfig);
			//nodeRunner.run(track, trackConfig);




			// run them
			nodeRunner.run(ultraSen, ultraConfig);
			nodeRunner.run(irSen, irConfig);
			//nodeRunner.run(pyroSen, pyroConfig);

			nodeRunner.run(senseListen, nodeConfig);

		} catch (Exception e) {
			if (node != null) {
				node.getLog().fatal(e);
			} else {
				e.printStackTrace();
			}
		}


	}

	@Override
	public void shutdown() {
		node.shutdown();
	}

}
