package main.java.org.ros.navigation;

import main.java.org.ros.navigation.BraitenburgAvoid;
import main.java.org.ros.navigation.MotorControl;
import main.java.org.ros.navigation.ObstacleAvoidance;

import org.ros.address.InetAddressFactory;
import org.ros.node.DefaultNodeFactory;
import org.ros.node.DefaultNodeRunner;
import org.ros.node.Node;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMain;
import org.ros.node.NodeRunner;

import com.google.common.base.Preconditions;

public class NavRunner implements NodeMain{

	private Node node;
	
	@Override
	public void main(NodeConfiguration configuration) {
		
		Preconditions.checkState(node == null);
		Preconditions.checkNotNull(configuration);

		try {
			node = new DefaultNodeFactory().newNode("nav_runner", configuration);
			NodeRunner nodeRunner = DefaultNodeRunner.newDefault();
			
			
			// set the nodes up
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
			
			SensorAvoidance infraAvoid = new SensorAvoidance();
			NodeConfiguration infraAvoidConfig = NodeConfiguration.newPublic(InetAddressFactory.newNonLoopback().getHostName());
			infraAvoidConfig.setNodeName("infrared_avoid");
			
			
			
			SensorAvoidance ultraAvoid = new SensorAvoidance();
			NodeConfiguration ultraAvoidConfig = NodeConfiguration.newPublic(InetAddressFactory.newNonLoopback().getHostName());
			ultraAvoidConfig.setNodeName("ultrasonic_avoid");
			
			
			SensorSideAvoidance leftIR = new SensorSideAvoidance();
			NodeConfiguration leftIRConfig = NodeConfiguration.newPublic(InetAddressFactory.newNonLoopback().getHostName());
			leftIRConfig.setNodeName("left_IR");
			
			SensorSideAvoidance rightIR = new SensorSideAvoidance();
			NodeConfiguration rightIRConfig = NodeConfiguration.newPublic(InetAddressFactory.newNonLoopback().getHostName());
			rightIRConfig.setNodeName("right_IR");

			SensorSideAvoidance leftUS = new SensorSideAvoidance();
			NodeConfiguration leftUSConfig = NodeConfiguration.newPublic(InetAddressFactory.newNonLoopback().getHostName());
			leftUSConfig.setNodeName("left_US");
			
			
			SensorSideAvoidance rightUS = new SensorSideAvoidance();
			NodeConfiguration rightUSConfig = NodeConfiguration.newPublic(InetAddressFactory.newNonLoopback().getHostName());
			rightUSConfig.setNodeName("right_US");
			
			*/
			
			// run them
			nodeRunner.run(obsAvoid, obsConfig);
			nodeRunner.run(mtrCtr, mtrCtrConfig);
			
			
			nodeRunner.run(infraAvoid, infraAvoidConfig);
			nodeRunner.run(ultraAvoid, ultraAvoidConfig);
			/*
			nodeRunner.run(leftIR, leftIRConfig);
			nodeRunner.run(rightIR, rightIRConfig);
			nodeRunner.run(leftUS, leftUSConfig);
			nodeRunner.run(rightUS, rightUSConfig);
			*/
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
