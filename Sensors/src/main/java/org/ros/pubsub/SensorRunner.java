package main.java.org.ros.pubsub;

import org.ros.address.InetAddressFactory;
import org.ros.node.DefaultNodeFactory;
import org.ros.node.DefaultNodeRunner;
import org.ros.node.Node;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMain;
import org.ros.node.NodeRunner;

import com.google.common.base.Preconditions;

public class SensorRunner implements NodeMain{

	private Node node;
	
	@Override
	public void main(NodeConfiguration configuration) {
		
		Preconditions.checkState(node == null);
		Preconditions.checkNotNull(configuration);

		try {
			node = new DefaultNodeFactory().newNode("sensor_runner", configuration);
			NodeRunner nodeRunner = DefaultNodeRunner.newDefault();
			
			
			// set the nodes up
			UltrasonicSensor ultraSen = new UltrasonicSensor();
			NodeConfiguration ultraConfig = NodeConfiguration.newPublic(InetAddressFactory.newNonLoopback().getHostName());
			ultraConfig.setNodeName("ultrasonic_sensor");
			
			IRSensor irSen= new IRSensor();
			NodeConfiguration irConfig = NodeConfiguration.newPublic(InetAddressFactory.newNonLoopback().getHostName());
			irConfig.setNodeName("infrared_sensor");
			
			PyroelectricSensor pyroSen = new PyroelectricSensor();
			NodeConfiguration pyroConfig = NodeConfiguration.newPublic(InetAddressFactory.newNonLoopback().getHostName());
			pyroConfig.setNodeName("human_sensor");
			
			
			// run them
			nodeRunner.run(ultraSen, ultraConfig);
			nodeRunner.run(irSen, irConfig);
			nodeRunner.run(pyroSen, pyroConfig);
			
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
