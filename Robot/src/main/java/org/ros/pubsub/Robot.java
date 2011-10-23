package main.java.org.ros.pubsub;

import org.ros.address.InetAddressFactory;
import org.ros.node.DefaultNodeFactory;
import org.ros.node.DefaultNodeRunner;
import org.ros.node.Node;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMain;
import org.ros.node.NodeRunner;

import com.google.common.base.Preconditions;
/**
 * This is the NodeRunner for the Robot package.  It starts
 * all nodes in this package. 
 * 
 */
public class Robot implements NodeMain{

	private Node node;
	
	@Override
	public void main(NodeConfiguration configuration) {
		
		Preconditions.checkState(node == null);
		Preconditions.checkNotNull(configuration);

		try {
			node = new DefaultNodeFactory().newNode("robot_runner", configuration);
			
			SensorListener senseListen = new SensorListener();
			NodeConfiguration nodeConfig = NodeConfiguration.newPublic(InetAddressFactory.newNonLoopback().getHostName());
			nodeConfig.setNodeName("sensor_listener");
			NodeRunner nodeRunner = DefaultNodeRunner.newDefault();
			
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
