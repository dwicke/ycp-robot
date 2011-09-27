/*
 * Copyright (C) 2011 Google Inc.
 * 
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 * 
 * http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
 */

package main.java.org.ros.pubsub;

import org.apache.commons.logging.Log;
import org.ros.message.MessageListener;
import org.ros.node.DefaultNodeFactory;
import org.ros.node.Node;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMain;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;
import org.ros.message.robot_msgs.*;
import org.ros.message.sensor_msgs.Range;

/**
 * This is a simple rosjava {@link Subscriber} {@link Node}. It assumes an
 * external roscore is already running.  The job of the Robot listener is to
 * listen for messages that have the sensor data in our implementation is from
 * either Converter or from VirtualX80SVP it depends on the startup configuration.
 * 
 * @author drewwicke@google.com (Drew Wicke)
 */
public class SensorListener implements NodeMain {

	private Node node;

	@Override
	public void main(NodeConfiguration configuration) {

		//ParameterTreenode.newParameterTree();
		try {
			node = new DefaultNodeFactory().newNode("sensor_listener", configuration);
			
			final Log log = node.getLog();
			node.newSubscriber("SensorData", "robot_msgs/SensorData",
					new MessageListener<SensorData>() {

				@Override
				public void onNewMessage(SensorData message) {
					
					log.info("I heard: \"" + message.infrared_frontLeftLeft_distance + "\"");
					
					// Ok so I heard the sensor data so publish the data in ROS format
					// to the specific topics
					
					// first do IR
					
					
					
					// then do Ultrasonic
					
					
					
				}
			});
		} catch (Exception e) {
			if (node != null) {
				node.getLog().fatal(e);
			} else {
				e.printStackTrace();
			}
		}





	}
	
	public void publishIR(String topic, float range)
	{
		Range leftfrontIR = new Range();
		leftfrontIR.radiation_type = Range.INFRARED;
		leftfrontIR.range = range;
		//String topics = message.infrared_frontLeftLeft_distance;
		
		Publisher<Range> publisher =
		          node.newPublisher("fronLeftLeftIRData", "sensor_msgs/Range");
		publisher.publish(leftfrontIR);
	}

	@Override
	public void shutdown() {
		node.shutdown();
	}

}
