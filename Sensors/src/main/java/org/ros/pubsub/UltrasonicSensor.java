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

import java.util.List;

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
 * external roscore is already running.  The job of the UltrasonicSensor node
 * is to filter the raw sensor data and publish it.
 * 
 * @author drewwicke@google.com (Drew Wicke)
 */
public class UltrasonicSensor implements NodeMain, MessageListener<Range> {

	private Node node;
	private Range prevFilteredRange, curRange;
	private Publisher<Range> filteredRange;
	
	private final float RDelta = (float) .2;// .2 m or 20cm can change if need to just chose this because
	// this is what they had in paper

	private Log log;
	@Override
	public void main(NodeConfiguration configuration) {

		//ParameterTreenode.newParameterTree();
		try {
			// this name will be changed when the node is created.
			// because multiple UltrasonicSensor nodes will be created
			// The name will become the name of the topic also this name will
			// come from the name of the variable from robot_msgs 
			node = new DefaultNodeFactory().newNode("sensor_listener", configuration);
			log = node.getLog();
			
			// Print out the name
			log.info("Sensor name: " + node.getName());
			filteredRange =
					node.newPublisher(node.getName() + "filtered", "sensor_msgs/Range");
			
			
			// then to get angle info about sensor do 
			//Integer senseNames = h.getInteger(node.getName() + "theta");
			
			// so I am subscribing to the raw sensor data coming from SensorListener
			// so that I can do some filtering
			node.newSubscriber(node.getName() + "raw", "sensor_msgs/Range", this);
			
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

	@Override
	public void onNewMessage(Range message) {
		// TODO Auto-generated method stub
		
		// set the prev to the current and the current to the just received message
		prevFilteredRange = curRange;
		curRange = message;
		
		// now publish the data
		
		
		
		if (curRange.range < curRange.max_range)
		{
			// if the range received is less than the max range then it is good
			// don't need to do any filtering
			
			filteredRange.publish(curRange);
			
			
		}
		else
		{
			//else if is equal to the max_range so must filter
			// create a clone of a Range message
			Range filtered = message.clone();
			
			// so I set the filtered range to the smallest value
			// the previous filtered range plus a delta
			// or the max range
			filtered.range = prevFilteredRange.range + RDelta;
			if (filtered.range > message.max_range)
			{
				filtered.range = message.max_range;
			}
			prevFilteredRange = filtered;
			// publish the filtered range
			filteredRange.publish(filtered);
		}
			
		
	}

}
