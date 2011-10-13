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
import java.util.Map;
import java.util.TreeMap;

import org.apache.commons.logging.Log;
import org.apache.commons.logging.impl.SimpleLog;
import org.ros.message.MessageListener;
import org.ros.node.DefaultNodeFactory;
import org.ros.node.Node;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMain;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;
import org.ros.message.sensor_msgs.Range;

import com.google.common.base.Preconditions;

/**
 * This is a simple rosjava {@link Subscriber} {@link Node}. It assumes an
 * external roscore is already running.  The job of the UltrasonicSensor node
 * is to filter the raw sensor data and publish it.
 * 
 * @author drewwicke@google.com (Drew Wicke)
 */
public class UltrasonicSensor implements NodeMain, MessageListener<Range> {

	private Node node;
	// there will be a prev... and cur... for each of the US
	private Map<String, Range> prevFilteredRange;
	// there will be a topic to publish to for each of the sensors
	private Map<String, Publisher<Range> > filteredRangeMap;

	private final float RDelta = (float) .2;// .2 m or 20cm can change if need to just chose this because
	// this is what they had in paper

	private SimpleLog log;
	@Override
	public void main(NodeConfiguration configuration) {

		Preconditions.checkState(node == null);
		Preconditions.checkNotNull(configuration);

		try {
			// this name will be changed when the node is created.
			node = new DefaultNodeFactory().newNode("sensor_listener", configuration);
			// create a log.
			log = new SimpleLog(node.getName().toString());

			// Print out the name
			log.info("Sensor name: " + node.getName());



			filteredRangeMap = new TreeMap<String, Publisher<Range>>();
			prevFilteredRange = new TreeMap<String, Range>();
			

			// get the names of the sensors for the IR sensors so I can make
			// the name of the topic I am going to publish to 
			List<String> topics = (List<String>) node.newParameterTree().getList("ultrasonic_topics");
			for (String topic : topics)
			{
				Publisher<Range> filteredRange =
						node.newPublisher(topic + "filtered", "sensor_msgs/Range");
				// make the new publisher and add it to the map
				filteredRangeMap.put(topic, filteredRange);
			}



			// make this node a subscriber to the Range messages that are being published
			// by SensorListener.
			// must do this after I create the publishers because I start recieving messages
			// as soon as I subscribe.
			for (String topic : topics)
			{
				// subscribe to the topic that sensorListener is publishing to
				node.newSubscriber(topic + "raw", "sensor_msgs/Range", this);
			}

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

		String key = message.header.frame_id;





		if (message.range < message.max_range)
		{
			// if the range received is less than the max range then it is good
			// don't need to do any filtering

			filteredRangeMap.get(key).publish(message);
			prevFilteredRange.put(key, message);

		}
		else
		{
			//else if is equal to the max_range so must filter

			// so I set the filtered range to the smallest value
			// the previous filtered range plus a delta
			// or the max range
			if (prevFilteredRange.get(key) != null)
			{
				message.range = prevFilteredRange.get(key).range + RDelta;
			}
			if (message.range > message.max_range)
			{
				message.range = message.max_range;
			}
			prevFilteredRange.put(key, message);
			// publish the filtered range
			filteredRangeMap.get(key).publish(message);
		}

	}

}
