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

import java.awt.image.FilteredImageSource;
import java.util.ArrayList;
import java.util.List;
import java.util.TreeMap;

import org.apache.commons.logging.Log;
import org.ros.message.MessageListener;
import org.ros.node.DefaultNodeFactory;
import org.ros.node.Node;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMain;
import org.ros.node.parameter.ParameterTree;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;
import org.ros.message.robot_msgs.*;
import org.ros.message.sensor_msgs.Range;

import com.google.common.base.Preconditions;
import com.sun.net.httpserver.Filter;

/**
 * This is a simple rosjava {@link Subscriber} {@link Node}. It assumes an
 * external roscore is already running.  The job of this node is to filter the IRSensor
 * data and publish it.
 * 
 * @author drewwicke@google.com (Drew Wicke)
 */
public class IRSensor implements NodeMain, MessageListener<Range> {

	private Node node;
	private TreeMap<String, Publisher<Range> > filteredRangeMap;
	private Log log;

	@Override
	public void main(NodeConfiguration configuration) {
		Preconditions.checkState(node == null);
		Preconditions.checkNotNull(configuration);

		try {
			// the name of the node gets changed when it is created... so not "sensor_listener"
			node = new DefaultNodeFactory().newNode("sensor_listener", configuration);

			log = node.getLog();
			// Print out the name
			log.info("Sensor name: " + node.getName());

			filteredRangeMap = new TreeMap<String, Publisher<Range>>();


			// get the names of the sensors for the IR sensors so I can make
			// the name of the topic I am going to publish to 
			List<String> topics = (List<String>) node.newParameterTree().getList("infrared_topics");
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
		// For now just forward the raw message along.  Later add a filter
		//Range newMess = message.clone();
		///newMess.header = message.header;
		//log.info(newMess.header.seq + "  " + message.header.seq);
		filteredRangeMap.get(message.header.frame_id).publish(message);

	}

}
