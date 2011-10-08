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

import java.util.ArrayList;
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
import com.google.common.base.Preconditions;
import org.ros.message.MotorControlMsg.MotorCommand;
import org.ros.message.sensor_msgs.Range;
/**
 * This is a simple rosjava {@link Subscriber} {@link Node}. 
 * Basically this node is named so that it subscribes to the correct
 * data ie IRLeft etc and so it will publish that way
 * 
 * @author drewwicke@google.com (Drew Wicke)
 */
public class SensorSideAvoidance implements NodeMain, MessageListener<Range> {

	private Node node;
	// this is the Object that I use to publish my final motor command
	private Publisher<MotorCommand> pubCmd;
	// this stores the incoming messages until I have all the data
	// needed to compute final MotorCommand to publish
	private Map<Integer, ArrayList<Range>> inputCommands;
	// This is the number of inputs I expect to receive before publishing
	private int numberInputs;

	// variance of function
	private double linearVariance, angularVariance;
	private Log log;
	private SimpleLog l;

	@Override
	public void main(NodeConfiguration configuration) {
		Preconditions.checkState(node == null);
		Preconditions.checkNotNull(configuration);
		//ParameterTreenode.newParameterTree();
		try {
			node = new DefaultNodeFactory().newNode("sensor_side_avoidance", configuration);

			// get the names of the topics by querying the parameter server
			// based on the name of this node

			@SuppressWarnings("unchecked")
			List<String> topics = (List<String>) node.newParameterTree().getList(node.getName());
			numberInputs = topics.size();
			// get the variance constants
			linearVariance = node.newParameterTree().getDouble("sigma_squared_linear");
			angularVariance = node.newParameterTree().getDouble("sigma_squared_angular");

			// Make the data structure to hold the messages that I recieve.
			// the key is the time stamp of the message.  For each time stamp
			// I have a list of Range objects for each of the sensors.
			inputCommands = new TreeMap<Integer, ArrayList<Range>>();


			// Say name of topic ie. left_IR_Motor_Command
			pubCmd = node.newPublisher(node.getName() + "_Motor_Command", "MotorControlMsg/MotorCommand");
			log = node.getLog();

			l = new SimpleLog(node.getName().toString());
		//	l.setLevel(SimpleLog.LOG_LEVEL_DEBUG);


			// subscribe to the topics that I am supposed to based on who I am
			// such as all of the left IR filtered sensor data
			for (String topic: topics)
			{
				// I need the filtered data
				node.newSubscriber(topic + "filtered", "sensor_msgs/Range", this);
			}


			// Must make the weights for each of the sensors as per the algorithm

			



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

		// Gets the range messages from the Left or right depending on name of this node
		// and when I get all of the messages I do math based on algorithm to get the value for that side.


		// I am using secs in the header to be the key
		// since timestamp secs couldn't keep up (neither could nsecs)
		// I define my own secs in terms of when it leaves the sensorListener
		// in the robot package.
		int key = message.header.stamp.secs;
		
		if (node.getName().toString().contains("left_IR"))
		{
			l.debug("Got a message. Key:" + key + " numberInputs: " + numberInputs);


			//	log.info("Key: " + key);
			//log.info("From: " + message.header.frame_id + " to: " + node.getName() + ":  " + key);

			l.debug("inputCommands contains the key " + inputCommands.containsKey(key) + "  And the number of " +
					"messages in the array is " + ((inputCommands.containsKey(key)) ? inputCommands.get(key).size() : 0));
		}
		if(inputCommands.containsKey(key) && inputCommands.get(key).size() == numberInputs - 1)
		{
			// Has the key and there are enough keys
			// all the input I need so get it and remove the key
			// 


			l.debug("Got enough messages publishing\n\n\n\n\n\n");
			MotorCommand mtrCmd = new MotorCommand();

			// get the range data
			ArrayList<Range> ranges = inputCommands.get(key);
			ranges.add(message);
			for (Range range : ranges)
			{
				// so now I can do the math
				// the normal of the filtered range * linear_weight
				double normalizedSensor = (range.range / range.max_range);
				//	mtrCmd.linear_velocity += normalizedSensor * linearWeight
				//mtrCmd.angular_velocity += (normalizedSensor * angularWeight)
			}

			mtrCmd.header.frame_id = node.getName().toString();
			mtrCmd.header.stamp.secs = key;
			//mtrCmd.header.seq = key;

			inputCommands.remove(key);
			pubCmd.publish(mtrCmd);
		}
		else if (!inputCommands.containsKey(key))
		{
			// no key present so add it and the message
			ArrayList<Range> newList = new ArrayList<Range>();
			l.debug("New List");
			newList.add(message);
			inputCommands.put(key, newList);
		}
		else
		{
			// has the key but I haven't received enough messages.
			// so add the message to the list
			if (node.getName().toString().contains("left_IR"))
			{
				l.debug("Added " + inputCommands.get(key).size() + " message.");
			}
			inputCommands.get(key).add(message);
		}


	}



}

