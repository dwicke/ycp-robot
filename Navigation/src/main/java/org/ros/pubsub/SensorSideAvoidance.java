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
import java.util.Iterator;
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
	//private MessageCollection<Range> mesCollector;
	private MotorCommand mtrCmd;// message to send
	private int curNum, curKey;// info used to sync data transmition
	// these are the received ranges
	private Iterator<Range> ranges;
	// This is the number of inputs I expect to receive before publishing
	private int numberInputs;

	// these are the anglular locations of the sensors
	// remember that the positive y axis is 0 degrees
	// positive x-axis is 90 degrees
	// negative x-axis is -90 degrees
	private Map<String, Double> theta;
	// These are the weights that I multiply the range measurement by
	private Map<String, Double> linearWeight, angularWeight;


	// variance of function
	private double linearVariance, angularVariance;

	private SimpleLog log;

	@SuppressWarnings("unchecked")
	@Override
	public void main(NodeConfiguration configuration) {
		Preconditions.checkState(node == null);
		Preconditions.checkNotNull(configuration);

		try {
			node = new DefaultNodeFactory().newNode("sensor_side_avoidance", configuration);
			log = new SimpleLog(node.getName().toString());
			log.setLevel(SimpleLog.LOG_LEVEL_INFO);
			//log.setLevel(SimpleLog.LOG_LEVEL_DEBUG);
			// get the names of the topics by querying the parameter server
			// based on the name of this node

			@SuppressWarnings("unchecked")
			List<String> topics = (List<String>) node.newParameterTree().getList(node.getName());
			numberInputs = topics.size();
			// get the variance constants
			linearVariance = node.newParameterTree().getDouble("sigma_squared_linear");
			angularVariance = node.newParameterTree().getDouble("sigma_squared_angular");
			log.info("LinearVariance: " + linearVariance + "AngularVariance: " + angularVariance);


			// get the angular distances
			theta = (Map<String, Double>) node.newParameterTree().getMap(node.getName()+"_theta");

			linearWeight = new TreeMap<String, Double>();
			angularWeight = new TreeMap<String, Double>();
			// Make the data structure to hold the messages that I receive.
			// the key is the time stamp of the message.  For each time stamp
			// I have a list of Range objects for each of the sensors.
			//mesCollector = new MessageCollection<Range>(numberInputs);
			curNum = 0;
			curKey = 0;
			mtrCmd = node.getMessageFactory().newMessage("MotorControlMsg/MotorCommand");
			mtrCmd.angular_velocity = (float) 0.0;
			mtrCmd.linear_velocity = (float) 0.0;
			

			// Say name of topic ie. left_IR_Motor_Command
			pubCmd = node.newPublisher(node.getName() + "_Motor_Command", "MotorControlMsg/MotorCommand");




			// Right now I am doing the simple approach and saying that
			// the normalizing constant k in the paper is the same for all 
			// the sensors.  Since the equation is w = 1 = k*sum(e^-(theta^2/(2*sigma^2)) (w = 1 since want normal)
			// I find k by dividing 1 / sum(e^-(theta^2/(2*sigma^2)) = k
			// the same is true for the angular velocity k
			// however later we could weight the center sensor with a higher value and
			// side sensors with lower values...


			// Must make the weights for each of the sensors as per the algorithm
			// linear is sum of 1 / e^(-theta^2 / (2sigma_squared)) of all thetas
			// use the sum of psi to 
			double linearConst = 0.0;
			double angularConst = 0.0;
			for (String key: topics)
			{

				double thetaSquared = theta.get(key) * theta.get(key);
				linearConst += Math.exp(-1.0 * ( thetaSquared) / (2.0 * linearVariance));
				angularConst += Math.exp(-1.0 * ( thetaSquared) / (2.0 * angularVariance)) * (90.0 - Math.abs(theta.get(key)));
			}

			// 1 / sum(e^-(theta^2/(2*sigma^2)) ...
			linearConst = 1.0 / linearConst; 
			angularConst = 1.0 / angularConst;

			double sum = 0.0;

			// Now I can make the linear and angular weights
			for (String key: topics)
			{
				double thetaSquared = theta.get(key) * theta.get(key);
				double linearPsi = Math.exp(-1.0 * ( thetaSquared) / (2.0 * linearVariance));
				double angularPsi = Math.exp(-1.0 * ( thetaSquared) / (2.0 * angularVariance)) * (90.0 - Math.abs(theta.get(key)));
				sum += linearConst * linearPsi + angularConst * angularPsi;
				linearWeight.put(key, linearConst * linearPsi);
				angularWeight.put(key, angularConst * angularPsi);
			}

			log.info("The sum of the sensors is: " + sum);//should be two

			// subscribe to the topics that I am supposed to based on who I am
			// such as all of the left IR filtered sensor data
			for (String topic: topics)
			{
				// I need the filtered data
				node.newSubscriber(topic + "filtered", "sensor_msgs/Range", this);
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
	public void onNewMessage(Range range) {

		int key = range.header.stamp.secs;
		log.debug(key);

		if (key != curKey)
		{
			mtrCmd.angular_velocity = (float) 0.0;
			mtrCmd.linear_velocity = (float) 0.0;
			curNum = 0;
			curKey = key;
		}
		// so now I can do the math
		// the normal of the filtered range * linear_weight
		log.info(range.range / range.max_range + " range " + range.range + " max range" + range.max_range);
		double normalizedSensor = (range.range / range.max_range);
		mtrCmd.linear_velocity += normalizedSensor * linearWeight.get(range.header.frame_id);
		mtrCmd.angular_velocity += (normalizedSensor * angularWeight.get(range.header.frame_id));

		// Once I receive all the messages I can then process them
		curNum++;// finished another
		if (curNum == numberInputs)
		{
			mtrCmd.header.frame_id = node.getName().toString();
			mtrCmd.header.stamp.secs = key;
			curNum = 0;
			pubCmd.publish(mtrCmd);
		}


	}



}

