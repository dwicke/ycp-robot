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
/**
 * This is a simple rosjava {@link Subscriber} {@link Node}. 
 * the purpose of this node is to combine the two sides of the sensor
 * arrays from SensorSideAvoidance and publish a motor command to ObstacleAvoidance
 * 
 * @author drewwicke@google.com (Drew Wicke)
 */
public class SensorAvoidance implements NodeMain, MessageListener<MotorCommand> {

	private Node node;
	// this is the Object that I use to publish my final motor command
	private Publisher<MotorCommand> pubCmd;


	// this stores the incoming messages until I have all the data
	// needed to compute final MotorCommand to publish
	//private MessageCollection<MotorCommand> mesCollector;

	// this is the list of motorcommands
	private Iterator<MotorCommand> cmds;

	// This is the number of inputs I expect to receive before publishing
	private int numberInputs;// the number of topics i am going to subscribe to
	private SimpleLog log;// log for debugging
	private MotorCommand mtrCmd;// message to send to ObstacleAvoidance
	private int key;// the key of the received message
	private int curKey, curNum;

	@Override
	public void main(NodeConfiguration configuration) {
		Preconditions.checkState(node == null);
		Preconditions.checkNotNull(configuration);
		try {
			node = new DefaultNodeFactory().newNode("motor_listener", configuration);

			mtrCmd = node.getMessageFactory().newMessage("MotorControlMsg/MotorCommand");
			// must say who I subscribe to.  So get my subscriptions from the Parameter server 
			@SuppressWarnings("unchecked")
			List<String> topics = (List<String>) node.newParameterTree().getList(node.getName() + "_subscriptions");

			numberInputs = topics.size();// Left and right sensor motor control messages should be two one left and one right

			//mesCollector = new MessageCollection<MotorCommand>(numberInputs);
			curNum = 0;
			curKey = 0;

			pubCmd = node.newPublisher(node.getName() + "_Motor_Command", "MotorControlMsg/MotorCommand");


			// Set up the debug log
			log = new SimpleLog(node.getName().toString());
			//log.setLevel(SimpleLog.LOG_LEVEL_DEBUG);
			// The job of this node is to provide to the MotorControler a linear and
			// angular velocity such that the robot avoids obstacles.  It uses
			// Braitenberg's agression behavior and motor fusion.



			for (String topic : topics)
			{
				node.newSubscriber(topic, "MotorControlMsg/MotorCommand", this);
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
	public void onNewMessage(MotorCommand cmd) {
		// TODO Auto-generated method stub

		// Combine the left and the right sides to get the linear
		// and the angular normalized velocity for this particular sensor
		// which is defined by the name given it when created.

		// I am using secs in the header to be the key
		// since timestamp secs couldn't keep up (neither could nsecs)
		// I define my own secs in terms of when it leaves the sensorListener
		// in the robot package.
		key = cmd.header.stamp.secs;
		//long key = message.header.seq;
		log.debug("Received message with key: " + key);

		// Receive the message and check if 

		// Has the key and there are enough keys
		// all the input I need so get it and remove the key and publish
		if (key != curKey)
		{
			mtrCmd.linear_velocity = (float) 0.0;
			mtrCmd.angular_velocity = (float) 0.0;
			curKey = key;
			curNum = 0;
		}
		// So I need to combine the two sides
		// add the left side to the right for the linear
		// and subtract the left side from the right
		// for the angular

		// Might need to multiply by .5 so that I get Sum(w) and Sum(D(theta')*w) = 1
		// since I am going to be doing only the right or left sides...
		mtrCmd.linear_velocity += cmd.linear_velocity;
		// the precondition is that the frame_id states in it somewhere that it is the left
		// side.
		if (cmd.header.frame_id.contains("left"))
		{// if left I subtract
			mtrCmd.angular_velocity -= cmd.angular_velocity;
		}
		else
		{// if right i add
			mtrCmd.angular_velocity += cmd.angular_velocity;
		}

		curNum++;// finished another
		if (curNum == numberInputs)
		{
			// Normalize the values
			mtrCmd.linear_velocity /= (numberInputs * numberInputs);
			mtrCmd.angular_velocity /= (numberInputs * numberInputs);

			// zero count
			curNum = 0;


			// remember to set the FrameID of the header to that of
			// either "ultrasonic_avoid" or "infrared_avoid" as in the yaml file
			// so that ObstacleAvoidance can get the appropriate constant
			// to wait that type of info
			// don't want the / in front of the name
			mtrCmd.header.frame_id = node.getName().toString().replace("/", "");
			// set the key
			mtrCmd.header.stamp.secs = key;
			// publish
			pubCmd.publish(mtrCmd);
		}

	}



}

