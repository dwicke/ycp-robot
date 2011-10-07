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
/**
 * This is a simple rosjava {@link Subscriber} {@link Node}. 
 * This node is givent the name of the sensor that it is going to 
 * represent.  Since both IR and US both have the same 
 * 
 * @author drewwicke@google.com (Drew Wicke)
 */
public class SensorAvoidance implements NodeMain, MessageListener<MotorCommand> {

	private Node node;
	// this is the Object that I use to publish my final motor command
	private Publisher<MotorCommand> pubCmd;
	// this stores the incoming messages until I have all the data
	// needed to compute final MotorCommand to publish
	private Map<Long, ArrayList<MotorCommand>> inputCommands;
	// This is the number of inputs I expect to receive before publishing
	private int numberInputs;
	private SimpleLog log;

	@Override
	public void main(NodeConfiguration configuration) {
		Preconditions.checkState(node == null);
		Preconditions.checkNotNull(configuration);
		//ParameterTreenode.newParameterTree();
		try {
			node = new DefaultNodeFactory().newNode("motor_listener", configuration);
			numberInputs = 2;// Left and right sensor motor control messages
			inputCommands = new TreeMap<Long, ArrayList<MotorCommand>>();
			// TODO decide how I publish...
			pubCmd = node.newPublisher(node.getName() + "_Motor_Command", "MotorControlMsg/MotorCommand");
			log = new SimpleLog(node.getName().toString());
			log.setLevel(SimpleLog.LOG_LEVEL_DEBUG);
			// The job of this node is to provide to the MotorControler a linear and
			// angular velocity such that the robot avoids obstacles.  It uses
			// Braitenberg's agression behavior and motor fusion.


			// must say who I subscribe to.  So get my subscriptions from the Parameter server 
			@SuppressWarnings("unchecked")
			List<String> topics = (List<String>) node.newParameterTree().getList(node.getName() + "_subscriptions");
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
	public void onNewMessage(MotorCommand message) {
		// TODO Auto-generated method stub

		// Combine the left and the right sides to get the linear
		// and the angular normalized velocity for this particular sensor
		// which is defined by the name given it when created.


	//	int key = message.header.stamp.nsecs;
		long key = message.header.seq;
		log.debug("Recieved message with key: " + key);
		
		if(inputCommands.containsKey(key) && inputCommands.get(key).size() == numberInputs - 1)
		{
			// Has the key and there are enough keys
			// all the input I need so get it and remove the key and publish

			MotorCommand mtrCmd = new MotorCommand();

			// So I need to combine the two sides
			// add the left side to the right for the linear
			// and subtract the left side from the right
			// for the angular

			for (MotorCommand cmd: inputCommands.get(key))
			{
				// Might need to multiply by .5 so that I get Sum(w) and Sum(D(theta')*w) = 1
				// since I am going to be doing only the right or left sides...
				mtrCmd.linear_velocity += cmd.linear_velocity;
				if (cmd.header.frame_id.contains("left"))
				{
					mtrCmd.angular_velocity -= cmd.angular_velocity;
				}
			}

			
			inputCommands.remove(key);
			// remember to set the FrameID of the header to that of
			// either "ultrasonic_avoid" or "infrared_avoid" as in the yaml file
			// so that ObstacleAvoidance can get the appropriate constant
			// to wait that type of info
			mtrCmd.header.frame_id = node.getName().toString();
			mtrCmd.header.stamp = node.getCurrentTime();
			mtrCmd.header.seq = key;
			pubCmd.publish(mtrCmd);

		}
		else if (!inputCommands.containsKey(key))
		{
			// no key present so add it and the message
			ArrayList<MotorCommand> newList = new ArrayList<MotorCommand>();
			newList.add(message);
			inputCommands.put(key, newList);
		}
		else
		{
			// has the key but I haven't received enough messages.
			// so add the message to the list
			inputCommands.get(key).add(message);
		}


	}



}

