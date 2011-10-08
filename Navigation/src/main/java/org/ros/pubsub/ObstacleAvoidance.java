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
 * This is a simple rosjava {@link Subscriber} {@link Node}. It assumes an
 * external roscore is already running.  The job of this controller class is 
 * to 
 * 
 * @author drewwicke@google.com (Drew Wicke)
 */
public class ObstacleAvoidance implements NodeMain, MessageListener<MotorCommand> {

	private Node node;
	// this is the Object that I use to publish my final motor command
	private Publisher<MotorCommand> pubCmd;
	// this stores the incoming messages until I have all the data
	// needed to compute final MotorCommand to publish
	private Map<Integer, ArrayList<MotorCommand>> inputCommands;
	// This is the number of inputs I expect to receive before publishing
	private int numberInputs;
	// This Map is the array of K values that I multiply
	// the incoming data by.
	// private M These values I can get from the parameter sever
	// since they are constant.
	private Map<String, Double> constants;
	// max linear and angular velocities 
	private double maxLinearVelocity, maxAngularVelocity;

	@SuppressWarnings("unchecked")
	@Override
	public void main(NodeConfiguration configuration) {
		Preconditions.checkState(node == null);
		Preconditions.checkNotNull(configuration);
		try {
			node = new DefaultNodeFactory().newNode("motor_listener", configuration);
			numberInputs = 2;// IR and US think of moving to parameter server.
			inputCommands = new TreeMap<Integer, ArrayList<MotorCommand>>();
			// get the max linear and angular velocity so that I can later compute the
			// real velocities from the normalized velocities.
			maxLinearVelocity = node.newParameterTree().getDouble("MAX_LINEAR_VELOCITY");
			maxAngularVelocity = node.newParameterTree().getDouble("MAX_ANGULAR_VELOCITY");
			// publish to Motor_Command
			pubCmd = node.newPublisher("Motor_Command", "MotorControlMsg/MotorCommand");
			final Log log = node.getLog();

			// The job of this node is to provide to the MotorControler a linear and
			// angular velocity such that the robot avoids obstacles.  It uses
			// Braitenberg's Aggression behavior and motor fusion.
			// These constants sum to 1 and provide the percentage that each sensor type contributes to the final
			// motorcommand
			constants = (Map<String, Double>) node.newParameterTree().getMap("comb_const");
			if (constants == null)
			{
				log.info("Constants was null");
				
			}

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
		//combines US and IR into a single motor_cmd message
		// that is published here

		// if I get two messages with the same time stamp then I can
		// assume that they are the IR and the US and move on
		// this is not general.
		int key = message.header.stamp.secs;
		//long key = message.header.seq;
		if(inputCommands.containsKey(key) && inputCommands.get(key).size() == numberInputs - 1)
		{
			// Has the key and there are enough keys
			// all the input I need so get it and remove the key


			ArrayList<MotorCommand> cmds = inputCommands.get(key);
			cmds.add(message);
			MotorCommand mtrCmd = new MotorCommand();
			for (MotorCommand cmd: cmds)
			{
				// calculate the angular and the linear velocity
				// convert alpha(normalized linear velocity) and beta(normailzied angular velocity)
				// into V and W (linear velocity and angular velocity)
				// by doing alpha * MAX_LINEAR_VELOCITY = linear velocity
				// beta * MAX_ANGULAR_VELOCITY = angular velocity.
				// and then also multiply that by the normalizing constant
				mtrCmd.angular_velocity += cmd.angular_velocity * maxAngularVelocity * constants.get(mtrCmd.header.frame_id);
				mtrCmd.linear_velocity  += cmd.linear_velocity * maxLinearVelocity * constants.get(mtrCmd.header.frame_id);
			}


			// and publish that
			mtrCmd.precedence = 0;// highest priority
			mtrCmd.header.frame_id = node.getName().toString();// name
			mtrCmd.header.stamp = node.getCurrentTime();// time sent
			// publish the motor command
			pubCmd.publish(mtrCmd);

			inputCommands.remove(key);
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

