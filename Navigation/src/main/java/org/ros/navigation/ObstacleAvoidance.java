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

package main.java.org.ros.navigation;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import java.util.Map;
import java.util.Set;
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

	// This is the number of inputs I expect to receive before publishing
	private int numberInputs;
	private int curNum;
	private int curKey;
	private MotorCommand mtrCmd;
	// This Map is the array of K values that I multiply
	// the incoming data by.
	// private M These values I can get from the parameter sever
	// since they are constant.
	private Map<String, Double> constants;
	// max linear and angular velocities 
	private double maxLinearVelocity, maxAngularVelocity;

	private SimpleLog log;
	// this stores the incoming messages until I have all the data
	// needed to compute final MotorCommand to publish
	float lin, ang;
	//private MessageCollection<MotorCommand> mesCollector;
	// this is the list of motorcommands
	private Iterator<MotorCommand> cmds;

	@SuppressWarnings("unchecked")
	@Override
	public void main(NodeConfiguration configuration) {
		Preconditions.checkState(node == null);
		Preconditions.checkNotNull(configuration);
		try {
			node = new DefaultNodeFactory().newNode("motor_listener", configuration);
			numberInputs = 2;// IR and US think of moving to parameter server.

			//mesCollector = new MessageCollection<MotorCommand>(numberInputs);
			mtrCmd = node.getMessageFactory().newMessage("MotorControlMsg/MotorCommand");
			curNum = 0;
			curKey = 0;

			// get the max linear and angular velocity so that I can later compute the
			// real velocities from the normalized velocities.
			maxLinearVelocity = node.newParameterTree().getDouble("MAX_LINEAR_VELOCITY");
			maxAngularVelocity = node.newParameterTree().getDouble("MAX_ANGULAR_VELOCITY");
			// publish to Motor_Command
			pubCmd = node.newPublisher("Motor_Command", "MotorControlMsg/MotorCommand");
			log = new SimpleLog(node.getName().toString());
			//log.setLevel(SimpleLog.LOG_LEVEL_DEBUG);
			log.setLevel(SimpleLog.LOG_LEVEL_OFF);

			// The job of this node is to provide to the MotorControler a linear and
			// angular velocity such that the robot avoids obstacles.  It uses
			// Braitenberg's Aggression behavior and motor fusion.
			// These constants sum to 1 and provide the percentage that each sensor type contributes to the final
			// motorcommand
			constants = (Map<String, Double>) node.newParameterTree().getMap("comb_const");


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

		if (curKey != message.header.stamp.secs)
		{
			curKey = message.header.stamp.secs;
			curNum = 0;
			mtrCmd.angular_velocity = 0;
			mtrCmd.linear_velocity = 0;
		}



		// calculate the angular and the linear velocity
		// convert alpha(normalized linear velocity) and beta(normalized angular velocity)
		// into V and W (linear velocity and angular velocity)
		// by doing alpha * MAX_LINEAR_VELOCITY = linear velocity
		// beta * MAX_ANGULAR_VELOCITY = angular velocity.
		// and then also multiply that by the normalizing constant
		log.debug("The constant is " + constants.get(message.header.frame_id.substring(1)));
		mtrCmd.angular_velocity += message.angular_velocity * maxAngularVelocity * constants.get(message.header.frame_id.substring(1));
		mtrCmd.linear_velocity  += message.linear_velocity * maxLinearVelocity * constants.get(message.header.frame_id.substring(1));
		curNum++;// done another

		log.debug("Received both IR and US key: " + message.header.stamp.secs + " Size of cmds = " + curNum);
		if (curNum == numberInputs)// check to see if done all
		{
			// and publish that
			log.debug("Angular = " + mtrCmd.angular_velocity + " Linear= " + mtrCmd.linear_velocity);
			mtrCmd.precedence = (mtrCmd.angular_velocity > (maxAngularVelocity / 40)) ? 0 : 5;// highest priority if turning else not so much
			
			log.debug("The precedence is = " + mtrCmd.precedence + " mtrCmd.angular_velocity = " + mtrCmd.angular_velocity + " ratio = " + (maxAngularVelocity / 40));
			
			mtrCmd.header.frame_id = node.getName().toString();// name
			mtrCmd.header.stamp = node.getCurrentTime();// time sent
			curNum = 0;
			// publish the motor command
			pubCmd.publish(mtrCmd);
		}


	}



}

