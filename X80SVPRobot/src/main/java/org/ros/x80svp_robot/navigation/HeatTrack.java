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

import com.google.common.base.Preconditions;


import org.apache.commons.logging.impl.SimpleLog;
import org.ros.node.DefaultNodeFactory;
import org.ros.node.Node;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMain;
import org.ros.node.topic.Publisher;
import org.ros.message.MessageListener;
import org.ros.message.MotorControlMsg.MotorCommand;
import org.ros.message.robot_msgs.*;
import org.ros.message.rosgraph_msgs.Log;
import org.ros.message.sensor_msgs.Range;

/**
 * This is a simple rosjava {@link Publisher} {@link Node}. It assumes an
 * external roscore is already running. Takes the left and right
 * human sensor data and converts it to left and right wheel velocities.
 * It will orient the robot to face the human target and steer towards it
 * slowing down and speeding up until obstacle avoidance must be engaged 
 * so that the robot won't hit the human.  
 * 
 * @author ethan.rublee@gmail.com (Ethan Rublee)
 * @author damonkohler@google.com (Damon Kohler)
 */
public class HeatTrack implements NodeMain, MessageListener<Range> {

	private Node node;
	private double maxVelocity;
	private Publisher<MotorCommand> pubCmd;
	private MotorCommand mtrCmd;
	private int countLeft, countRight;
	private int midRange;
	private double thresh;
	private double maxAvg;

	private SimpleLog log;
	@Override
	public void main(NodeConfiguration configuration) {
		Preconditions.checkState(node == null);
		Preconditions.checkNotNull(configuration);


		try {
			//get max velocity from parameter server
			node = new DefaultNodeFactory().newNode("heat_track", configuration);
			maxVelocity = node.newParameterTree().getDouble("MAX_LINEAR_VELOCITY");

			log = new SimpleLog(node.getName().toString());
			log.setLevel(SimpleLog.LOG_LEVEL_DEBUG);
			
			
			// set up publisher
			pubCmd = node.newPublisher("Motor_Command", "MotorControlMsg/MotorCommand");

			// set up cmd to be published
			mtrCmd = node.getMessageFactory().newMessage("MotorControlMsg/MotorCommand");

			mtrCmd.isLeftRightVel = true;
			mtrCmd.precedence = 1;
			// set up count 
			countLeft = 0;
			countRight = 0;
			// set the mid range of the sensor
			midRange = 2047;
			// set the threshold for which something is said to be human
			thresh = node.newParameterTree().getDouble("human_thresh");
			// set the max that we expect the range to be
			maxAvg = node.newParameterTree().getDouble("human_range_avg_dif");

			// set up subscriptions
			List<String> topics = (List<String>) node.newParameterTree().getList("human_topics");
			for (String topic : topics)
			{
				// only care about presence
				if (topic.contains("presence"))
				{
					node.newSubscriber(topic + "filtered", "sensor_msgs/Range", this);
				}
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
		node = null;
	}

	@Override
	public void onNewMessage(Range message) {
		// TODO Auto-generated method stub

		log.debug("RECIEVED MESSAGE");
		
		// So I get a range message that describes the 
		if (message.header.frame_id.contains("left"))
		{
			log.debug("Left sensor" + message.range);
			// do left velocity
			if (Math.abs(midRange - message.range) <= thresh)
			{
				// this is left velocity not linear velocity
				mtrCmd.linear_velocity = 0;
			}
			else
			{
				mtrCmd.linear_velocity = Math.abs(midRange - message.range);
			}
			mtrCmd.linear_velocity = (float) (maxVelocity * (1 - (mtrCmd.linear_velocity / maxAvg) ));
			countLeft++;
		}
		else
		{
			log.debug("Right sensor" + message.range);
			// do right velocity
			if (Math.abs(midRange - message.range) <= thresh)
			{
				// this is right wheel velocity not angular velocity
				mtrCmd.angular_velocity = 0;
			}
			else
			{
				mtrCmd.angular_velocity = Math.abs(midRange - message.range);
			}
			mtrCmd.angular_velocity = (float) (maxVelocity * (1 - (mtrCmd.angular_velocity / maxAvg) ));
			countRight++;
		}
		
		if (countLeft > 0 && countRight > 0)
		{
			// there are two sensors so pub once got them both
			countLeft = 0;// reset counter
			countRight = 0; 
			// assign priority
			
			if (mtrCmd.angular_velocity != maxVelocity || mtrCmd.linear_velocity != maxVelocity)
			{
				log.debug("Publishing Heat track");
				pubCmd.publish(mtrCmd);
			}
			else
			{
				log.debug("Not publishing Heat track");
			}
			
		}


	}

}
