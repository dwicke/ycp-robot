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

package main.java.org.ros.x80svp_robot.navigation;

import java.util.List;

import org.apache.commons.logging.Log;
import org.ros.message.MessageListener;
import org.ros.node.DefaultNodeFactory;
import org.ros.node.Node;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMain;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;
import org.ros.message.MotorControlMsg.MotorCommand;
import org.ros.message.robot_msgs.*;
import org.ros.message.sensor_msgs.Range;

import com.google.common.base.Preconditions;

/**
 * This is a simple rosjava {@link Subscriber} {@link Node}. It assumes an
 * external roscore is already running.  
 * 
 * this performs a pivot maneuver
 * 
 * @author drewwicke@google.com (Drew Wicke)
 */
public class HeatSearch implements NodeMain, MessageListener<Range> {

	private Node node;
	private Publisher<MotorCommand> mtrPub;
	private MotorCommand mtrCmd;
	private int countRange, countSent;
	private double thresh;// the threshold of the human sensor
	private double maxCount;
	@Override
	public void main(NodeConfiguration configuration) {
		Preconditions.checkState(node == null);
		Preconditions.checkNotNull(configuration);
		
		try {
			node = new DefaultNodeFactory().newNode("sensor_listener", configuration);
			mtrCmd = node.getMessageFactory().newMessage("MotorControlMsg/MotorCommand");
			mtrCmd.isLeftRightVel = false;// since turning might as well use angular
			mtrCmd.linear_velocity = 0;
			mtrCmd.angular_velocity = (float) 1.0;// 1 rad/s 57deg per sec

			mtrCmd.precedence = 1;
			thresh = node.newParameterTree().getDouble("human_thresh");
			maxCount = 5;

			mtrPub = node.newPublisher("Motor_Command", "MotorControlMsg/MotorCommand");


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
	}



	@Override
	public void onNewMessage(Range range) {
		// TODO Auto-generated method stub
		// if the ranges have been not above the threshold for 4 recieved
		// messages then send out a turn command
		if (Math.abs(range.range - (range.max_range / 2)) <= thresh)
		{
			// if did not see anything
			countRange++;
		}
		else
		{
			countRange = 0;
		}
		if (countRange >= maxCount)
		{
			if (countRange < maxCount * 4)
			{
				// so only publish until 4 * the max
				// so don't keep circling
				mtrPub.publish(mtrCmd);
			}
			else 
			{
				countRange = 0;
			}
		}

	}

}
