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

import org.apache.commons.logging.Log;
import org.apache.commons.logging.impl.SimpleLog;
import org.ros.message.MessageListener;
import org.ros.node.DefaultNodeFactory;
import org.ros.node.Node;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMain;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;
import org.ros.message.MotorControlMsg.MotorCommand;
import org.ros.message.robot_msgs.*;

import com.google.common.base.Preconditions;

/**
 * This is a simple rosjava {@link Subscriber} {@link Node}. 
 * I accept a 
 * 
 * @author drewwicke@google.com (Drew Wicke)
 */
public class MotorControl implements NodeMain, MessageListener<MotorCommand> {

	private Node node;
	private double wheelbase;
	private Publisher<MotorData> motorData;
	private SimpleLog log; 
	@Override
	public void main(NodeConfiguration configuration) {

		Preconditions.checkState(node == null);
		Preconditions.checkNotNull(configuration);

		try {
			node = new DefaultNodeFactory().newNode("motor_control", configuration);
			wheelbase = node.newParameterTree().getDouble("wheelbase");
			log = new SimpleLog(node.getName().toString());
			//log.setLevel(SimpleLog.LOG_LEVEL_OFF);
			
			motorData = node.newPublisher("motordata", "robot_msgs/MotorData");
			node.newSubscriber("Motor_Command", "MotorControlMsg/MotorCommand",this);
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
		//log.info("I heard: \"" + message.motor_left_velocity + "\"");
		//log.info("I heard: \"" + message.motor_left_velocity + "\"");
		MotorData newMsg = node.getMessageFactory().newMessage("robot_msgs/MotorData");

		if (message.isLeftRightVel == false)
		{
			// Convert the message I heard into left and right wheel velocities
			// VLeft = (2*(LINEAR_VELOCITY) + d(ANGULAR_VELOCITY)) / 2
			// VRight = VLeft - d(ANGULAR_VELOCITY)
			// where d is the wheel base of the robot
			
			newMsg.motor_left_velocity = (float) ((2 * message.linear_velocity + (wheelbase * message.angular_velocity)) / 2) * 100 ;
			newMsg.motor_right_velocity = (float) (newMsg.motor_left_velocity - (wheelbase * message.angular_velocity * 100));
			newMsg.motor_left_time = 55;// based on nav.cpp 1100 is one second  so send it for 1/20 s
			newMsg.motor_right_time = 55;// change later if to slow.
			log.info("MotorData: LeftV: " + newMsg.motor_left_velocity + "  RightV: " + newMsg.motor_right_velocity);
		}
		else
		{
			newMsg.motor_left_velocity = message.linear_velocity * 100;
			newMsg.motor_right_velocity = message.angular_velocity *  100;

		}

		newMsg.motor_left_time = 55;// based on nav.cpp 1100 is one second  so send it for 1/20 s
		newMsg.motor_right_time = 55;// change later if to slow.
		log.info("MotorData: LeftV: " + newMsg.motor_left_velocity + "  RightV: " + newMsg.motor_right_velocity);

		motorData.publish(newMsg);
	}

}
