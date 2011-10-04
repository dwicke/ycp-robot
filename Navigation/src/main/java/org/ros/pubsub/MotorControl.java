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

import org.apache.commons.logging.Log;
import org.ros.message.MessageListener;
import org.ros.node.DefaultNodeFactory;
import org.ros.node.Node;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMain;
import org.ros.node.topic.Subscriber;
import org.ros.message.MotorControlMsg.MotorCommand;
import org.ros.message.robot_msgs.*;

/**
 * This is a simple rosjava {@link Subscriber} {@link Node}. 
 * I accept a 
 * 
 * @author drewwicke@google.com (Drew Wicke)
 */
public class MotorControl implements NodeMain, MessageListener<MotorCommand> {

	private Node node;
	private double wheelbase;
	@Override
	public void main(NodeConfiguration configuration) {


		try {
			node = new DefaultNodeFactory().newNode("listener", configuration);
			wheelbase = node.newParameterTree().getDouble("wheelbase");
			final Log log = node.getLog();
			node.newSubscriber("MotorCommand", "MotorControlMsg/MotorCommand",this);
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
		MotorData newMsg = new MotorData();
		// Convert the message I heard into left and right wheel velocities
		// VLeft = (2*(LINEAR_VELOCITY) + d(ANGULAR_VELOCITY)) / 2
		// VRight = VLeft - d(ANGULAR_VELOCITY)
		// where d is the wheel base of the robot
		newMsg.motor_left_velocity = (float) (2 * message.linear_velocity + (wheelbase * message.angular_velocity) / 2);
		newMsg.motor_right_velocity = (float) (newMsg.motor_left_velocity - wheelbase * message.angular_velocity);


	}

}
