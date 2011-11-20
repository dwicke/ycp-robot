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
 * This node tests to make sure I get a message from the SensorSideAvoidance
 * This is used with the TestPublisher
 * 
 * @author drewwicke@google.com (Drew Wicke)
 */
public class SensorSideAvoidanceTestSubscriber implements NodeMain, MessageListener<MotorCommand> {

	private Node node;
	// this is the Object that I use to publish my final motor command
	private Publisher<MotorCommand> pubCmd;
	// this stores the incoming messages until I have all the data
	// needed to compute final MotorCommand to publish
	private Map<Integer, ArrayList<MotorCommand>> inputCommands;
	// This is the number of inputs I expect to receive before publishing
	private int numberInputs;
	private SimpleLog log;

	@Override
	public void main(NodeConfiguration configuration) {
		Preconditions.checkState(node == null);
		Preconditions.checkNotNull(configuration);
		try {
			node = new DefaultNodeFactory().newNode("motor_listener", configuration);
			numberInputs = 2;// Left and right sensor motor control messages


			// Set up the debug log
			log = new SimpleLog(node.getName().toString());
			log.setLevel(SimpleLog.LOG_LEVEL_DEBUG);
			
			
			node.newSubscriber("left_IR_Motor_Command", "MotorControlMsg/MotorCommand", this);



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
		log.debug("got message");

	}



}

