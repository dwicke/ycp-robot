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
	private Map<Integer, ArrayList<MotorCommand>> inputCommands;
	// This is the number of inputs I expect to receive before publishing
	private int numberInputs;
	
	@Override
	public void main(NodeConfiguration configuration) {
		Preconditions.checkState(node == null);
	    Preconditions.checkNotNull(configuration);
		//ParameterTreenode.newParameterTree();
		try {
			node = new DefaultNodeFactory().newNode("motor_listener", configuration);
			numberInputs = 2;// IR and US
			inputCommands = new TreeMap<Integer, ArrayList<MotorCommand>>();
			// publish to Motor_Command
			pubCmd = node.newPublisher("Motor_Command", "MotorControlMsg/MotorCommand");
			final Log log = node.getLog();
			
			// The job of this node is to provide to the MotorControler a linear and
			// angular velocity such that the robot avoids obstacles.  It uses
			// Braitenberg's agression behavior and motor fusion.
			
			
			

			
			
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
		if(inputCommands.containsKey(key) && inputCommands.get(key).size() == numberInputs)
		{
			// Has the key and there are enough keys
			// all the input I need so get it and remove the key
			
			
			
		}
		else if (!inputCommands.containsKey(key))
		{
			// no key present so add it and the message
			ArrayList<MotorCommand> newList = new ArrayList<MotorCommand>();
			newList.add(message);
			inputCommands.put(message.header.stamp.secs, newList);
		}
		else
		{
			// has the key but I haven't received enough messages.
		}
		
		
	}



}

