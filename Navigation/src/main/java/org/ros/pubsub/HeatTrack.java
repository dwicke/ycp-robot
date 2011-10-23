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

import com.google.common.base.Preconditions;

import org.ros.node.DefaultNodeFactory;
import org.ros.node.Node;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMain;
import org.ros.node.topic.Publisher;
import org.ros.message.MessageListener;
import org.ros.message.robot_msgs.*;
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
	// this is the datastructure to synchronize messages
	private MessageCollection<Range> mesCollector;
	// these are the received ranges
	private Iterator<Range> ranges;

	@Override
	public void main(NodeConfiguration configuration) {
		Preconditions.checkState(node == null);
		Preconditions.checkNotNull(configuration);


		try {
			//get max velocity from parameter server
			node = new DefaultNodeFactory().newNode("heat_track", configuration);
			maxVelocity = node.newParameterTree().getDouble("MAX_LINEAR_VELOCITY");
			mesCollector = new MessageCollection<Range>(2);


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

		// So I get a range message that describes the 
		if ((ranges = this.mesCollector.receiveMessage(message, message.header.stamp.secs)) != null)
		{
			
			

		}


	}

}
