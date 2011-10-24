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
import org.ros.message.MessageListener;
import org.ros.node.DefaultNodeFactory;
import org.ros.node.Node;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMain;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;
import org.ros.message.robot_msgs.*;
import org.ros.message.sensor_msgs.Range;

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

	@Override
	public void main(NodeConfiguration configuration) {

		//ParameterTreenode.newParameterTree();
		try {
			node = new DefaultNodeFactory().newNode("sensor_listener", configuration);
			
			
			

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
	public void onNewMessage(Range arg0) {
		// TODO Auto-generated method stub
		
	}

}
