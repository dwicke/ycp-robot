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

package main.java.org.ros.x80svp_robot.sensorfilters;

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
import org.ros.message.sensor_msgs.Range;

import com.google.common.base.Preconditions;

/**
 * This is a simple rosjava {@link Subscriber} {@link Node}. It assumes an
 * external roscore is already running.  The job of the UltrasonicSensorFilter node
 * is to filter the raw sensor data and publish it.
 * 
 * @author drewwicke@google.com (Drew Wicke)
 */
public class UltrasonicSensorFilter extends SensorFilter {

	
	// there will be a prev... and cur... for each of the US
	private Map<String, Range> prevFilteredRange;
	

	private final float RDelta = (float) .2;// .2 m or 20cm can change if need to just chose this because
	// this is what they had in paper

	
	@Override
	public void main(NodeConfiguration configuration) {

		Preconditions.checkState(node == null);
		Preconditions.checkNotNull(configuration);
		node = new DefaultNodeFactory().newNode("sensor_listener", configuration);
		prevFilteredRange = new TreeMap<String, Range>();
		this.setup("ultrasonic_topics");
	}


	@Override
	public void onNewMessage(Range message) {

		String key = message.header.frame_id;

		if (message.range < message.max_range)
		{
			// if the range received is less than the max range then it is good
			// don't need to do any filtering

			filteredRangeMap.get(key).publish(message);
			prevFilteredRange.put(key, message);

		}
		else
		{
			//else if is equal to the max_range so must filter

			// so I set the filtered range to the smallest value
			// the previous filtered range plus a delta
			// or the max range
			if (prevFilteredRange.get(key) != null)
			{
				message.range = prevFilteredRange.get(key).range + RDelta;
			}
			if (message.range > message.max_range)
			{
				message.range = message.max_range;
			}
			prevFilteredRange.put(key, message);
			
			// publish the filtered range
			filteredRangeMap.get(key).publish(message);
		}

	}

}
