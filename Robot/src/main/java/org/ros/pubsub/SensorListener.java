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

import java.lang.reflect.Field;
import java.util.Iterator;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.TreeMap;

import org.apache.commons.logging.Log;
import org.ros.message.MessageListener;
import org.ros.node.DefaultNodeFactory;
import org.ros.node.Node;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMain;
import org.ros.node.parameter.ParameterTree;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;
import org.ros.message.robot_msgs.*;
import org.ros.message.sensor_msgs.Range;

/**
 * This is a simple rosjava {@link Subscriber} {@link Node}. It assumes an
 * external roscore is already running.  The job of the Robot listener is to
 * listen for messages that have the sensor data in our implementation is from
 * either Converter or from VirtualX80SVP it depends on the startup configuration.
 * 
 * @author drewwicke@google.com (Drew Wicke)
 */
public class SensorListener implements NodeMain, MessageListener<SensorData> {

	private Node node;
	private Log log;
	
	private Map<String, Publisher<Range> > publisher;
	
	
	@Override
	public void main(NodeConfiguration configuration) {

		//ParameterTreenode.newParameterTree();
		try {
			node = new DefaultNodeFactory().newNode("sensor_listener", configuration);
			log = node.getLog();
			publisher = new TreeMap<String, Publisher<Range>>();
			
			
			Field[] fields = SensorData.class.getDeclaredFields();
			// Loop through all the fields and extract the name of the field
			// to use as the name of the topic
			// NOTE that I may have to add stuff to the name here because of the whole 
			// namespace thing.  Not sure yet how that works.
			
			
			
			for (int i = 0; i < fields.length; i++)
			{
				log.info("The name of the first field is: " + fields[i].getName());
				Publisher<Range> pub = node.newPublisher(fields[i].getName() + "raw", "sensor_msgs/Range");
				
				publisher.put(fields[i].getName(), pub);
			}
			
			node.newSubscriber("SensorData", "robot_msgs/SensorData", this);
			
		} catch (Exception e) {
			if (node != null) {
				node.getLog().fatal(e);
			} else {
				e.printStackTrace();
			}
		}





	}
	
	public void publishData(SensorData message)
	{
		// so go through and publish data based on names
		Class theMessage = message.getClass();
		Field messField;
		Set<String> keys = publisher.keySet();
		Iterator<String> itr = keys.iterator();
		while(itr.hasNext())
		{
			String fieldName = itr.next();
			try {
				// get the field name
				messField = theMessage.getField(fieldName);
				
				if (fieldName.contains("infrared"))
				{
					// get the field data
					float rangeData = (Float) messField.get(message);
					
					// now publish it!
					Range IRRange = new Range();
					IRRange.max_range = (float) .8; // 80cm
					IRRange.min_range = (float) .075; // 7.5cm
					IRRange.radiation_type = Range.INFRARED;
					IRRange.range = (float) (rangeData * .01);
					publisher.get(fieldName).publish(IRRange);
				}
				else if (fieldName.contains("ultrasonic"))
				{
					// get field data
					int rangeData = (Integer) messField.get(message);
					Range USRange = new Range();
					USRange.max_range = (float) 2.55; // 255cm
					USRange.min_range = (float) .04; // 4cm
					USRange.radiation_type = Range.ULTRASOUND;
					USRange.range = (float) ((float) rangeData * .01);
					publisher.get(fieldName).publish(USRange);
				}
				else if (fieldName.contains("human"))
				{
					// I will need to come back to this
					// because I need to make a new message
					// type for this sensor
				}
				
				
				
			} catch (SecurityException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			} catch (NoSuchFieldException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			} catch (IllegalArgumentException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			} catch (IllegalAccessException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
			
		}
	}

	@Override
	public void shutdown() {
		node.shutdown();
	} 

	@Override
	public void onNewMessage(SensorData message) {
		
		// Ok so I heard the sensor data so publish the data in ROS format
		// to the specific topics
		publishData(message);
		
	}

}
