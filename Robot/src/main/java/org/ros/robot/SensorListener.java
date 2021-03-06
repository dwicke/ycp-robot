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

package main.java.org.ros.robot;

import java.lang.reflect.Field;
import java.util.Iterator;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.TreeMap;

import org.apache.commons.logging.Log;
import org.apache.commons.logging.impl.SimpleLog;
import org.ros.message.MessageListener;
import org.ros.message.Time;
import org.ros.node.DefaultNodeFactory;
import org.ros.node.Node;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMain;
import org.ros.node.parameter.ParameterTree;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;
import org.ros.message.robot_msgs.*;
import org.ros.message.sensor_msgs.Range;

import com.google.common.base.Preconditions;

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
	private SimpleLog log;
	
	
	private Map<String, Publisher<Range> > publisher;
	private Map<String, Integer> sensorAngles;
	private long count;
	private Map<String, Range> pubRange;
	
	@Override
	public void main(NodeConfiguration configuration) {
		Preconditions.checkState(node == null);
	    Preconditions.checkNotNull(configuration);
		//ParameterTreenode.newParameterTree();
		try {
			node = new DefaultNodeFactory().newNode("sensor_listener", configuration);
			pubRange = new TreeMap<String, Range>();
			
					
			log = new SimpleLog(node.getName().toString());
			log.setLevel(SimpleLog.LOG_LEVEL_OFF);
			publisher = new TreeMap<String, Publisher<Range>>();
			count = 0;
			
			Field[] fields = SensorData.class.getDeclaredFields();
			// Loop through all the fields and extract the name of the field
			// to use as the name of the topic
			// NOTE that I may have to add stuff to the name here because of the whole 
			// namespace thing.  Not sure yet how that works.
			
			// get the angle measurements of the sensors from the parameter server
			// in order to put that info into the Range messages
			ParameterTree paramTree = node.newParameterTree();
			sensorAngles = (Map<String, Integer>) paramTree.getMap("sensor_angles");
			
			for (int i = 0; i < fields.length; i++)
			{
				
			//	log.info("The name of the first field is: " + fields[i].getName() + "degree " + sensorAngles.get(fields[i].getName()));
				Publisher<Range> pub = node.newPublisher(fields[i].getName() + "raw", "sensor_msgs/Range");
				Range curRange = (Range) node.getMessageFactory().newMessage("sensor_msgs/Range");
				
				
				if (fields[i].getName().contains("infrared"))
				{
					curRange.max_range = (float) .8; // 80cm
					curRange.min_range = (float) .075; // 7.5cm
					curRange.radiation_type = Range.INFRARED;
					// name it the same as the sensor
					curRange.header.frame_id = fields[i].getName();
					// not right field_of_view but fill it in
					curRange.field_of_view = (float) sensorAngles.get(fields[i].getName());
				}
				else if (fields[i].getName().contains("ultrasonic"))
				{
					curRange.max_range = (float) 2.55; // 255cm
					curRange.min_range = (float) .04; // 4cm
					curRange.radiation_type = Range.ULTRASOUND;
					// not right but fill it in
					curRange.field_of_view = (float) sensorAngles.get(fields[i].getName());
					// name it the same as the sensor
					curRange.header.frame_id = fields[i].getName();
				}
				else if(fields[i].getName().contains("human"))
				{
					curRange.header.frame_id = fields[i].getName();
					curRange.max_range = 4095;
					curRange.min_range = 0;
					curRange.radiation_type = 3;
					curRange.field_of_view = 0;
				}
				
				pubRange.put(fields[i].getName(), curRange);
				
				publisher.put(fields[i].getName(), pub);
			}
			
			node.newSubscriber("sensordata", "robot_msgs/SensorData", this);
			
		} catch (Exception e) {
			if (node != null) {
				node.getLog().fatal(e);
			} else {
				e.printStackTrace();
			}
		}





	}
	
	@SuppressWarnings("rawtypes")
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
				// get the current time for the time stamps
				//Time timeStamp = node.getCurrentTime();
			
				
				if (fieldName.contains("infrared"))
				{
					// get the field data
					float rangeData = (Float) messField.get(message);
					
					pubRange.get(fieldName).range = (float) (rangeData * .01);
					
					// I am using secs in the header to be the key
					// since timestamp secs couldn't keep up (neither could nsecs)
					// I define my own secs in terms of when it leaves here
					pubRange.get(fieldName).header.stamp.secs = (int) count;
					// now publish it!
					publisher.get(fieldName).publish(pubRange.get(fieldName));
				}
				else if (fieldName.contains("ultrasonic"))
				{
					// get field data
					// must do this because java doesn't have unsigned bytes...
					int rangeData = ((Byte) messField.get(message)) & 0xff;
					
					
					
					log.debug(rangeData);
					
					
					
					pubRange.get(fieldName).range = (float) ((float) rangeData * .01);
					
					
					// I am using secs in the header to be the key
					// since timestamp secs couldn't keep up (neither could nsecs)
					// I define my own secs in terms of when it leaves here
					pubRange.get(fieldName).header.stamp.secs = (int) count;
					publisher.get(fieldName).publish(pubRange.get(fieldName));
				}
				else if (fieldName.contains("human"))
				{
					// this outputs the human data as range messages
					
					// get the range
					int rangeData = (Integer) messField.get(message);
					
					// put in the data
					
					pubRange.get(fieldName).range = (float) rangeData;
					
					// I am using secs in the header to be the key
					// since timestamp secs couldn't keep up (neither could nsecs)
					// I define my own secs in terms of when it leaves here
					pubRange.get(fieldName).header.stamp.secs = (int) count;
					
					//log.info("Recieved Message from " + fieldName + " range is " + pubRange.range);
					
					publisher.get(fieldName).publish(pubRange.get(fieldName));
					
					
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
		count++;
		
		
	}

}
