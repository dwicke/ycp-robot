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

import java.util.TreeMap;

import com.google.common.base.Preconditions;

import org.ros.node.DefaultNodeFactory;
import org.ros.node.Node;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMain;
import org.ros.node.parameter.ParameterTree;
import org.ros.node.topic.Publisher;
import org.ros.message.robot_msgs.*;
import org.ros.message.sensor_msgs.Range;

/**
 * This is a simple rosjava {@link Publisher} {@link Node}. It assumes an
 * external roscore is already running.
 * 
 * @author ethan.rublee@gmail.com (Ethan Rublee)
 * @author damonkohler@google.com (Damon Kohler)
 */
public class Talker implements NodeMain {

  private Node node;
  

  @Override
  public void main(NodeConfiguration configuration) {
    Preconditions.checkState(node == null);
    Preconditions.checkNotNull(configuration);
    try {
      node = new DefaultNodeFactory().newNode("talker", configuration);
      
      
      
      ParameterTree tr = node.newParameterTree();
      TreeMap<String, Integer> t = new TreeMap<String, Integer>();
      t.put("Theta1", 3);
      tr.set("test", t);
      
      
      
      
      
      
      
      
      Publisher<MotorData> publisher =
          node.newPublisher("MotorData", "robot_msgs/MotorData");
      
      Publisher<SensorData> pubSense = node.newPublisher("SensorData", "robot_msgs/SensorData");
      
      int seq = 0;
      while (true) {
       // org.ros.message.std_msgs.String str = new org.ros.message.std_msgs.String();
    	  
    	  MotorData motorMessage = new MotorData();
    	  motorMessage.motor_left_velocity = (float) 3.3;
    	  
    	  SensorData d = new SensorData();
    	  d.infrared_frontLeftCenter_distance = (float) 33.43;
    	  d.ultrasonic_frontRight_distance = (byte) 255;
    	  pubSense.publish(d);
    	  
    	  
    	  seq += 1;
        publisher.publish(motorMessage);
        Thread.sleep(1000);
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
    node = null;
  }

}
