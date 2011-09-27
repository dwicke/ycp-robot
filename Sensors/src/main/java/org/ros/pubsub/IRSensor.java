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
import org.ros.message.robot_msgs.*;

/**
 * This is a simple rosjava {@link Subscriber} {@link Node}. It assumes an
 * external roscore is already running.  The job of the Robot listener is to
 * listen for messages that have the sensor data in our implementation is from
 * either Converter or from VirtualX80SVP it depends on the startup configuration.
 * 
 * @author drewwicke@google.com (Drew Wicke)
 */
public class IRSensor implements NodeMain {

  private Node node;

  @Override
  public void main(NodeConfiguration configuration) {
	  
	  
    try {
      node = new DefaultNodeFactory().newNode("listener", configuration);
      
      final Log log = node.getLog();
      node.newSubscriber("MotorData", "robot_msgs/MotorData",
          new MessageListener<MotorData>() {
    	  
            @Override
            public void onNewMessage(MotorData message) {
              log.info("I heard: \"" + message.motor_left_velocity + "\"");
              log.info("I am " + node.getName() + "\n");
            }
          });
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

}
