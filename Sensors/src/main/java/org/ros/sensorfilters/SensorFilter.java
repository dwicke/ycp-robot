package main.java.org.ros.sensorfilters;

import java.util.List;
import java.util.TreeMap;

import org.apache.commons.logging.impl.SimpleLog;
import org.ros.message.MessageListener;
import org.ros.message.sensor_msgs.Range;
import org.ros.node.DefaultNodeFactory;
import org.ros.node.Node;
import org.ros.node.NodeMain;
import org.ros.node.topic.Publisher;

public abstract class SensorFilter implements NodeMain, MessageListener<Range>{
	protected Node node;
	protected TreeMap<String, Publisher<Range> > filteredRangeMap;
	protected SimpleLog log;

	
	protected void setup(String topicListName)
	{
		try{
			log = new SimpleLog(node.getName().toString());
			log.setLevel(SimpleLog.LOG_LEVEL_INFO);
			//log.setLevel(SimpleLog.LOG_LEVEL_OFF);
			// Print out the name
			log.info("Sensor name: " + node.getName());

			filteredRangeMap = new TreeMap<String, Publisher<Range>>();


			// get the names of the sensors for the IR sensors so I can make
			// the name of the topic I am going to publish to 
			List<String> topics = (List<String>) node.newParameterTree().getList(topicListName);
			for (String topic : topics)
			{
				Publisher<Range> filteredRange =
						node.newPublisher(topic + "filtered", "sensor_msgs/Range");
				// make the new publisher and add it to the map
				filteredRangeMap.put(topic, filteredRange);
			}



			// make this node a subscriber to the Range messages that are being published
			// by SensorListener.
			// must do this after I create the publishers because I start recieving messages
			// as soon as I subscribe.
			for (String topic : topics)
			{
				// subscribe to the topic that sensorListener is publishing to
				node.newSubscriber(topic + "raw", "sensor_msgs/Range", this);
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
	}

	@Override
	public void onNewMessage(Range message) {
		// For now just forward the raw message along.  Later add a filter
		//Range newMess = message.clone();
		///newMess.header = message.header;
		//log.info(newMess.header.seq + "  " + message.header.seq);

		filteredRangeMap.get(message.header.frame_id).publish(message);

	}


}
