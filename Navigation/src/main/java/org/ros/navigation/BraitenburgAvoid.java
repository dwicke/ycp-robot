package main.java.org.ros.navigation;

import java.util.Iterator;
import java.util.List;
import java.util.Map;
import java.util.TreeMap;

import org.apache.commons.logging.impl.SimpleLog;
import org.ros.message.MessageListener;
import org.ros.message.MotorControlMsg.MotorCommand;
import org.ros.message.sensor_msgs.Range;
import org.ros.node.DefaultNodeFactory;
import org.ros.node.Node;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMain;
import org.ros.node.topic.Publisher;

import com.google.common.base.Preconditions;

/**
 * create one for infrared and one for ultrasonic
 * publish a motor command to obstacleAvoidance
 * which will combine the ir and us. and 
 * send to motor control
 */
public class BraitenburgAvoid implements NodeMain, MessageListener<Range> {
	private Node node;
	// this is the Object that I use to publish my final motor command
	private Publisher<MotorCommand> pubCmd;
	// this stores the incoming messages until I have all the data
	// needed to compute final MotorCommand to publish
	//private MessageCollection<Range> mesCollector;
	private MotorCommand mtrCmd;// message to send
	private int curNum, curKey;// info used to sync data transmition
	// these are the received ranges
	private Iterator<Range> ranges;
	// This is the number of inputs I expect to receive before publishing
	private int numberInputs;

	// these values I use to know how to normalize the lin and ang velocity
	private int numAngInput, numLinInput;

	// these are the anglular locations of the sensors
	// remember that the positive y axis is 0 degrees
	// positive x-axis is 90 degrees
	// negative x-axis is -90 degrees
	private Map<String, Double> theta;
	// These are the weights that I multiply the range measurement by
	private Map<String, Double> linearWeight, angularWeight;


	// variance of function
	private double linearVariance, angularVariance;

	private SimpleLog log;



	@SuppressWarnings("unchecked")
	@Override
	public void main(NodeConfiguration configuration) {
		Preconditions.checkState(node == null);
		Preconditions.checkNotNull(configuration);

		try {
			node = new DefaultNodeFactory().newNode("sensor_side_avoidance", configuration);
			log = new SimpleLog(node.getName().toString());
			//log.setLevel(SimpleLog.LOG_LEVEL_INFO);
			//log.setLevel(SimpleLog.LOG_LEVEL_OFF);
			log.setLevel(SimpleLog.LOG_LEVEL_DEBUG);
			// get the names of the topics by querying the parameter server
			// based on the name of this node

			@SuppressWarnings("unchecked")
			List<String> topics = (List<String>) node.newParameterTree().getList(node.getName() + "_subscriptions");
			numberInputs = topics.size();
			// get the variance constants
			linearVariance = node.newParameterTree().getDouble("sigma_squared_linear");
			angularVariance = node.newParameterTree().getDouble("sigma_squared_angular");
			log.info("LinearVariance: " + linearVariance + "AngularVariance: " + angularVariance);


			// get the angular distances
			theta = (Map<String, Double>) node.newParameterTree().getMap(node.getName()+"_theta");

			linearWeight = new TreeMap<String, Double>();
			angularWeight = new TreeMap<String, Double>();
			// Make the data structure to hold the messages that I receive.
			// the key is the time stamp of the message.  For each time stamp
			// I have a list of Range objects for each of the sensors.
			//mesCollector = new MessageCollection<Range>(numberInputs);
			curNum = 0;
			curKey = 0;
			numAngInput = 1;
			numLinInput = 1;
			mtrCmd = node.getMessageFactory().newMessage("MotorControlMsg/MotorCommand");
			mtrCmd.angular_velocity = (float) 0.0;
			mtrCmd.linear_velocity = (float) 0.0;
			mtrCmd.header.frame_id = node.getName().toString();

			// Say name of topic ie. left_IR_Motor_Command
			pubCmd = node.newPublisher(node.getName() + "_Motor_Command", "MotorControlMsg/MotorCommand");




			// Right now I am doing the simple approach and saying that
			// the normalizing constant k in the paper is the same for all 
			// the sensors.  Since the equation is w = 1 = k*sum(e^-(theta^2/(2*sigma^2)) (w = 1 since want normal)
			// I find k by dividing 1 / sum(e^-(theta^2/(2*sigma^2)) = k
			// the same is true for the angular velocity k
			// however later we could weight the center sensor with a higher value and
			// side sensors with lower values...


			// Must make the weights for each of the sensors as per the algorithm
			// linear is sum of 1 / e^(-theta^2 / (2sigma_squared)) of all thetas
			// use the sum of psi to 
			double linearConst = 0.0;
			double angularConst = 0.0;
			for (String key: topics)
			{
				if (!key.contains("frontCenter"))
				{
					double thetaSquared = theta.get(key) * theta.get(key);
					linearConst += Math.exp(-1.0 * ( thetaSquared) / (2.0 * linearVariance));
					angularConst += Math.exp(-1.0 * ( thetaSquared) / (2.0 * angularVariance)) * (90.0 - Math.abs(theta.get(key)));

				}
			}
			// 1 / sum(e^-(theta^2/(2*sigma^2)) ...
			linearConst = 1.0 / linearConst; 
			angularConst = 1.0 / angularConst;

			double sum = 0.0;

			// Now I can make the linear and angular weights
			for (String key: topics)
			{
				//for now I can't do frontCenter
				if (!key.contains("frontCenter"))
				{
					double thetaSquared = theta.get(key) * theta.get(key);
					double linearPsi = Math.exp(-1.0 * ( thetaSquared) / (2.0 * linearVariance));
					double angularPsi = Math.exp(-1.0 * ( thetaSquared) / (2.0 * angularVariance)) * (90.0 - Math.abs(theta.get(key)));
					sum += linearConst * linearPsi + angularConst * angularPsi;
					linearWeight.put(key, linearConst * linearPsi);
					angularWeight.put(key, angularConst * angularPsi);
				}
			}

			log.info("The sum of the sensors is: " + sum);//should be two

			// subscribe to the topics that I am supposed to based on who I am
			// such as all of the left IR filtered sensor data
			for (String topic: topics)
			{
				// I need the filtered data
				node.newSubscriber(topic + "filtered", "sensor_msgs/Range", this);
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
	public void onNewMessage(Range range) {
		int key = range.header.stamp.secs;
		//log.debug(key);

		if (key != curKey)
		{
			mtrCmd.angular_velocity = (float) 0.0;
			mtrCmd.linear_velocity = (float) 0.0;
			curNum = 0;
			numAngInput = 1;
			numLinInput = 1;
			curKey = key;

		}
		log.debug(range.header.frame_id);
		// so now I can do the math
		// the normal of the filtered range * linear_weight
		log.info(range.range / range.max_range + " range " + range.range + " max range" + range.max_range);
		double normalizedSensor = (range.range / range.max_range);
		
		if (range.header.frame_id.contains("frontCenter"))
		{
			
			mtrCmd.linear_velocity += normalizedSensor;
			
			if (normalizedSensor < 0.95)
			{
				mtrCmd.angular_velocity -= normalizedSensor;
				numAngInput = 2;
			}
			numLinInput = 2;
			// else we say that it canceled since nothing is there
		}
		else if (range.header.frame_id.contains("Left"))
		{// if left I subtract
			log.debug("LEfT ............................");
			mtrCmd.linear_velocity += normalizedSensor * linearWeight.get(range.header.frame_id);
			mtrCmd.angular_velocity -= (normalizedSensor * angularWeight.get(range.header.frame_id));
			//numAngInput++;
		}
		else// right or center
		{
			mtrCmd.linear_velocity += normalizedSensor * linearWeight.get(range.header.frame_id);
			mtrCmd.angular_velocity += (normalizedSensor * angularWeight.get(range.header.frame_id));
			//numAngInput++;
		}
		// Once I receive all the messages I can then process them
		curNum++;// finished another
		//numLinInput++;
		if (curNum == numberInputs)
		{
			// normalize the values
			mtrCmd.angular_velocity /= numAngInput ;
			mtrCmd.linear_velocity /= numLinInput;
			log.debug("Ang: " + mtrCmd.angular_velocity + " Lin: " + mtrCmd.linear_velocity);
			mtrCmd.header.stamp.secs = key;
			curNum = 0;
			pubCmd.publish(mtrCmd);
		}
	}



}
