package main.java.org.ros.pubsub;

import java.util.ArrayList;

public class MessageCollection <MessageType> {
	
	private int curKey;
	private int numExpecting;
	private int curIndex;
	private ArrayList<MessageType> messages;
	
	public MessageCollection(int numExpecting)
	{
		this.numExpecting = numExpecting;
		curKey = 0;
		messages = new ArrayList<MessageType>(numExpecting);
		for (int i = 0; i < numExpecting; i++)
		{
			messages.add(null);
		}
		curIndex = 0;
	}
	
	public ArrayList<MessageType> recieveMessage(MessageType newMessage, int key)
	{
		// First check if we are too slow and we are now on a different key
		// or just on a new key because we have finished the previous key
		if (key != curKey)
		{
			curIndex = 0;
			curKey = key;
		}
		// add the message to the list
		messages.set(curIndex, newMessage);
		// move to the next index
		curIndex++;
		// check if we have enough messages
		if (curIndex == numExpecting)
		{
			// if we do set the index to zero
			curIndex = 0;
			return messages;
		}
		
		// else we do not have enough messages to move on so return null
		return null;
	}

}
