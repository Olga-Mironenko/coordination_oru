package se.oru.coordination.coordination_oru.code;

import java.util.Arrays;

import com.vividsolutions.jts.geom.Coordinate;

public class Vehicle {

	private final byte ID;
	private final byte priorityID;
	private String color; // FIXME Need to check in FleetVisualization
	private final float maxVelocity;
	private final float maxAcceleration;
	//private final int defaultRobotTrackingPeriod; TODO DEFAULT_ROBOT_TRACKING_PERIOD
	private final boolean isHumanDriven; // FIXME No implementation
	public Coordinate[] footPrint;

	public boolean isHumanDriven() {
		return isHumanDriven;
	}

	public Vehicle(int ID, int priorityID, String color, double maxVelocity, double maxAcceleration, boolean isHumanDriven) {
		this.ID = (byte) ID;
		this.priorityID = (byte) priorityID;
		this.color = color;
		this.maxVelocity = (float) maxVelocity;
		this.maxAcceleration = (float) maxAcceleration;
		this.isHumanDriven = isHumanDriven;
	}

	public String getColor() {
		return color;
	}
	public void setColor(String color) {
		this.color = color;
	}
	public float getMaxVelocity() {
		return maxVelocity;
	}
	public byte getID() {
		return ID;
	}
	public byte getPriorityID() {return priorityID;	}
	public float getMaxAcceleration() {
		return maxAcceleration;
	}

	public String showFootPrint() {
		float[][] coordinateArray = new float[footPrint.length][3]; 
		for (int i = 0; i < footPrint.length; i++) {
			coordinateArray[i][0] = (float) footPrint[i].x;
			coordinateArray[i][1] = (float) footPrint[i].y;
			coordinateArray[i][2] = (float) footPrint[i].z;
		}
		return Arrays.deepToString(coordinateArray);
	}

}
