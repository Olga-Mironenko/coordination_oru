package se.oru.coordination.coordination_oru;

import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.utility.logging.MetaCSPLogging;
import se.oru.coordination.coordination_oru.util.gates.GatedCalendar;

/**
 * A {@link RobotReport} is issued by an {@link AbstractTrajectoryEnvelopeTracker} when requested via
 * its getRobotReport() method. It contains information pertaining to the current {@link Pose} of the
 * robot, current path index, velocity and distance traveled along its current mission.
 * 
 * @author fpa
 *
 */
public class RobotReport {

	private int robotID = -1;

	private Pose pose = null;
	private int pathIndex = -1;
	private int criticalPoint = -1; // the nearest one

	public String statusString = null;
	private double velocity = 0.0;

	private double elapsedTrackingTime = 0; // seconds
	private double distanceTraveled = 0.0; // meters in the current mission (path)

	private long timeInMillis;

	/**
	 * Create a {@link RobotReport} with given current state of the robot.
	 * @param pose The current pose of the robot.
	 * @param pathIndex The index of the last pose passed by the robot. 
	 * @param velocity The current speed of the robot.
	 * @param distanceTraveled The distance traveled so far along the current current path.
	 * @param criticalPoint The current active critical point of the robot (-1 if no critical point).
	 */
	public RobotReport(int robotID, Pose pose, int pathIndex, double velocity, double distanceTraveled, int criticalPoint) {
		this.robotID = robotID;
		this.pose = pose;
		this.velocity = velocity;
		this.pathIndex = pathIndex;
		this.distanceTraveled = distanceTraveled;
		this.criticalPoint = criticalPoint;
		this.timeInMillis = GatedCalendar.getInstance().getTimeInMillis();
	}

	public RobotReport(int robotID, Pose pose, int pathIndex, double velocity, double distanceTraveled, double elapsedTrackingTime, int criticalPoint) {
		this(robotID, pose, pathIndex, velocity, distanceTraveled, criticalPoint);
		this.elapsedTrackingTime = elapsedTrackingTime;
	}

	/**
	 * Get the ID of the robot to which this report refers to.
	 * @return The ID of the robot.
	 */
	public int getRobotID() {
		return this.robotID;
	}

	/**
	 * Get the current pose of the robot.
	 * @return The current pose of the robot.
	 */
	public Pose getPose() {
		return pose;
	}

	public double getX() {
		if (pose == null) {
			return Double.NaN;
		}
		return pose.getX();
	}

	public double getY() {
		if (pose == null) {
			return Double.NaN;
		}
		return pose.getY();
	}

	/**
	 * Get the index of the last pose passed by the robot. 
	 * @return The index of the last pose passed by the robot.
	 */
	public int getPathIndex() {
		return pathIndex;
	}

	/**
	 * Get the current speed of the robot.
	 * @return The current speed of the robot.
	 */
	public double getVelocity() {
		return velocity;
	}

	/**
	 * Get the distance traveled so far along the current path.
	 * @return The distance traveled so far along the current path.
	 */
	public double getDistanceTraveled() {
		return distanceTraveled;
	}

	/**
	 * Get the current active critical point of the robot (-1 if no critical point).
	 * @return The current active critical point of the robot (-1 if no critical point).
	 */
	public int getCriticalPoint() {
		return criticalPoint;
	}

	@Override
	public String toString() {
		return "Distance: " + MetaCSPLogging.printDouble(this.distanceTraveled,4) +  "  Pose: " + this.pose + "  Index: " + this.pathIndex + "  Velocity: " + MetaCSPLogging.printDouble(velocity,4);
	}

	public double getElapsedTrackingTime() {
		return elapsedTrackingTime;
	}
}
