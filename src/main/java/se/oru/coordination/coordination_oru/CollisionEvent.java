package se.oru.coordination.coordination_oru;

import se.oru.coordination.coordination_oru.util.BrowserVisualization;
import se.oru.coordination.coordination_oru.util.gates.Timekeeper;

public class CollisionEvent {
	protected long time = 0;
	protected RobotReport[] reports = new RobotReport[2];
	public CriticalSection cs;
	public boolean isMajor;
	public long millis;

	/**
	 * Create a {@link CollisionEvent} with information on time and robots involved.
	 * @param time The time when the collision happens.
	 * @param reports The reports of the robots where the collision happens (the current ones, without delays).
	 */
	public CollisionEvent(long time, RobotReport report1, RobotReport report2, CriticalSection cs, boolean isMajor) {
		this.time = time;
		this.reports[0] = report1;
		this.reports[1] = report2;
		this.cs = cs;
		this.isMajor = isMajor;
		this.millis = Timekeeper.getVirtualMillisPassed();
	};
	
	/**
	 * Get the time when the collision happens.
	 * @return The time when the collision happens.
	 */
	public long getTime() {
		return this.time;
	};
	
	/**
	 * Get the reports of the robots where the collision happens (the current ones, without delays).
	 * @return The reports of the robots.
	 */
	public RobotReport[] getReports() {
		return this.reports;
	};
		
	/**
	 * Get an informative string related to the collision event.
	 * @return The critical section where the collision happens.
	 */
	public String toString() {
		return "Robots: [" + this.reports[0].getRobotID() +"," + this.reports[1].getRobotID()+"], PathIndices: [" + this.reports[0].getPathIndex() +", " + this.reports[1].getPathIndex() + "], Poses: "+ this.reports[0].getPose().toString() + ", " + this.reports[1].getPose().toString() + ".";
	};

	protected String getRobotInfo(int index) {
		RobotReport report = reports[index];
		return String.format("V%d @ %s",
				report.getRobotID(),
				report.getPose()
		);
	}

	public String toCompactString(int robotID) {
		int index;
		if (reports[0].getRobotID() == robotID) {
			assert reports[1].getRobotID() != robotID;
			index = 0;
		} else {
			assert reports[1].getRobotID() == robotID;
			index = 1;
		}

		return String.format("%s collision @ %s: %s",
				isMajor ? "major" : "minor",
				BrowserVisualization.secondsToHMS(millis / 1000),
				getRobotInfo(index)
		);
	};
}

