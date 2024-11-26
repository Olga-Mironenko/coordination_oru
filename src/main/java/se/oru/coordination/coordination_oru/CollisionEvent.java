package se.oru.coordination.coordination_oru;

import se.oru.coordination.coordination_oru.util.BrowserVisualization;

public class CollisionEvent {
	protected long millis = 0;
	protected RobotReport[] reports = new RobotReport[2];
	public CriticalSection cs;
	public boolean isMajor;

	/**
	 * Create a {@link CollisionEvent} with information on time and robots involved.
	 * @param millis The time when the collision happens.
	 * @param report1 The reports of the robots where the collision happens (the current ones, without delays).
	 */
	public CollisionEvent(long millis, RobotReport report1, RobotReport report2, CriticalSection cs, boolean isMajor) {
		this.millis = millis;
		this.reports[0] = report1;
		this.reports[1] = report2;
		this.cs = cs;
		this.isMajor = isMajor;
	};
	
	/**
	 * Get the time when the collision happens.
	 * @return The time when the collision happens.
	 */
	public long getMillis() {
		return this.millis;
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
			index = 1;
		} else {
			assert reports[1].getRobotID() == robotID;
			index = 0;
		}

		return String.format("%s collision @ %s: %s",
				isMajor ? "major" : "minor",
				BrowserVisualization.secondsToHMS(millis / 1000),
				getRobotInfo(index)
		);
	};
}

