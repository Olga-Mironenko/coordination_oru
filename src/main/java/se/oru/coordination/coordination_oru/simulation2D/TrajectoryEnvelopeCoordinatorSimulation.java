package se.oru.coordination.coordination_oru.simulation2D;

import java.util.*;
import java.util.concurrent.atomic.AtomicInteger;

import org.metacsp.framework.Constraint;
import org.metacsp.framework.Variable;
import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.PoseSteering;
import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelope;

import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.Geometry;

import se.oru.coordination.coordination_oru.AbstractTrajectoryEnvelopeTracker;
import se.oru.coordination.coordination_oru.CollisionEvent;
import se.oru.coordination.coordination_oru.CriticalSection;
import se.oru.coordination.coordination_oru.Mission;
import se.oru.coordination.coordination_oru.RobotReport;
import se.oru.coordination.coordination_oru.TrackingCallback;
import se.oru.coordination.coordination_oru.TrajectoryEnvelopeCoordinator;
import se.oru.coordination.coordination_oru.TrajectoryEnvelopeTrackerDummy;
import se.oru.coordination.coordination_oru.code.AbstractVehicle;
import se.oru.coordination.coordination_oru.code.VehiclesHashMap;
import se.oru.coordination.coordination_oru.util.Event;
import se.oru.coordination.coordination_oru.util.Missions;
import se.oru.coordination.coordination_oru.util.gates.GatedCalendar;
import se.oru.coordination.coordination_oru.util.gates.GatedThread;
import se.oru.coordination.coordination_oru.util.gates.Timekeeper;

public class TrajectoryEnvelopeCoordinatorSimulation extends TrajectoryEnvelopeCoordinator {
	public static TrajectoryEnvelopeCoordinatorSimulation tec = null;
	
	protected static final long START_TIME = GatedCalendar.getInstance().getTimeInMillis();
	protected boolean useInternalCPs = true;
	
	protected boolean checkCollisions = true; // TODO
	public ArrayList<CollisionEvent> allCollisionsList = new ArrayList<CollisionEvent>();
	public ArrayList<CollisionEvent> majorCollisionsList = new ArrayList<CollisionEvent>();
	public HashMap<Integer, List<CollisionEvent>> robotIDToAllCollisions = new HashMap<>();
	public HashMap<Integer, List<CollisionEvent>> robotIDToMinorCollisions = new HashMap<>();
	public HashMap<Integer, List<CollisionEvent>> robotIDToMajorCollisions = new HashMap<>();
	protected Thread collisionThread = null;

	public HashMap<Integer, Integer> robotIDToNumReroutingsAtParked = new HashMap<>();
	public HashMap<Integer, Integer> robotIDToNumReroutingsAtSlow = new HashMap<>();

	public HashMap<Integer, HashSet<CriticalSection>> robotIDToCriticalSectionsPassFirstAffected = new HashMap<>();
	public HashMap<Integer, HashSet<CriticalSection>> robotIDToCriticalSectionsPassFirstUnaffected = new HashMap<>();

	protected AtomicInteger totalMsgsLost = new AtomicInteger(0);
	protected AtomicInteger totalPacketsLost = new AtomicInteger(0);
	
	protected double DEFAULT_MAX_VELOCITY;
	protected double DEFAULT_MAX_ACCELERATION;

	public static void incrementForRobot(HashMap<Integer, Integer> robotIDToNum, int robotID) {
		robotIDToNum.put(robotID, robotIDToNum.getOrDefault(robotID, 0) + 1);
	}
	
	/**
	 * The default footprint used for robots if none is specified.
	 * NOTE: coordinates in footprints must be given in in CCW or CW order. 
	 */
	public static Coordinate[] DEFAULT_FOOTPRINT = new Coordinate[] {
			new Coordinate(-1.7, 0.7),	//back left
			new Coordinate(-1.7, -0.7),	//back right
			new Coordinate(2.7, -0.7),	//front right
			new Coordinate(2.7, 0.7)	//front left
	};

	/**
	 * Dimension of the default footprint.
	 */
	public static double MAX_DEFAULT_FOOTPRINT_DIMENSION = 4.4;
	
	/**
	 * Enable the collision checking thread.
	 * @param enable <code>true</code>  if the thread for checking collisions should be enabled.
	 */
	public void setCheckCollisions(boolean enable) {
		this.checkCollisions = enable;
	}
		
	/** 
	 * Just for statistic purposes (simulation).
	 */
	public void incrementLostMsgsCounter() {
		this.totalMsgsLost.incrementAndGet();
	}
	
	/** 
	 * Just for statistic purposes (simulation).
	 */
	public void incrementLostPacketsCounter() {
		this.totalPacketsLost.incrementAndGet();
	}
	
	/**
	 * Create a new {@link TrajectoryEnvelopeCoordinatorSimulation} with given parameters.
	 * @param CONTROL_PERIOD The control period of the coordinator (e.g., 1000 msec)
	 * @param TEMPORAL_RESOLUTION The temporal resolution at which the control period is specified (e.g., 1000)
	 * @param MAX_VELOCITY The maximum speed of a robot (used to appropriately instantiate the {@link TrajectoryEnvelopeTrackerRK4} instances).
	 * @param MAX_ACCELERATION The maximum acceleration/deceleration of a robot (used to appropriately instantiate the {@link TrajectoryEnvelopeTrackerRK4} instances).
	 * @param DEFAULT_ROBOT_TRACKING_PERIOD The default tracking period in milliseconds (used to appropriately instantiate the {@link TrajectoryEnvelopeTrackerRK4} instances).
	 */
	public TrajectoryEnvelopeCoordinatorSimulation(int CONTROL_PERIOD, double TEMPORAL_RESOLUTION, double MAX_VELOCITY, double MAX_ACCELERATION, int DEFAULT_ROBOT_TRACKING_PERIOD) {
		super(CONTROL_PERIOD, TEMPORAL_RESOLUTION);
		this.DEFAULT_MAX_VELOCITY = MAX_VELOCITY;
		this.DEFAULT_MAX_ACCELERATION = MAX_ACCELERATION;
		this.DEFAULT_ROBOT_TRACKING_PERIOD = DEFAULT_ROBOT_TRACKING_PERIOD;
		assert tec == null;
		tec = this;
	}
	
	/**
	 * Create a new {@link TrajectoryEnvelopeCoordinatorSimulation} with the following default values:
	 * <ul>
	 * <li><code>CONTROL_PERIOD</code> = 1000</li>
	 * <li><code>TEMPORAL_RESOLUTION</code> = 1000</li>
	 * <li><code>DEFAULT_ROBOT_TRACKING_PERIOD</code> = 30</li>
	 * <li><code>PARKING_DURATION</code> = 3000</li>
	 * </ul>
	 */
	public TrajectoryEnvelopeCoordinatorSimulation(int CONTROL_PERIOD, double TEMPORAL_RESOLUTION, double MAX_VELOCITY, double MAX_ACCELERATION) {
		this(CONTROL_PERIOD, TEMPORAL_RESOLUTION, MAX_VELOCITY, MAX_ACCELERATION, Timekeeper.virtualMillisPerTimestep);
		assert GatedThread.isEnabled();
	}

	/**
	 * Create a new {@link TrajectoryEnvelopeCoordinatorSimulation} with the following default values:
	 * <ul>
	 * <li><code>CONTROL_PERIOD</code> = 1000</li>
	 * <li><code>TEMPORAL_RESOLUTION</code> = 1000</li>
	 * <li><code>DEFAULT_ROBOT_TRACKING_PERIOD</code> = 30</li>
	 * <li><code>PARKING_DURATION</code> = 3000</li>
	 * </ul>
	 */
	public TrajectoryEnvelopeCoordinatorSimulation(double MAX_VELOCITY, double MAX_ACCELERATION) {
		this(1000, 1000, MAX_VELOCITY, MAX_ACCELERATION, 30);
	}
	
	/**
	 * Create a new {@link TrajectoryEnvelopeCoordinatorSimulation} with the following default values:
	 * <ul>
	 * <li><code>CONTROL_PERIOD</code> = 1000</li>
	 * <li><code>TEMPORAL_RESOLUTION</code> = 1000</li>
	 * <li><code>MAX_VELOCITY</code> = 10.0</li>
	 * <li><code>MAX_ACCELERATION</code> = 1.0</li>
	 * <li><code>DEFAULT_ROBOT_TRACKING_PERIOD</code> = 30</li>
	 * <li><code>PARKING_DURATION</code> = 3000</li>
	 * </ul>
	 */
	public TrajectoryEnvelopeCoordinatorSimulation() {
		this(1000, 1000, 10.0, 1.0, 30);
	}

	private ArrayList<Integer> computeStoppingPoints(PoseSteering[] poses) {
		ArrayList<Integer> ret = new ArrayList<Integer>();
		double prevTheta = poses[0].getTheta();
		if (poses.length > 1) prevTheta = Math.atan2(poses[1].getY() - poses[0].getY(), poses[1].getX() - poses[0].getX());
		for (int i = 0; i < poses.length-1; i++) {
			double theta = Math.atan2(poses[i+1].getY() - poses[i].getY(), poses[i+1].getX() - poses[i].getX());
			double deltaTheta = (theta-prevTheta);
			prevTheta = theta;
			if (Math.abs(deltaTheta) > Math.PI/2 && Math.abs(deltaTheta) < 1.9*Math.PI) {
				ret.add(i);
			}
		}
		return ret;
	}

	@Override
	public boolean addMissions(Mission... missions) {
		HashMap<Mission, HashMap<Pose, Integer>> userStoppingPoints = new HashMap<Mission, HashMap<Pose,Integer>>();
		if (this.useInternalCPs) {
			for (Mission m : missions) {
				PoseSteering[] path = m.getPath();
				ArrayList<Integer> sps = computeStoppingPoints(path);
				userStoppingPoints.put(m, m.getStoppingPoints());
				for (Integer i : sps) m.setStoppingPoint(path[i-1].getPose(), 100);				
			}
		}
		if (!super.addMissions(missions)) {
			if (this.useInternalCPs) {
				for (Mission m : missions) {
					m.clearStoppingPoints();
					for (Pose p : userStoppingPoints.get(m).keySet()) {
						m.setStoppingPoint(p, userStoppingPoints.get(m).get(p));
					}
				}			
			}
			return false;
		}
		return true;
	}


	/**
	 * Enable (default) or disable the use of internal critical points in the {@link TrajectoryEnvelopeTrackerRK4} trackers.
	 * @param value <code>true</code> if these critical points should be used to slow down, <code>false</code> otherwise.
	 */
	public void setUseInternalCriticalPoints(boolean value) {
		this.useInternalCPs = value;
	}
		
	/**
	 * Get the maximum velocity of a given robot (m/s).
	 * @param robotID The ID of the robot.
	 * @return The maximum velocity of the robot (or the default value if not specified).
	 */
	@Override
	public Double getRobotMaxVelocity(int robotID) {
		if (this.robotMaxVelocity.containsKey(robotID)) 
			return this.robotMaxVelocity.get(robotID);
		if (! GatedThread.isEnabled()) {
			return DEFAULT_MAX_VELOCITY;
		}

		AbstractVehicle vehicle = VehiclesHashMap.getVehicle(robotID);
		if (vehicle == null) {
			return DEFAULT_MAX_VELOCITY;
		}
		return vehicle.getMaxVelocity();
	}
	
	/**
	 * Get the maximum acceleration of a given robot (m/s^2).
	 * @param robotID The ID of the robot.
	 * @return The maximum acceleration of the robot (or the default value if not specified).
	 */
	@Override
	public Double getRobotMaxAcceleration(int robotID) {
		if (this.robotMaxAcceleration.containsKey(robotID)) 
			return this.robotMaxAcceleration.get(robotID);
		if (! GatedThread.isEnabled()) {
			return DEFAULT_MAX_ACCELERATION;
		}

		AbstractVehicle vehicle = VehiclesHashMap.getVehicle(robotID);
		if (vehicle == null) {
			return DEFAULT_MAX_ACCELERATION;
		}
		return vehicle.getMaxAcceleration();
	}
	
	@Override
	protected Double getMaxFootprintDimension(int robotID) {
		if (this.footprints.containsKey(robotID)) return maxFootprintDimensions.get(robotID);
		return MAX_DEFAULT_FOOTPRINT_DIMENSION;
	}

	/**
	 * Get the {@link Coordinate}s defining the default footprint of robots.
	 * @return The {@link Coordinate}s defining the default footprint of robots.
	 */
	public Coordinate[] getDefaultFootprint() {
		return DEFAULT_FOOTPRINT;
	}

	/**
	 * Get the {@link Coordinate}s defining the footprint of a given robot.
	 * @param robotID the ID of the robot
	 * @return The {@link Coordinate}s defining the footprint of a given robot.
	 */
	@Override
	public Coordinate[] getFootprint(int robotID) {
		if (this.footprints.containsKey(robotID)) return this.footprints.get(robotID);
		return DEFAULT_FOOTPRINT;
	}
	
	/**
	 * Get a {@link Geometry} representing the default footprint of robots.
	 * @return A {@link Geometry} representing the default footprint of robots.
	 */
	public Geometry getDefaultFootprintPolygon() {
		Geometry fpGeom = TrajectoryEnvelope.createFootprintPolygon(DEFAULT_FOOTPRINT);
		return fpGeom;
	}
	
	/**
	 * Set the default footprint of robots, which is used for computing spatial envelopes.
	 * Provide the bounding polygon of the machine assuming its reference point is in (0,0), and its
	 * orientation is aligned with the x-axis. The coordinates must be in CW or CCW order.
	 * @param coordinates The coordinates delimiting bounding polygon of the footprint.
	 */
	public void setDefaultFootprint(Coordinate ... coordinates) {
		DEFAULT_FOOTPRINT = coordinates;
		MAX_DEFAULT_FOOTPRINT_DIMENSION = computeMaxFootprintDimension(coordinates);
	}

	@Override
	public AbstractTrajectoryEnvelopeTracker getNewTracker(TrajectoryEnvelope te, TrackingCallback cb) {
		if (AdaptiveTrajectoryEnvelopeTrackerRK4.isEnabledForTe(te)) {
			return getNewAdaptiveTracker(te, cb);
		}

		if (this.getRobotTrackingPeriodInMillis(te.getRobotID()) == null || this.getRobotMaxVelocity(te.getRobotID()) == null || this.getRobotMaxAcceleration(te.getRobotID()) == null) throw new Error("Robot" +  te.getRobotID() + ": missing kinodynamic parameters.");
		TrajectoryEnvelopeTrackerRK4 ret = new TrajectoryEnvelopeTrackerRK4(te, this.getRobotTrackingPeriodInMillis(te.getRobotID()), TEMPORAL_RESOLUTION, this, cb) {
			//Method for measuring time in the trajectory envelope tracker
			@Override
			public long getCurrentTimeInMillis() {
				return tec.getCurrentTimeInMillis();
			}
		};
		//ret.setUseInternalCriticalPoints(this.useInternalCPs);
		ret.setUseInternalCriticalPoints(false);
		return ret;
	}

	private AbstractTrajectoryEnvelopeTracker getNewAdaptiveTracker(TrajectoryEnvelope te, TrackingCallback cb) {
		if (this.getRobotTrackingPeriodInMillis(te.getRobotID()) == null || this.getRobotMaxVelocity(te.getRobotID()) == null || this.getRobotMaxAcceleration(te.getRobotID()) == null) throw new Error("Robot" +  te.getRobotID() + ": missing kinodynamic parameters.");
		AdaptiveTrajectoryEnvelopeTrackerRK4 ret = new AdaptiveTrajectoryEnvelopeTrackerRK4(te, this.getRobotTrackingPeriodInMillis(te.getRobotID()), TEMPORAL_RESOLUTION, this, cb) {
			//Method for measuring time in the trajectory envelope tracker
			@Override
			public long getCurrentTimeInMillis() {
				return tec.getCurrentTimeInMillis();
			}
		};
		//ret.setUseInternalCriticalPoints(this.useInternalCPs);
		ret.setUseInternalCriticalPoints(false);
		return ret;
	}

	//Method for measuring time in the trajectory envelope coordinator
	@Override
	public long getCurrentTimeInMillis() {
		return GatedCalendar.getInstance().getTimeInMillis()-START_TIME;
	}

	public String constraintsToGraphviz(Constraint[] constraints) {
		String ret = "digraph { ";
		for (Constraint c : constraints) {
			Variable[] vs = c.getScope();
			if (vs.length == 2) {
				ret += "\"" + vs[0] + "\" -> \"" + vs[1] + "\" [label=\"" + c.getEdgeLabel() + "\"]; ";
			}
		}
		ret += "}";
		return ret;
	}

	@Override
	protected String[] getStatistics() {
		
		String CONNECTOR_BRANCH = (char)0x251C + "" + (char)0x2500 + " ";
		String CONNECTOR_LEAF = (char)0x2514 + "" + (char)0x2500 + " ";
		ArrayList<String> ret = new ArrayList<String>();
		int numVar = solver.getConstraintNetwork().getVariables().length;
		Constraint[] constraints = solver.getConstraintNetwork().getConstraints();
		int numCon = constraints.length;
		
		/*synchronized (trackers) {*/ { // for better debugging
			ret.add("Status @ "  + getCurrentTimeInMillis() + " ms");
			ret.add(CONNECTOR_BRANCH + 		"Eff period .......... " + EFFECTIVE_CONTROL_PERIOD + " ms");
			ret.add(CONNECTOR_BRANCH + 		"Constraint network .. " + numVar + " variables, " + numCon + " constraints");
			for (int i = 0; i < numCon; i++) {
				ret.add(CONNECTOR_BRANCH + 		"constraints[" + i + "] = " + constraints[i]);
			}
			ret.add(CONNECTOR_BRANCH + constraintsToGraphviz(constraints));

			HashSet<Integer> allRobots = new HashSet<Integer>();
			for (Integer robotID : trackers.keySet()) {
				allRobots.add(robotID);
			}
			String st = CONNECTOR_BRANCH +	"Robot states ........ ";
			for (Integer robotID : allRobots) {
				AbstractTrajectoryEnvelopeTracker tracker = trackers.get(robotID);
				RobotReport rr = tracker.getRobotReport(); 
				int currentPP = rr.getPathIndex();
				st += tracker.getTrajectoryEnvelope().getComponent();
				if (tracker instanceof TrajectoryEnvelopeTrackerDummy) st += " (P)";
				else st += " (D)";
				st += ": " + currentPP + "   ";
			}
			ret.add(st);
			st = CONNECTOR_BRANCH + 		"Robots' view ........ ";
			for (Integer robotID : allRobots) {
				AbstractTrajectoryEnvelopeTracker tracker = trackers.get(robotID);
				RobotReport rr = getRobotReport(robotID); 
				if (rr != null ) {
					int currentPP = rr.getPathIndex();
					st += tracker.getTrajectoryEnvelope().getComponent();
					if (tracker instanceof TrajectoryEnvelopeTrackerDummy) st += " (P)";
					else st += " (D)";
					st += ": " + currentPP + "   ";
				}
			}
			ret.add(st);
		}
		/*synchronized (currentDependencies) {*/ { // for better debugging
			ret.add(CONNECTOR_BRANCH + 		"Dependencies ........ " + currentDependencies);
		}
		if (checkCollisions) {
			for (ArrayList<CollisionEvent> collisionsList : Arrays.asList(allCollisionsList, majorCollisionsList)) {
				int numberOfCollisions = 0;
				/*synchronized (collisionsList) {*/ { // for better debugging
					numberOfCollisions = collisionsList.size();
					String label = collisionsList == allCollisionsList ? " all " : "major";
					ret.add(CONNECTOR_BRANCH + "Collisions (" + label + ")... " + numberOfCollisions + ".");
					if (numberOfCollisions > 0) {
						for (int cindex = 0; cindex < collisionsList.size() - 1; cindex++) {
							CollisionEvent ce = collisionsList.get(cindex);
							ret.add(" " + CONNECTOR_BRANCH + " " + ce.toString());
						}
						CollisionEvent ce = collisionsList.get(collisionsList.size() - 1);
						ret.add(" " + CONNECTOR_LEAF + " " + ce.toString());
					}
				}
			}
		}
		ret.add(CONNECTOR_BRANCH + 			"Obsolete crit sect .. " + criticalSectionCounter.get() + ".");
		ret.add(CONNECTOR_BRANCH + 			"Messages sent ....... " + totalMsgsSent.get() + ", lost: " + totalMsgsLost.get() + ", retransmitted: " + totalMsgsReTx.get() + ". Packets lost: " + totalPacketsLost.get() + ", number of replicas: " + numberOfReplicas + ".");
		ret.add(CONNECTOR_BRANCH + 			"Unalive states ...... " + nonliveStatesDetected.get() + ", avoided: " + nonliveStatesAvoided.get() + ", revised according to heuristic: " + currentOrdersHeurusticallyDecided.get() + ".");
		ret.add(CONNECTOR_LEAF + 			"Re-planned paths .... " + replanningTrialsCounter.get() + ", successful: " + successfulReplanningTrialsCounter.get() + ".");
		return ret.toArray(new String[ret.size()]);
	}

	public int getCountCollisionsList() {
		return allCollisionsList.size();
	}

	protected int findCSinCollisionsList(CriticalSection cs, List<CollisionEvent> collisionsList) {
		for (int i = 0; i < collisionsList.size(); i++) {
			if (collisionsList.get(i).cs == cs) {
				return i;
			}
		}
		return -1;
	}

	@Override
	public void onCriticalSectionUpdate() {
		startCollisionThread();
		Missions.computeMissionLinearizations(false);
	}

	private void startCollisionThread() {
		if (! (checkCollisions && collisionThread == null && ! this.allCriticalSections.isEmpty())) {
			return;
		}

		// Start the collision checking thread.
		// The tread will be alive until there will be almost one critical section.
		collisionThread = new GatedThread("Collision checking thread.") {

			@Override
			public void runCore() {
				metaCSPLogger.info("Starting the collision checking thread.");

				while(true) {
					//collisions can happen only in critical sections
					/*synchronized (allCriticalSections) {*/ { // for better debugging
//							if (allCriticalSections.isEmpty())
//								break; //break the thread if there are no critical sections to control

						for (CriticalSection cs : allCriticalSections) {
							//FIXME sample the real pose of the robots (ok in simulation, but not otherwise)
							RobotReport robotReport1, robotReport2;
							AbstractTrajectoryEnvelopeTracker tracker1, tracker2;
							try {
								/*synchronized (trackers) {*/ { // for better debugging
									tracker1 = trackers.get(cs.getTe1().getRobotID());
									robotReport1 = tracker1.getRobotReport();
									tracker2 = trackers.get(cs.getTe2().getRobotID());
									robotReport2 = tracker2.getRobotReport();
								}
							} catch (NullPointerException e) {
								continue; //skip this cycle
							}

							if (robotReport1 == null || robotReport2 == null) {
								continue;
							}

							PoseSteering[] path1 = cs.getTe1().getTrajectory().getPoseSteering();
							PoseSteering[] path2 = cs.getTe2().getTrajectory().getPoseSteering();

							int i1 = robotReport1.getPathIndex();
							if (i1 == -1) {
								i1 = robotReport1.getDistanceTraveled() == 0.0 ? 0 : path1.length - 1;
								// This is because in a dummy tracker:
								// - cs.getTe1Start() == cs.getTe1End() == 0
								// - cs.getTe1().getTrajectory().getPoseSteering() has only index 0
							}

							int i2 = robotReport2.getPathIndex();
							if (i2 == -1) {
								i2 = robotReport2.getDistanceTraveled() == 0.0 ? 0 : path2.length - 1;
							}

							int s1 = cs.getTe1Start();
							int s2 = cs.getTe2Start();

							int e1 = cs.getTe1End();
							int e2 = cs.getTe2End();

							boolean isInside1 = s1 <= i1 && i1 <= e1;
							boolean isInside2 = s2 <= i2 && i2 <= e2;

							boolean isInside = isInside1 && isInside2;
							if (!isInside) {
								continue;
							}

							for (ArrayList<CollisionEvent> collisionsList : Arrays.asList(allCollisionsList, majorCollisionsList)) {
								boolean isMajor = collisionsList == majorCollisionsList;
								//check if both the robots are inside the critical section

								// TODO: This is a hack (`setInnerFootprint` should be called in a proper place).
								cs.getTe1().setInnerFootprint(tec.getInnerFootprint(cs.getTe1RobotID()));
								cs.getTe2().setInnerFootprint(tec.getInnerFootprint(cs.getTe2RobotID()));

								//place robot  in pose and get geometry
								Geometry placement1 = isMajor
									? cs.getTe1().makeInnerFootprint(path1[i1])
									: cs.getTe1().makeFootprint(path1[i1]);

								Geometry placement2 = isMajor
									? cs.getTe2().makeInnerFootprint(path2[i2])
									: cs.getTe2().makeFootprint(path2[i2]);

								//check intersection
								if (!placement1.intersects(placement2)) {
									continue;
								}

								if (findCSinCollisionsList(cs, collisionsList) != -1) {
									continue;
								}

								metaCSPLogger.info(" * NEW COLLISION *");
								CollisionEvent ce = new CollisionEvent(GatedCalendar.getInstance().getTimeInMillis(), robotReport1, robotReport2, cs, isMajor);

								collisionsList.add(ce);

								for (List<Integer> pair : List.of(
										List.of(robotReport1.getRobotID(), robotReport2.getRobotID()),
										List.of(robotReport2.getRobotID(), robotReport1.getRobotID())
								)) {
									int a = pair.get(0);
									int b = pair.get(1);
									if (isMajor) {
										new Event.MajorCollisionFromMinor(a, b).write();
									} else {
										new Event.MinorCollision(a, b).write();
									}
								}

								HashMap<Integer, List<CollisionEvent>> robotIDToCollisions = isMajor ? robotIDToMajorCollisions : robotIDToAllCollisions;
								for (int robotID : Arrays.asList(cs.getTe1RobotID(), cs.getTe2RobotID(), -1)) {
									// Prepare a list of collisions:
									if (! robotIDToCollisions.containsKey(robotID)) {
										robotIDToCollisions.put(robotID, new ArrayList<>());
										if (! isMajor) {
											robotIDToMinorCollisions.put(robotID, new ArrayList<>());
										}
									}

									// Add the collision:
									robotIDToCollisions.get(robotID).add(ce);
									if (! isMajor) {
										robotIDToMinorCollisions.get(robotID).add(ce);
									} else {
										// Remove the corresponding minor collision:
										int indexMinor = findCSinCollisionsList(cs, robotIDToMinorCollisions.get(robotID));
										assert indexMinor != -1; // TODO: perhaps doesn't work if there's no safety distance
										robotIDToMinorCollisions.get(robotID).remove(indexMinor);

										// Change the type of the corresponding minor collision:
										int indexAdd = findCSinCollisionsList(cs, robotIDToAllCollisions.get(robotID));
										assert indexAdd != -1;
										CollisionEvent ceAll = robotIDToAllCollisions.get(robotID).get(indexAdd);
										if (! ceAll.isMajor) {
											ceAll.isMajor = true;
										}
									}
								}
							}
						}
					}

					try { GatedThread.sleep((long)(1000.0/30.0)); }
					catch (InterruptedException e) { e.printStackTrace(); return; }
				}
//					metaCSPLogger.info("Ending the collision checking thread.");
			}

		};

		collisionThread.start();
	}
}
