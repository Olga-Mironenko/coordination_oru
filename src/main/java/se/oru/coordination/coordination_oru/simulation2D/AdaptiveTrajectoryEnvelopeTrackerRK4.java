package se.oru.coordination.coordination_oru.simulation2D;

import java.util.*;

import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.PoseSteering;
import org.metacsp.multi.spatioTemporal.paths.Trajectory;
import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelope;

import se.oru.coordination.coordination_oru.*;
import se.oru.coordination.coordination_oru.code.AbstractVehicle;
import se.oru.coordination.coordination_oru.code.HumanDrivenVehicle;
import se.oru.coordination.coordination_oru.code.VehiclesHashMap;
import se.oru.coordination.coordination_oru.util.*;
import se.oru.coordination.coordination_oru.util.gates.GatedCalendar;
import se.oru.coordination.coordination_oru.util.gates.GatedThread;
import se.oru.coordination.coordination_oru.util.gates.Timekeeper;

public abstract class AdaptiveTrajectoryEnvelopeTrackerRK4 extends AbstractTrajectoryEnvelopeTracker implements Runnable {
	public static boolean isEnabledGlobally = false;
	public static boolean isReroutingAtParkedForHuman = false;
	public static boolean isReroutingAtParkedForNonHuman = false;
	public static Integer millisReroutingAtParkedIfNotInDummyTracker = 3000;
	public static boolean isReroutingAtSlowForHuman = false;
	public static boolean isReroutingAtSlowForNonHuman = false;
	public static boolean isRacingThroughCrossroadAllowed = false;

	public static boolean isCautiousMode = false;
	public static double deltaMaxVelocityCautious = 0.0;
	public static double minMaxVelocityCautious = 0.0;

	public static double coefDeltaTimeForSlowdownProfile = 1;

	public static double coefRatioDecelerationToAcceleration = 1.7;
	/**
	 * https://www.sciencedirect.com/science/article/pii/S2352146517307937:
	 * - mean acceleration of trucks: ~0.3 m/s^2
	 * https://www.researchgate.net/publication/233954314_Study_of_Deceleration_Behaviour_of_Different_Vehicle_Types: (TODO: one more link)
	 * - mean deceleration of trucks: ~0.51 m/s^2
	 */

	protected static final long WAIT_AMOUNT_AT_END = 0;
	protected static final double EPSILON = 0.01;
	protected double overallDistance = 0.0; // to the end of the mission
	protected double totalDistance = 0.0; // to the nearest critical point
	public double positionToSlowDown = Double.POSITIVE_INFINITY;

	protected double slowdownDebugEarlyStart;
	protected double slowdownDebugEarlyFinishUnderestimation;
	protected double slowdownDebugEarlyFinishOverestimation;
	protected double slowdownDebugLateStart;
	protected double slowdownDebugLateFinishUnderestimation;
	protected double slowdownDebugLateFinishOverestimation;

	public boolean isRunCalled = false;
	protected double elapsedTrackingTime = 0.0;
	private Thread th = null;
	protected State state = null;
	protected double[] curvatureDampening = null;
	protected boolean hasCurvatureDampeningEqualValues = false;
	private ArrayList<Integer> internalCriticalPoints = new ArrayList<Integer>();
	private int numberOfReplicas = 1;
	private Random rand;
	private TreeMap<Double,Double> slowDownProfile = null;
	private boolean slowingDown = false;
	private boolean useInternalCPs = true;
	protected ArrayList<RobotReport> reportsList = new ArrayList<RobotReport>();
	protected ArrayList<Long> reportTimeLists = new ArrayList<Long>();

	private HashMap<Integer,Integer> userCPReplacements = null;
	public static EmergencyBreaker emergencyBreaker = new EmergencyBreaker(false, false);
	public Status statusLast;

	private Deque<Integer> queueStopEvents = new LinkedList<>();
	public static int millisStopEventsInitial = 10000;
	private int millisStopEvents;
	private Integer lastOtherRobotIDStopEvents = null;

	private boolean isCautious = false;
	private Double maxVelocityBeforeCautious = null;

	public double distanceToCP;
	public static double probabilityForcingForHuman = 0.0;
	public static double distanceToCPForForcing = 5.0;
	public static ForcingMaintainer forcingMaintainer;
	public static double probabilitySlowingDownForHuman = 0.0;
	public static double velocitySlowingDownForHuman = 1.0;
	public static double lengthIntervalSlowingDownForHuman = 10.0;
	public static HashMap<Integer, Double> robotIDToDurationStoppedMinimumForBlock = new HashMap<>();
	public static double durationStoppedMinimumForBlockDefault = 60.0;
	public static double deltaDurationStoppedMinimumForBlock = 60.0;

	public double durationStopped;

	public void setUseInternalCriticalPoints(boolean value) {
		this.useInternalCPs = value;
	}

	private void computeInternalCriticalPoints() {
		this.hasCurvatureDampeningEqualValues = true;
		this.curvatureDampening = new double[te.getTrajectory().getPose().length];
		this.curvatureDampening[0] = 1.0;
		Pose[] poses = this.traj.getPose();
		double prevTheta = poses[0].getTheta();
		if (poses.length > 1) prevTheta = Math.atan2(poses[1].getY() - poses[0].getY(), poses[1].getX() - poses[0].getX());
		for (int i = 0; i < poses.length-1; i++) {
			double theta = Math.atan2(poses[i+1].getY() - poses[i].getY(), poses[i+1].getX() - poses[i].getX());
			double deltaTheta = (theta-prevTheta);
			prevTheta = theta;
			if (Math.abs(deltaTheta) > Math.PI/2 && Math.abs(deltaTheta) < 1.9*Math.PI) {
				internalCriticalPoints.add(i);
				metaCSPLogger.info("Found internal critical point (" + te.getComponent() + "): " + (i));
			}
			this.curvatureDampening[i+1] = 1.0;
			if (this.curvatureDampening[i+1] != this.curvatureDampening[0]) {
				this.hasCurvatureDampeningEqualValues = false;
			}
		}
	}

	public void setCurvatureDampening(int index, double dampening) {
		this.curvatureDampening[index] = dampening;
	}

	public void setCurvatureDampening(int indexFrom, int indexTo, double dampening) {
		for (int i = indexFrom; i < indexTo; i++) curvatureDampening[i] = dampening;
	}

	public void resetCurvatureDampening() {
		for (int i  = 0; i < curvatureDampening.length; i++) curvatureDampening[i] = 1.0;
	}

	public double[] getCurvatureDampening() {
		return this.curvatureDampening;
	}

	private void computeCurvatureDampening() {
		PoseSteering[] path = this.traj.getPoseSteering();
		double deltaSinTheta = 0;
		double sinThetaPrev = Math.sin(Missions.wrapAngle180b(path[0].getTheta()));
		for (int i = 1; i < path.length; i++) {
			double sinTheta = Math.sin(Missions.wrapAngle180b(path[i].getTheta()));
			double deltaSinThetaNew = sinTheta-sinThetaPrev;
			if (deltaSinThetaNew*deltaSinTheta < 0 && i != 1) {
				System.out.println("Direction change for Robot" + this.te.getRobotID() + " in " + i);
				this.curvatureDampening[i] = 0.2;
			}
			deltaSinTheta = deltaSinThetaNew;
			sinThetaPrev = deltaSinThetaNew;
		}
	}

	public double getCurvatureDampening(int index, boolean backwards) {
		if (!backwards) return curvatureDampening[index];
		return curvatureDampening[this.traj.getPose().length-1-index];
	}

	public AdaptiveTrajectoryEnvelopeTrackerRK4(TrajectoryEnvelope te, int timeStep, double temporalResolution, TrajectoryEnvelopeCoordinator tec, TrackingCallback cb) {
		super(te, temporalResolution, tec, timeStep, cb);
		this.rand = new DeterministicRandom(AdaptiveTrajectoryEnvelopeTrackerRK4.class.getName(), te.getRobotID());
		this.state = new State(0.0, 0.0);
		this.totalDistance = traj.getPathLength();
		this.overallDistance = totalDistance;
		this.computeInternalCriticalPoints();
		this.slowDownProfile = this.computeSlowdownProfile();
		Double pos = this.computePositionToSlowDown(totalDistance, false);
		assert pos != null;
		this.positionToSlowDown = pos;
		AdaptiveTrajectoryEnvelopeTrackerRK4 self = this;
		this.th = new GatedThread("RK4 tracker " + te.getComponent()) {
			@Override
			public void runCore() {
				self.run();
			}
		};
		this.th.setPriority(Thread.MAX_PRIORITY);
	}

	@Override
	public void onTrajectoryEnvelopeUpdate() {
		/*synchronized(reportsList) { //FIXME not ok, all the mutex should be changed*/ { // for better debugging
			this.totalDistance = traj.getPathLength();
			this.overallDistance = totalDistance;
			this.internalCriticalPoints.clear();
			this.computeInternalCriticalPoints();
			this.slowDownProfile = this.computeSlowdownProfile();
			Double pos = this.computePositionToSlowDown(totalDistance, false);
			assert pos != null;
			this.positionToSlowDown = pos;
			reportsList.clear();
			reportTimeLists.clear(); //semplify to avoid discontinuities ... to be fixed.

			if (this.criticalPoint != -1) {
				int criticalPoint = this.criticalPoint;
				this.setFieldCriticalPoint(-1);
				setCriticalPoint(criticalPoint);
			}
		}
	}

	@Override
	public void startTracking() {
		assert th != null; // TODO: remove the while loop
		while (this.th == null) {
			try { GatedThread.sleep(10); }
			catch (InterruptedException e) { e.printStackTrace(); return; }
		}
		this.th.start();
		if (useInternalCPs) this.startInternalCPThread();
	}

	public static double computeDistance(Trajectory traj, int startIndex, int endIndex) {
		double ret = 0.0;
		for (int i = startIndex; i < Math.min(endIndex,traj.getPoseSteering().length-1); i++) {
			ret += traj.getPoseSteering()[i].getPose().distanceTo(traj.getPoseSteering()[i+1].getPose());
		}
		return ret;
	}

	private double computeDistance(int startIndex, int endIndex) {
		return computeDistance(this.traj, startIndex, endIndex);
	}

	private void enqueueOneReport() {

		/*synchronized (reportsList) {*/ { // for better debugging

			//Before start, initialize the position
			if (reportsList.isEmpty()) {
				if (getRobotReport() != null) {
					reportsList.add(0, getRobotReport());
					reportTimeLists.add(0, GatedCalendar.getInstance().getTimeInMillis());
				}
				return;
			}

			long timeNow = GatedCalendar.getInstance().getTimeInMillis();
			final int numberOfReplicasReceiving = this.numberOfReplicas;

			timeNow = GatedCalendar.getInstance().getTimeInMillis();
			long timeOfArrival = timeNow;
			if (NetworkConfiguration.getMaximumTxDelay() > 0) {
				//the real delay
				int delay = (NetworkConfiguration.getMaximumTxDelay()-NetworkConfiguration.getMinimumTxDelay() > 0) ? rand.nextInt(NetworkConfiguration.getMaximumTxDelay()-NetworkConfiguration.getMinimumTxDelay()) : 0;
				timeOfArrival = timeOfArrival + NetworkConfiguration.getMinimumTxDelay() + delay;
			}

			//Get the message according to packet loss probability (numberOfReplicas trials)
			boolean received = (NetworkConfiguration.PROBABILITY_OF_PACKET_LOSS > 0) ? false : true;
			int trial = 0;
			while(!received && trial < numberOfReplicasReceiving) {
				if (rand.nextDouble() < (1-NetworkConfiguration.PROBABILITY_OF_PACKET_LOSS)) //the real packet loss probability
					received = true;
				trial++;
			}
			if (received) {
				//Delete old messages that, due to the communication delay, will arrive after this one.
				ArrayList<Long> reportTimeToRemove = new ArrayList<Long>();
				ArrayList<RobotReport> reportToRemove = new ArrayList<RobotReport>();

				for (int index = 0; index < reportTimeLists.size(); index++) {
					if (reportTimeLists.get(index) < timeOfArrival) break;
					if (reportTimeLists.get(index) >= timeOfArrival) {
						reportToRemove.add(reportsList.get(index));
						reportTimeToRemove.add(reportTimeLists.get(index));
					}
				}

				for (Long time : reportTimeToRemove) reportTimeLists.remove(time);
				for (RobotReport report : reportToRemove) reportsList.remove(report);

				reportsList.add(0, getRobotReport()); //The new one is the one that will arrive later and is added in front of the queue.
				reportTimeLists.add(0, timeOfArrival); //The oldest is in the end.
			}

			//Keep alive just the most recent message before now.
			if (reportTimeLists.get(reportTimeLists.size()-1) > timeNow) {
				metaCSPLogger.severe("* ERROR * Unknown status Robot"+te.getRobotID());
				//FIXME add a function for stopping pausing the fleet and eventually restart
			}
			else {
				ArrayList<Long> reportTimeToRemove = new ArrayList<Long>();
				ArrayList<RobotReport> reportToRemove = new ArrayList<RobotReport>();

				for (int index = reportTimeLists.size()-1; index > 0; index--) {
					if (reportTimeLists.get(index) > timeNow) break; //the first in the future
					if (reportTimeLists.get(index) < timeNow && reportTimeLists.get(index-1) <= timeNow) {
						reportToRemove.add(reportsList.get(index));
						reportTimeToRemove.add(reportTimeLists.get(index));
					}
				}

				for (Long time : reportTimeToRemove) reportTimeLists.remove(time);
				for (RobotReport report : reportToRemove) reportsList.remove(report);
			}

			//Check if the current status message is too old.
			if (timeNow - reportTimeLists.get(reportTimeLists.size()-1) > tec.getControlPeriod() + TrajectoryEnvelopeCoordinator.MAX_TX_DELAY) { //the known delay
				metaCSPLogger.severe("* ERROR * Status of Robot"+ te.getRobotID() + " is too old.");
				//FIXME add a function for stopping pausing the fleet and eventually restart
				}
		}
	}

	@Override
	public RobotReport getLastRobotReport() {
		/*synchronized (reportsList) {*/ { // for better debugging
			if (reportsList.isEmpty()) return getRobotReport();
			return reportsList.get(reportsList.size()-1);
		}
	}

	private void startInternalCPThread() {
		Thread t = new GatedThread("internalCPThread") {
			@Override
			public void runCore() {
				userCPReplacements = new HashMap<Integer, Integer>();

				while (th.isAlive()) {
					ArrayList<Integer> toRemove = new ArrayList<Integer>();
					for (Integer i : internalCriticalPoints) {
						if (getRobotReport().getPathIndex() >= i) {
							toRemove.add(i);
							setCriticalPoint(userCPReplacements.get(i));
							metaCSPLogger.info("Restored critical point (" + te.getComponent() + "): " + userCPReplacements.get(i) + " which was masked by internal critical point " + i);
							break;
						}
						else {
							if (criticalPoint == -1 || criticalPoint > i) {
								userCPReplacements.put(i, criticalPoint);
								metaCSPLogger.info("Set internal critical point (" + te.getComponent() + "): " + i + " replacing critical point " + criticalPoint);
								setCriticalPoint(i);
								break;
							}
						}
					}
					for (Integer i : toRemove) {
						internalCriticalPoints.remove(i);
					}

					try { GatedThread.sleep(trackingPeriodInMillis); }
					catch (InterruptedException e) { e.printStackTrace(); return; }
				}
			}
		};
		t.start();
	}


	private TreeMap<Double,Double> computeSlowdownProfile() {
		AbstractVehicle vehicle = VehiclesHashMap.getVehicle(te.getRobotID());

		double maxVelocity = vehicle.getMaxVelocityOriginal() * 1.1;
		// (because speed is sometimes slightly higher than the maximum speed)

		TreeMap<Double,Double> ret = new TreeMap<Double, Double>();
		State tempStateBW = new State(0.0, maxVelocity);
		ret.put(tempStateBW.getVelocity(), tempStateBW.getPosition());

		double time = 0.0;
		double deltaTime = coefDeltaTimeForSlowdownProfile * this.trackingPeriodInMillis / this.temporalResolution;
		//Compute where to slow down (can do forward here for both states...)

		while (tempStateBW.getVelocity() > 0.0) {
			double dampeningBW = hasCurvatureDampeningEqualValues ? curvatureDampening[0] :
					getCurvatureDampening(getRobotReport(tempStateBW).getPathIndex(), true);
			//Use slightly conservative max deceleration (which is positive acceleration since we simulate FW dynamics).
			// (This is regarding dampeningBW < 1?)

			integrateRK4(tempStateBW, time, deltaTime, true, maxVelocity, dampeningBW, vehicle.getMaxAcceleration(), -1);
			ret.put(tempStateBW.getVelocity(), tempStateBW.getPosition());

			time += deltaTime;
		}
		assert tempStateBW.getVelocity() == 0.0;

		for (var entry : ret.entrySet()) {
			ret.put(entry.getKey(), tempStateBW.getPosition() - entry.getValue());
		}

		//for (Double speed : ret.keySet()) System.out.println("@speed " + speed + " --> " + ret.get(speed));
		return ret; // map each speed to the stopping distance for it
	}

	/** `null` means: "It's too late to slow down". */
	private Double computePositionToSlowDown(double targetDistance, boolean isRealCriticalPoint) {
		if (state.getPosition() >= targetDistance) {
			return state.getPosition(); // essentially force slowing down
		}

		AbstractVehicle vehicle = VehiclesHashMap.getVehicle(te.getRobotID());

		State stateToBe = new State(state.getPosition(), Math.max(0, state.getVelocity()));
		double time = 0.0;
		double deltaTime = this.trackingPeriodInMillis / this.temporalResolution;
		Double prevPosition = null;

		//Compute where to slow down (can do forward here, we have the slowdown profile...)
		while (stateToBe.getPosition() < targetDistance) {
			var approximationUnderestimation = slowDownProfile.floorEntry(stateToBe.getVelocity());
			var approximationOverestimation = slowDownProfile.ceilingEntry(stateToBe.getVelocity());

			assert approximationUnderestimation != null;
			assert approximationOverestimation != null;

			double velocityUnderestimation = approximationUnderestimation.getKey();
			double velocityOverestimation = approximationOverestimation.getKey();
			assert velocityUnderestimation <= stateToBe.getVelocity() && stateToBe.getVelocity() <= velocityOverestimation;

			double stoppingDistanceUnderestimation = approximationUnderestimation.getValue();
			// stoppingDistanceUnderestimation <= actual stopping distance

			double stoppingDistanceOverestimation = approximationOverestimation.getValue();
			// stoppingDistanceOverestimation >= actual stopping distance
			// (because the velocity for approximation >= `stateToBe.getVelocity()`)

			double landingPositionUnderestimation = stateToBe.getPosition() + stoppingDistanceUnderestimation;
			double landingPositionOverestimation = stateToBe.getPosition() + stoppingDistanceOverestimation;

			if (landingPositionOverestimation > targetDistance) {
				slowdownDebugLateStart = stateToBe.getPosition();
				slowdownDebugLateFinishUnderestimation = landingPositionUnderestimation;
				slowdownDebugLateFinishOverestimation = landingPositionOverestimation;

				if (prevPosition == null) {
					return null;
				}
				return prevPosition; // at `prevPosition`, `landingPosition <= totalDistance`
			}

			slowdownDebugEarlyStart = stateToBe.getPosition();
			slowdownDebugEarlyFinishUnderestimation = landingPositionUnderestimation;
			slowdownDebugEarlyFinishOverestimation = landingPositionOverestimation;

			prevPosition = stateToBe.getPosition();

			double dampeningFW = hasCurvatureDampeningEqualValues ? curvatureDampening[0] :
					getCurvatureDampening(getRobotReport(stateToBe).getPathIndex(), true); // backwards should be `false`?
			integrateRK4(stateToBe, time, deltaTime, false, vehicle.getMaxVelocity(), dampeningFW, vehicle.getMaxAcceleration(), te.getRobotID());
			assert stateToBe.getPosition() > prevPosition;

			time += deltaTime;
		}

//		throw new RuntimeException("we should have returned a value in the loop");
//		assert Math.abs(state.getVelocity()) < 0.5;
		return null; // essentially force slowing down
	}

	public static void integrateRK4(
			State state, double time, double deltaTime, boolean slowDown,
			double MAX_VELOCITY, double MAX_VELOCITY_DAMPENING_FACTOR, double MAX_ACCELERATION,
			int robotID
	) {
		//assert MAX_VELOCITY_DAMPENING_FACTOR == 1.0; // this is just to check whether it can be different

		double MAX_DECELERATION = MAX_ACCELERATION * coefRatioDecelerationToAcceleration;

		if (VehiclesHashMap.isHuman(robotID)) {
			// maxAcc=2:  actual (observed) surplus 0.066 (during tests)
			// maxAcc=10: actual (observed) surplus 0.333 (during tests)
			// => 0.0333 per each maxAcc unit (a linear function)
			if (! slowDown && state.getVelocity() > MAX_VELOCITY + MAX_ACCELERATION * 0.034) {
				slowDown = true; // -MAX_ACCELERATION
			}
		}

//		var numIntegrateCalls = TrajectoryEnvelopeCoordinatorSimulation.tec.numIntegrateCalls;
//		numIntegrateCalls.put(robotID, numIntegrateCalls.getOrDefault(robotID, 0) + 1);

		/*synchronized(state) {*/ { // for better debugging
			Derivative a = Derivative.evaluate(state, time, 0.0, new Derivative(), slowDown, MAX_VELOCITY, MAX_VELOCITY_DAMPENING_FACTOR, MAX_ACCELERATION, MAX_DECELERATION);
			Derivative b = Derivative.evaluate(state, time, deltaTime/2.0, a, slowDown, MAX_VELOCITY, MAX_VELOCITY_DAMPENING_FACTOR, MAX_ACCELERATION, MAX_DECELERATION);
			Derivative c = Derivative.evaluate(state, time, deltaTime/2.0, b, slowDown, MAX_VELOCITY, MAX_VELOCITY_DAMPENING_FACTOR,MAX_ACCELERATION, MAX_DECELERATION);
			Derivative d = Derivative.evaluate(state, time, deltaTime, c, slowDown, MAX_VELOCITY, MAX_VELOCITY_DAMPENING_FACTOR, MAX_ACCELERATION, MAX_DECELERATION);

			double dxdt = (1.0 / 6.0) * ( a.getVelocity() + 2.0*(b.getVelocity() + c.getVelocity()) + d.getVelocity() );
			double dvdt = (1.0 / 6.0) * (a.getAcceleration()
					+ 2.0 * (b.getAcceleration() + c.getAcceleration()) + d.getAcceleration());

			double velocityNew = state.getVelocity() + dvdt * deltaTime;
			velocityNew = Math.max(0.0, Math.min(MAX_VELOCITY, velocityNew));
			state.setVelocity(velocityNew);

			if (velocityNew != 0.0) {
				double deltaPosition = Math.max(0, dxdt) * deltaTime;
				double positionNew = state.getPosition() + deltaPosition;
				state.setPosition(positionNew);
			}
		}
	}

	@Override
	public void setCriticalPoint(int criticalPointToSet, int extCPCounter) {

		final int criticalPoint = criticalPointToSet;
		final int externalCPCount = extCPCounter;
		final int numberOfReplicas = this.numberOfReplicas;

		//Define a thread that will send the information
		GatedThread waitToTXThread = new GatedThread("Wait to TX thread for robot " + te.getRobotID()) {

            @Override
            public void runCore() {

				int delayTx = 0;
				if (NetworkConfiguration.getMaximumTxDelay() > 0) {
					//the real delay
					int delay = (NetworkConfiguration.getMaximumTxDelay()-NetworkConfiguration.getMinimumTxDelay() > 0) ? rand.nextInt(NetworkConfiguration.getMaximumTxDelay()-NetworkConfiguration.getMinimumTxDelay()) : 0;
					delayTx = NetworkConfiguration.getMinimumTxDelay() + delay;
				}

				if (! GatedThread.isEnabled()) {
					//Sleep for delay in communication
					try { GatedThread.sleep(delayTx); }
					catch (InterruptedException e) { e.printStackTrace(); return; }
				}

				//if possible (according to packet loss, send
				/*synchronized (externalCPCounter)*/ { // for better debugging
					boolean send = false;
					int trial = 0;
					//while(!send && trial < numberOfReplicas) {
					while(trial < numberOfReplicas) {
						if (rand.nextDouble() < (1-NetworkConfiguration.PROBABILITY_OF_PACKET_LOSS)) //the real one
							send = true;
						else {
							TrajectoryEnvelopeCoordinatorSimulation tc = (TrajectoryEnvelopeCoordinatorSimulation)tec;
							tc.incrementLostPacketsCounter();
						}
						trial++;
					}
					if (send) {
						//metaCSPLogger.info("PACKET to Robot" + te.getRobotID() + " SENT, criticalPoint: " + criticalPoint + ", externalCPCounter: " + externalCPCount);
						if (
								(externalCPCount < externalCPCounter && externalCPCount-externalCPCounter > Integer.MAX_VALUE/2.0) ||
								(externalCPCounter > externalCPCount && externalCPCounter-externalCPCount < Integer.MAX_VALUE/2.0)) {
							metaCSPLogger.info("Ignored critical point " + criticalPoint + " related to counter " + externalCPCount + " because counter is already at " + externalCPCounter + ".");
						}
						else {
							setCriticalPoint(criticalPoint);
							externalCPCounter = externalCPCount;
						}

						if (!canStartTracking()) {
							setCanStartTracking();
						}
					}
					else {
						TrajectoryEnvelopeCoordinatorSimulation tc = (TrajectoryEnvelopeCoordinatorSimulation)tec;
						tc.incrementLostMsgsCounter();
						metaCSPLogger.info("PACKET to Robot" + te.getRobotID() + " LOST, criticalPoint: " + criticalPoint + ", externalCPCounter: " + externalCPCount);
					}
				}
			}
		};
		//let's start the thread
		waitToTXThread.runCore();

	}

	public HashSet<CriticalSection> getCriticalSectionsForRobot(Integer criticalPointToConsider) {
		int robotID = te.getRobotID();

		HashSet<CriticalSection> criticalSections = new HashSet<>();
		for (CriticalSection cs : tec.allCriticalSections) {
			Integer start = cs.getStart(robotID);
			if (start == null) {
				continue;
			}
			Integer end = cs.getEnd(robotID);
			assert end != null;

			final int margin = AbstractTrajectoryEnvelopeCoordinator.TRAILING_PATH_POINTS;

			// TODO: Rather than trying to guess the critical section of a saved critical point,
			//       just save critical sections.
			if (
					criticalPointToConsider == null ||
							start - margin <= criticalPointToConsider && criticalPointToConsider <= end + margin
			) {
				criticalSections.add(cs);
			}
		}
		return criticalSections;
	}

	private boolean areThereTooManyStopEventsRecently(double otherRobotMaxVelocity) {
		int millisMin = Timekeeper.getVirtualMillisPassed() - millisStopEvents;
		while (! queueStopEvents.isEmpty()) {
			Integer first = queueStopEvents.peek();
			if (first < millisMin) {
				queueStopEvents.remove();
			} else {
				break;
			}
		}

		assert queueStopEvents.isEmpty() || queueStopEvents.peek() >= millisMin;

		double criticalPointsPerSecond = otherRobotMaxVelocity / Missions.getMapResolution();
		double criticalPointsPerMillisStopEvents = criticalPointsPerSecond * millisStopEvents / 1000;
		return queueStopEvents.size() > criticalPointsPerMillisStopEvents / 2;
	}

	private void rerouteAtSlowIfNeeded(int criticalPointToSet, Set<CriticalSection> criticalSections) {
		if (this.criticalPoint == criticalPointToSet || criticalPointToSet == -1) {
			return;
		}
		if (! slowingDown) {
			return;
		}
		// Otherwise, this is a stop event.

		int robotID = te.getRobotID();
		AbstractVehicle vehicle = VehiclesHashMap.getVehicle(robotID);
		if (vehicle instanceof HumanDrivenVehicle) {
			if (! isReroutingAtSlowForHuman) {
				return;
			}
		} else {
			if (! isReroutingAtSlowForNonHuman) {
				return;
			}
		}

		if (! queueStopEvents.isEmpty() && queueStopEvents.peekLast() == Timekeeper.getVirtualMillisPassed()) {
			return;
		}

		if (criticalSections.size() != 1) {
			return;
		}

		CriticalSection cs = criticalSections.iterator().next();
		int otherID = cs.getOtherRobotID(robotID);
		AbstractVehicle otherVehicle = VehiclesHashMap.getVehicle(otherID);
		if (! (otherVehicle instanceof HumanDrivenVehicle) && otherVehicle.getMaxVelocity() >= vehicle.getMaxVelocity()) {
			return;
		}

		if (lastOtherRobotIDStopEvents == null || lastOtherRobotIDStopEvents != otherID) {
			lastOtherRobotIDStopEvents = otherID;
			millisStopEvents = millisStopEventsInitial;
		}
		queueStopEvents.add(Timekeeper.getVirtualMillisPassed());
		if (areThereTooManyStopEventsRecently(otherVehicle.getMaxVelocity())) {
			if (
					tryToReplanNearVehicle(false, null) ==
					ResultOfTryToReplanNearVehicle.SUCCEEDED
			) {
				millisStopEvents = millisStopEventsInitial;
			} else {
				millisStopEvents *= 2;
			}
			queueStopEvents.clear(); // don't try to reroute again in the nearest future
		}
	}

	private boolean areAllCriticalSectionsWithForcing(int robotID, Set<CriticalSection> criticalSections) {
		for (CriticalSection cs : criticalSections) {
			int otherID = cs.getOtherRobotID(robotID);
			if (cs.getWeight(otherID) != CriticalSection.Weight.WEIGHT_FORCING) {
				return false;
			}
		}
		return true;
	}

	public synchronized void setCriticalPoint(int criticalPointToSet) {
		metaCSPLogger.finest("setCriticalPoint: (" + te.getComponent() + "): " + criticalPointToSet);
		int robotID = te.getRobotID();

		if (criticalPointToSet == TrajectoryEnvelopeCoordinator.CP_FORCING_HACK) {
			return;
		}

		/*
		if (this.criticalPoint == criticalPointToSet) {
			//Same critical point was already set
			metaCSPLogger.warning("Critical point (" + te.getComponent() + ") " + criticalPointToSet + " was already set!");
			return;
		}
		*/

		if (criticalPointToSet == -1) {
			//The critical point has been reset, go to the end
			this.setFieldCriticalPoint(-1);
			this.totalDistance = traj.getPathLength();
			Double pos = this.computePositionToSlowDown(totalDistance, false);
			assert pos != null;
			this.positionToSlowDown = pos;
			metaCSPLogger.finest("Set critical point (" + te.getComponent() + "): " + criticalPointToSet);
			return;
		}
		//A new intermediate index to stop at has been given

		Set<CriticalSection> criticalSections = getCriticalSectionsForRobot(criticalPointToSet);

		//assert ! criticalSections.isEmpty();
		boolean isFreezingCP = criticalPointToSet == TrajectoryEnvelopeCoordinatorSimulation.CP_ASAP;
//		if (criticalSections.isEmpty() && ! isFreezingCP) {
//			return;
//		}

		rerouteAtSlowIfNeeded(criticalPointToSet, criticalSections);

		if (Forcing.isForcingToBeIgnored) {
			if (! criticalSections.isEmpty() && areAllCriticalSectionsWithForcing(robotID, criticalSections)) {
				return;
			}
		}

		RobotReport rr = getRobotReport();
		if (isFreezingCP || ! isRacingThroughCrossroadAllowed || criticalPointToSet >= rr.getPathIndex()) {
			//TOTDIST: ---(state.getPosition)--->x--(computeDist)--->CP
			double targetDistance = computeDistance(0, criticalPointToSet);
			Double positionToSlowDownTemporary = computePositionToSlowDown(targetDistance, true);

			// When we start executing this, the robot is a little bit further than what is written in `state`.
			// Therefore, its path index may be still equal to the path index derived from `state`,
			// but its position is greater than what is written in `state` (unless it's stopped).
			if (isFreezingCP || ! isRacingThroughCrossroadAllowed || (
					positionToSlowDownTemporary != null && positionToSlowDownTemporary >= state.getPosition() ||
					positionToSlowDownTemporary == null && isStopped()
			)) {
				this.setFieldCriticalPoint(criticalPointToSet);

				//assert criticalSectionsReal.size() == 1; // doesn't work when the other robot returns through the same intersection
				//this.criticalSection = criticalSectionsReal.iterator().next();
				// If there are several CSes that are close to each other
				// (contained in `criticalSectionsReal` or related to different calls of `setCriticalPoint`),
				// then they can be combined into a single artificial `this.criticalSection`.

				this.totalDistance = targetDistance;
				this.positionToSlowDown = positionToSlowDownTemporary == null ? state.getPosition() : positionToSlowDownTemporary;

				metaCSPLogger.finest("Set critical point (" + te.getComponent() + "): " + criticalPointToSet + ", currently at point " + this.getRobotReport().getPathIndex() + ", distance " + state.getPosition() + ", will slow down at distance " + this.positionToSlowDown);
				return;

				// TODO: If not `areCriticalSectionsFound`, `this.criticalPoint` becomes outdated sometimes?
				// (Unless `setCriticalPoint` is always called for `-1` before getting called for real points.)
			}
		}

		// If this code has been reached, it means that the robot should have stopped before the intersection
		// but doesn't manage to do so because of the speed.
		if (Forcing.isRobotFrozen(robotID)) {
			// ...So we will stop after the intersection (or all the intersections) related to the critical point.
			//
			// Here we find where the critical section ends. In case there are several critical sections
			// related to the critical point, we use the end of the last critical section.
			Integer positionToStop = null;
			for (CriticalSection cs : criticalSections) {
				Integer end = cs.getEnd(robotID);
				assert end != null;
				int stop = end + 1;
				if (positionToStop == null || stop > positionToStop) {
					positionToStop = stop;
				}
			}
			Forcing.robotIDToPathIndexToStop.put(robotID, positionToStop);
		}
		for (CriticalSection cs : criticalSections) {
			cs.setWeight(robotID, CriticalSection.Weight.WEIGHT_RACING);
			// So the current robot gets higher priority.
		}
	}

	@Override
	public RobotReport getRobotReport() {
		if (state == null) return null;
		if (!this.th.isAlive()) return new RobotReport(te.getRobotID(), traj.getPoseSteering()[0].getPose(), -1, 0.0, 0.0, 0.0, -1);
		/*synchronized(state) {*/ { // for better debugging
			Pose pose = null;
			int currentPathIndex = -1;
			double accumulatedDist = 0.0;
			Pose[] poses = traj.getPose();
			for (int i = 0; i < poses.length-1; i++) {
				double deltaS = poses[i].distanceTo(poses[i+1]);
				accumulatedDist += deltaS;
				if (accumulatedDist > state.getPosition()) {
					double ratio = 1.0-(accumulatedDist-state.getPosition())/deltaS;
					pose = poses[i].interpolate(poses[i+1], ratio);
					currentPathIndex = i;
					break;
				}
			}
			if (currentPathIndex == -1) {
				currentPathIndex = poses.length-1;
				pose = poses[currentPathIndex];
			}
			RobotReport rr = new RobotReport(te.getRobotID(), pose, currentPathIndex, state.getVelocity(), state.getPosition(), elapsedTrackingTime, this.criticalPoint);
			rr.statusString = statusLast == null ? null : statusLast.toString();
			return rr;
		}
	}

	private static RobotReport getRobotReport(Trajectory traj, State auxState) {
		if (auxState == null) return null;
		Pose pose = null;
		int currentPathIndex = -1;
		double accumulatedDist = 0.0;
		Pose[] poses = traj.getPose();
		for (int i = 0; i < poses.length-1; i++) {
			double deltaS = poses[i].distanceTo(poses[i+1]);
			accumulatedDist += deltaS;
			if (accumulatedDist > auxState.getPosition()) {
				double ratio = 1.0-(accumulatedDist-auxState.getPosition())/deltaS;
				pose = poses[i].interpolate(poses[i+1], ratio);
				currentPathIndex = i;
				break;
			}
		}
		if (currentPathIndex == -1) {
			currentPathIndex = poses.length-1;
			pose = poses[currentPathIndex];
		}
		return new RobotReport(-1, pose, currentPathIndex, auxState.getVelocity(), auxState.getPosition(), 0.0,-1);
	}

	public RobotReport getRobotReport(State auxState) {
		if (auxState == null) return null;
		Pose pose = null;
		int currentPathIndex = -1;
		double accumulatedDist = 0.0;
		Pose[] poses = traj.getPose();
		for (int i = 0; i < poses.length-1; i++) {
			double deltaS = poses[i].distanceTo(poses[i+1]);
			accumulatedDist += deltaS;
			if (accumulatedDist > auxState.getPosition()) {
				double ratio = 1.0-(accumulatedDist-auxState.getPosition())/deltaS;
				pose = poses[i].interpolate(poses[i+1], ratio);
				currentPathIndex = i;
				break;
			}
		}
		if (currentPathIndex == -1) {
			currentPathIndex = poses.length-1;
			pose = poses[currentPathIndex];
		}
		return new RobotReport(te.getRobotID(), pose, currentPathIndex, auxState.getVelocity(), auxState.getPosition(), 0.0, -1);
	}

	public void setNumberOfReplicas(int numberOfReplicas) {
		if (numberOfReplicas < 1) {
			metaCSPLogger.warning("Set number of replicas failed: invalid argument.");
			return;
		}
		this.numberOfReplicas = numberOfReplicas;
	}

	public void setNumberOfReplicas(int coordinationNumberOfReplicas, int coordinationPeriodInMillis) {
		this.numberOfReplicas = Math.max(1, (int)Math.ceil(coordinationNumberOfReplicas*(double)trackingPeriodInMillis/coordinationPeriodInMillis));
	}

	private ArrayList<CriticalSection> getCriticalSectionsOfInferior() {
		if (AbstractTrajectoryEnvelopeCoordinator.isHumanIgnored) {
			return CriticalSection.sortCriticalSections(
					getCriticalSectionsForRobot(null),
					te.getRobotID()
			);
		}

		if (criticalPoint == -1) {
			return null;
		}
		return CriticalSection.sortCriticalSections(getCriticalSectionsForRobot(criticalPoint), te.getRobotID());
	}

	private CriticalSection getFirstOfCriticalSectionsOfInferior() {
		ArrayList<CriticalSection> criticalSections = getCriticalSectionsOfInferior();
		if (criticalSections == null || criticalSections.isEmpty()) {
			return null;
		}
		return criticalSections.get(0);
	}

	private boolean isCautiousSituation() {
		int myRobotID = te.getRobotID();
		if (VehiclesHashMap.isHuman(myRobotID)) {
			return false;
		}

		final boolean mustBeSuperior = false;

		HashSet<CriticalSection> criticalSections = getCriticalSectionsForRobot(null); // all
		for (CriticalSection cs : criticalSections) {
			if (VehiclesHashMap.isHuman(cs.getOtherRobotID(myRobotID)) && (! mustBeSuperior || cs.isSuperior(myRobotID))) {
				return true;
			}
		}
		return false;
	}

	void maintainCautiousMode(AbstractVehicle vehicle) {
		if (! isCautious) {
			if (isCautiousSituation()) {
				isCautious = true;

				assert deltaMaxVelocityCautious <= 0;
				// if max velocity isn't to be increased, then it's OK to keep the same `slowDownProfile`

				assert maxVelocityBeforeCautious == null;
				maxVelocityBeforeCautious = vehicle.getMaxVelocity();
				double maxVelocityNew = Math.max(
						minMaxVelocityCautious,
						vehicle.getMaxVelocity() + deltaMaxVelocityCautious
				);

				new Event.CautiousStarted(vehicle.getID(), maxVelocityNew, maxVelocityBeforeCautious).write();
				vehicle.setMaxVelocity(maxVelocityNew);
			}
		} else {
			if (! isCautiousSituation()) {
				isCautious = false;

				assert maxVelocityBeforeCautious != null;
				new Event.CautiousFinished(vehicle.getID(), maxVelocityBeforeCautious, vehicle.getMaxVelocity()).write();
				vehicle.setMaxVelocity(maxVelocityBeforeCautious);
				maxVelocityBeforeCautious = null;
			}
		}
	}

	public boolean checkFreezing() {
		return false;
		/*
		int myRobotID = te.getRobotID();

		Integer pathIndexToStop = Forcing.robotIDToPathIndexToStop.get(myRobotID);
		if (pathIndexToStop != null) {
			if (getRobotReport().getPathIndex() < pathIndexToStop) {
				// The autonomous robot should stop in the future. So we ignore whether it's frozen.
				return false;
			}
			// The autonomous robot should stop now (unless it has been already unfrozen).
			Forcing.robotIDToPathIndexToStop.remove(myRobotID);
		}

		return Forcing.isRobotFrozen(myRobotID);
		 */
	}

	private void checkIfCanPassFirst() {
		if (!CriticalSection.isCanPassFirstActiveForRobot(te.getRobotID())) {
			return;
		}

		int myRobotID = te.getRobotID();

		for (CriticalSection criticalSection : tec.allCriticalSections) {
			if (myRobotID == criticalSection.getInferior()) {
				checkIfCanPassFirstCS(criticalSection);
			}
		}
	}

	private void checkIfCanPassFirstCS(CriticalSection criticalSection) {
		int myRobotID = te.getRobotID();

		Boolean can = criticalSection.canPassFirst(myRobotID);
		if (can == null) {
			return;
		}

		HashMap<Integer, HashSet<CriticalSection>> robotIDToCSes = can
				? TrajectoryEnvelopeCoordinatorSimulation.tec.robotIDToCriticalSectionsPassFirstAffected
				: TrajectoryEnvelopeCoordinatorSimulation.tec.robotIDToCriticalSectionsPassFirstUnaffected;
		if (! robotIDToCSes.containsKey(myRobotID)) {
			robotIDToCSes.put(myRobotID, new HashSet<>());
		}
		robotIDToCSes.get(myRobotID).add(criticalSection);

		if (! can) {
			return;
		}

		int otherRobotID = criticalSection.getOtherRobotID(myRobotID);
		CriticalSection.Weight myWeight = criticalSection.getWeight(myRobotID);
		CriticalSection.Weight otherWeight = criticalSection.getWeight(otherRobotID);
		new Event.PassFirst(myRobotID, otherRobotID, myWeight.toString(), otherWeight.toString()).write();

		if (myWeight.compareTo(CriticalSection.Weight.WEIGHT_CAN_PASS_FIRST) < 0) {
			criticalSection.setWeight(myRobotID, CriticalSection.Weight.WEIGHT_CAN_PASS_FIRST);
			// TODO: Should we mark other robots affected only in this case? (Otherwise, something like forcing
			//       is going on anyway.)
		}

		if (otherWeight == CriticalSection.Weight.WEIGHT_CAN_PASS_FIRST) {
			criticalSection.setWeight(otherRobotID, CriticalSection.Weight.WEIGHT_NORMAL);
		}
	}

	public enum Status {
		DRIVING,
		STOPPED_AT_CP,
		FULL_STOP
	}

	private boolean isStopped() {
		assert this.state.getVelocity() >= 0.0;
		return this.state.getVelocity() == 0.0;
	}

	private Status checkIfStopped(Status status) {
		int myRobotID = te.getRobotID();

		//End condition: passed the middle AND velocity < 0 AND no criticalPoint
		//if (state.getPosition() >= totalDistance/2.0 && state.getVelocity() < 0.0) {
		boolean continueToStay = this.state.getPosition() >= this.positionToSlowDown && isStopped();

		if (! continueToStay) {
			if (status == Status.STOPPED_AT_CP) {
				// STOPPED_AT_CP -> DRIVING if CP=-1 (so `positionToSlowDown` becomes large, so
				// `state.getPosition() >= positionToSlowDown` is false).
				metaCSPLogger.info("Resuming from critical point (" + te.getComponent() + ")");
			}

			return Status.DRIVING;
		}

		if (status == Status.STOPPED_AT_CP) {
			// waiting for another robot
			return status;
		}

		assert status == Status.DRIVING;
		assert continueToStay;
		if (criticalPoint == -1) { // The end of mission.
			if (totalDistance == 0.0) {
				assert slowdownDebugEarlyFinishOverestimation == 0.0;
				assert slowdownDebugLateFinishOverestimation == 0.0;
			} else {
				assert slowdownDebugEarlyFinishOverestimation <= totalDistance;
				assert totalDistance < slowdownDebugLateFinishOverestimation;
			}

			assert slowdownDebugEarlyFinishUnderestimation * 0.999 <= state.getPosition() && state.getPosition() <= slowdownDebugEarlyFinishOverestimation * 1.001;
			// The multiplication is only because of a limited floating-point precision.

			double underrunUnderestimation = traj.getPathLength() - slowdownDebugEarlyFinishOverestimation;
			double underrunOverestimation = traj.getPathLength() - slowdownDebugEarlyFinishUnderestimation;

			double underrunActual = totalDistance - state.getPosition();
			assert underrunActual >= 0;

			assert underrunUnderestimation * 0.999 <= underrunActual && underrunActual <= underrunOverestimation * 1.001;
			// The multiplication is only because of a limited floating-point precision.

			//set state to final position, just in case it didn't quite get there (it's certainly close enough)
			this.state = new State(totalDistance, 0.0);

			return Status.FULL_STOP;
		}

		// We have just arrived at a CP.

		int pathIndex = getRobotReport().getPathIndex();
		metaCSPLogger.info("At critical point (" + te.getComponent() + "): " + criticalPoint + " (" + pathIndex + ")");
		if (pathIndex > criticalPoint) {
			metaCSPLogger.severe("* ATTENTION! STOPPED AFTER!! *");

			emergencyBreaker.stopRobots(myRobotID, pathIndex);
			if (emergencyBreaker.isStopped(myRobotID)) {
				return Status.FULL_STOP;
			}
			// TODO: Do emergency break when `state.getPosition() >= this.positionToSlowDown` and
			// `pathIndex > criticalPoint` (regardless of `state.getVelocity() < 0.0`)?
		}

		return Status.STOPPED_AT_CP;
	}

	public double getDurationStoppedMinimumForBlock() {
		return robotIDToDurationStoppedMinimumForBlock.getOrDefault(
				te.getRobotID(), durationStoppedMinimumForBlockDefault
		);
	}

	public boolean isBlocked() {
		return durationStopped >= getDurationStoppedMinimumForBlock();
	}

	private void updateState(double deltaTime, AbstractVehicle vehicle) {
 		double maxVelocity = vehicle.getMaxVelocity();
		double maxAcceleration = vehicle.getMaxAcceleration();

		slowingDown = state.getPosition() >= positionToSlowDown || checkFreezing();
		double dampening = hasCurvatureDampeningEqualValues ? curvatureDampening[0] :
				getCurvatureDampening(getRobotReport().getPathIndex(), false);

		double positionOld = state.getPosition();

		// Prefer to stop earlier (before `positionToSlowDown`) than to cross over.
		State stateTemp = new State(state.getPosition(), state.getVelocity());
		integrateRK4(stateTemp, elapsedTrackingTime, deltaTime, slowingDown, maxVelocity, dampening, maxAcceleration, te.getRobotID());
		if (! slowingDown && stateTemp.getPosition() > positionToSlowDown) {
			slowingDown = true;
			integrateRK4(state, elapsedTrackingTime, deltaTime, slowingDown, maxVelocity, dampening, maxAcceleration, te.getRobotID());
			// "assert state.getPosition() <= positionToSlowDown" -- not always: what we need is being slowing down
			// at `positionToSlowDown`.
		} else {
			state = stateTemp;
		}

		double positionNew = state.getPosition();
		double deltaPosition = positionNew - positionOld;
		assert deltaPosition >= 0;
		vehicle.totalDistance += deltaPosition;
	}

	private Double waitForNextStep(long timeStart) {
		//Sleep for tracking period
		int delay = trackingPeriodInMillis;
		if (NetworkConfiguration.getMaximumTxDelay() > 0) delay += rand.nextInt(NetworkConfiguration.getMaximumTxDelay());
		try { GatedThread.sleep(delay); }
		catch (InterruptedException e) { e.printStackTrace(); return null; }

		//Advance time to reflect how much we have slept (~ trackingPeriod)
		long deltaTimeInMillis = GatedThread.isEnabled() ? trackingPeriodInMillis : GatedCalendar.getInstance().getTimeInMillis() - timeStart;
		return deltaTimeInMillis / this.temporalResolution;
	}

	private Pose[] findFurtherGoals() {
		int robotID = te.getRobotID();

		ArrayList<Integer> stoppingPointsOrig = tec.stoppingPoints.getOrDefault(robotID, new ArrayList<>());
		ArrayList<Integer> stoppingPoints = new ArrayList<>(stoppingPointsOrig); // Copy the original list
		Collections.sort(stoppingPoints);

		PoseSteering[] psa = te.getTrajectory().getPoseSteering();
		int lastPoint = psa.length - 1;
		if (stoppingPoints.isEmpty() || stoppingPoints.get(stoppingPoints.size() - 1) != lastPoint) {
			stoppingPoints.add(lastPoint);
		}

		assert stoppingPoints.get(stoppingPoints.size() - 1) == lastPoint;
		if (stoppingPoints.size() == 1) { // avoiding to compute `getRobotReport()`
			return new Pose[] { psa[lastPoint].getPose() };
		}

		int currentPoint = getRobotReport().getPathIndex();
		for (int i = 0; i < stoppingPoints.size(); i++) {
			int stoppingPoint = stoppingPoints.get(i);
			if (stoppingPoint > currentPoint) {
				Pose[] goals = new Pose[stoppingPoints.size() - i];
				for (int j = 0; j < goals.length; j++) {
					goals[j] = psa[stoppingPoints.get(i + j)].getPose();
				}
				return goals;
			}
		}

		assert lastPoint == currentPoint;
		return new Pose[] { psa[lastPoint].getPose() };
	}

	public enum ResultOfTryToReplanNearVehicle { NOT_TRIED, TRIED_AND_FAILED, SUCCEEDED };

	public ResultOfTryToReplanNearVehicle tryToReplanNearVehicle(
			boolean mustBeParked, Integer millisStartedTryingToRerouteAtParked
	) {
		int myRobotID = te.getRobotID();

		CriticalSection cs = getFirstOfCriticalSectionsOfInferior();
		//assert cs != null; // this may happen when the CS has just been removed
		if (cs == null) {
			return ResultOfTryToReplanNearVehicle.NOT_TRIED;
		}

		int superiorID = cs.getSuperior();
		if (myRobotID == superiorID) {
			return ResultOfTryToReplanNearVehicle.NOT_TRIED;
		}

		if (mustBeParked) {
			assert millisStartedTryingToRerouteAtParked != null;
			int millisTrying = Timekeeper.getVirtualMillisPassed() - millisStartedTryingToRerouteAtParked;
			assert millisTrying >= 0;

			boolean isOk = (
				(tec.getTracker(superiorID) instanceof TrajectoryEnvelopeTrackerDummy) ||
						millisReroutingAtParkedIfNotInDummyTracker != null &&
						millisTrying >= millisReroutingAtParkedIfNotInDummyTracker
			);
			if (! isOk) {
				return ResultOfTryToReplanNearVehicle.NOT_TRIED;
			}
		}

		Pose[] goals = findFurtherGoals();
		if (HumanControl.moveRobot(myRobotID, goals, new int[] {superiorID})) {
			TrajectoryEnvelopeCoordinatorSimulation.incrementForRobot(
					mustBeParked
							? TrajectoryEnvelopeCoordinatorSimulation.tec.robotIDToNumReroutingsAtParked
							: TrajectoryEnvelopeCoordinatorSimulation.tec.robotIDToNumReroutingsAtSlow,
					myRobotID
			);
			new Event.Rerouting(myRobotID, superiorID, mustBeParked).write();
			return ResultOfTryToReplanNearVehicle.SUCCEEDED;
		}
		return ResultOfTryToReplanNearVehicle.TRIED_AND_FAILED;
	}

	@Override
	public void run() {
		isRunCalled = true;

		elapsedTrackingTime = 0.0;
		durationStopped = 0.0;

		double deltaTime = 0.0;
		int myRobotID = te.getRobotID();
		AbstractVehicle vehicle = VehiclesHashMap.getVehicle(myRobotID);
		Status status = Status.DRIVING;

		boolean isHuman = VehiclesHashMap.isHuman(myRobotID);

		boolean isReroutingAtParkedOK = true;
		Integer millisStartedTryingToRerouteAtParked = null;
		boolean isReroutingAtParked =
				isHuman ?
						isReroutingAtParkedForHuman :
						isReroutingAtParkedForNonHuman;

		boolean isExpectingDistanceShrinking = true;
		double distanceToCPLast = Double.POSITIVE_INFINITY;
		if (isHuman && forcingMaintainer == null) {
			forcingMaintainer = new ForcingMaintainer();
		}
		DistanceMonitor distanceMonitor = new DistanceMonitor();

//		if (myRobotID == 1) {
//			TrajectoryEnvelopeCoordinatorSimulation.tec.addStoppingPoint(myRobotID, TrajectoryEnvelopeCoordinatorSimulation.CP_ASAP, 10000);
//		}

		while (true) {
			long timeStart = GatedCalendar.getInstance().getTimeInMillis();

//			if (myRobotID == 1 && timeStart == 3000) {
//				TrajectoryEnvelopeCoordinatorSimulation.tec.removeStoppingPoint(myRobotID, TrajectoryEnvelopeCoordinatorSimulation.CP_ASAP);
//			}

			distanceToCP = computeDistanceToCP();
			if (isHuman) { // forcing
				boolean isForcingNow = false;
				if (! isExpectingDistanceShrinking && distanceToCP > distanceToCPLast) {
					isExpectingDistanceShrinking = true;
				}
				if (isExpectingDistanceShrinking &&
						distanceToCP <= distanceToCPForForcing && distanceToCPForForcing < distanceToCPLast
				) {
					isExpectingDistanceShrinking = false;

					if (rand.nextDouble() < probabilityForcingForHuman) {
						isForcingNow = true;
					}
				}
				forcingMaintainer.update(
						myRobotID, distanceToCP,
						isForcingNow,
						false, false
				);
				distanceToCPLast = distanceToCP;
			}
			if (isHuman) { // slowing down
				if (distanceMonitor.update(lengthIntervalSlowingDownForHuman, getLastRobotReport().getDistanceTraveled())) {
					double maxVelocityOld = vehicle.getMaxVelocity();

					if (rand.nextDouble() < probabilitySlowingDownForHuman) {
						vehicle.setMaxVelocity(velocitySlowingDownForHuman);
					} else {
						vehicle.resetMaxVelocity();
					}

					new Event.MaxVelocitySet(vehicle.getID(), vehicle.getMaxVelocity(), maxVelocityOld).write();
				}
			}

			if (isCautiousMode) {
				maintainCautiousMode(vehicle);
			}

			if (emergencyBreaker.isStopped(myRobotID)) {
				// TODO: just skip the current time step?
				status = Status.FULL_STOP;
				// TODO: Make a new status where we decelerate the robot and exit
				// (rather than just stop its control).
				break;
			}

			if (isStopped()) {
				durationStopped += deltaTime;
			}

			status = checkIfStopped(status);
			if (status == Status.FULL_STOP) {
				break;
			}

			if (status == Status.DRIVING) { // TODO: skip if `deltaTime == 0.0`
				checkIfCanPassFirst();
				isReroutingAtParkedOK = true;
				millisStartedTryingToRerouteAtParked = null;

				//Update the robot's state via RK4 numerical integration
				updateState(deltaTime, vehicle);
				// TODO: Move `checkIfStopped` AFTER the `updateState` (because `updateState` just reflects
				//       what the robot was ordered to do during the last `deltaTime`).

			} else if (status == Status.STOPPED_AT_CP) {
				if (isReroutingAtParked && isReroutingAtParkedOK) {
					if (millisStartedTryingToRerouteAtParked == null) {
						millisStartedTryingToRerouteAtParked = Timekeeper.getVirtualMillisPassed();
					}

					if (
							tryToReplanNearVehicle(true, millisStartedTryingToRerouteAtParked) !=
							ResultOfTryToReplanNearVehicle.NOT_TRIED
					) {
						isReroutingAtParkedOK = false;
					}
				}
			}

			if (! isStopped()) {
				if (isBlocked()) {
					robotIDToDurationStoppedMinimumForBlock.put(
							te.getRobotID(),
							getDurationStoppedMinimumForBlock() + deltaDurationStoppedMinimumForBlock
					);
				}
				durationStopped = 0.0;
			}

			statusLast = status;
			//Do some user function on position update
			onPositionUpdate();
			enqueueOneReport();

			Double delta = waitForNextStep(timeStart);
			if (delta == null) {
				return;
			}
			deltaTime = delta;
			elapsedTrackingTime += deltaTime;
		}

		assert status == Status.FULL_STOP;
		onPositionUpdate();
		enqueueOneReport();
		new Event.MissionFinished(myRobotID).write();

		//continue transmitting until the coordinator will be informed of having reached the last position.
		while (tec.getRobotReport(te.getRobotID()).getPathIndex() != -1)
		{
			enqueueOneReport();
			try { GatedThread.sleep(trackingPeriodInMillis); }
			catch (InterruptedException e) { e.printStackTrace(); return; }
		}
		// By this moment, a dummy tracker (which reports index=-1) has been started.

		//persevere with last path point in case listeners didn't catch it!
		long timerStart = getCurrentTimeInMillis();
		while (getCurrentTimeInMillis()-timerStart < WAIT_AMOUNT_AT_END) {
//			System.out.println("Waiting " + te.getComponent());
			try { GatedThread.sleep(trackingPeriodInMillis); }
			catch (InterruptedException e) { e.printStackTrace(); return; }
		}
		metaCSPLogger.info("RK4 tracking thread terminates (Robot " + myRobotID + ", TrajectoryEnvelope " + te.getID() + ")");
	}

	private double computeDistanceToCP() {
		if (criticalPoint != -1) {
			return computeDistance(getRobotReport().getPathIndex(), criticalPoint);
		}

		CriticalSection cs = getFirstOfCriticalSectionsOfInferior();
		if (cs == null) {
			return Double.POSITIVE_INFINITY;
		}

		int robotID = te.getRobotID();
		int start = cs.getStart(robotID);
		return computeDistance(getRobotReport().getPathIndex(), start);
	}

	public static double[] computeDTs(Trajectory traj, double maxVel, double maxAccel, int robotID) {
		double distance = computeDistance(traj, 0, traj.getPose().length-1);
		State state = new State(0.0, 0.0);
		double time = 0.0;
		double deltaTime = 0.0001;

		ArrayList<Double> dts = new ArrayList<Double>();
		HashMap<Integer,Double> times = new HashMap<Integer, Double>();
		dts.add(0.0);
		times.put(0, 0.0);

		//First compute time to stop (can do FW here...)
		while (state.getPosition() < distance/2.0 && state.getVelocity() < maxVel) {
			integrateRK4(state, time, deltaTime, false, maxVel, 1.0, maxAccel, robotID);
			time += deltaTime;
		}
		double positionToSlowDown = distance-state.getPosition();
		//System.out.println("Position to slow down is: " + MetaCSPLogging.printDouble(positionToSlowDown,4));

		state = new State(0.0, 0.0);
		time = 0.0;
		while (true) {
			if (state.getPosition() >= distance/2.0 && state.getVelocity() < 0.0) break;
			if (state.getPosition() >= positionToSlowDown) {
				integrateRK4(state, time, deltaTime, true, maxVel, 1.0, maxAccel, robotID);
			}
			else {
				integrateRK4(state, time, deltaTime, false, maxVel, 1.0, maxAccel, robotID);
			}
			//System.out.println("Time: " + time + " " + rr);
			//System.out.println("Time: " + MetaCSPLogging.printDouble(time,4) + "\tpos: " + MetaCSPLogging.printDouble(state.getPosition(),4) + "\tvel: " + MetaCSPLogging.printDouble(state.getVelocity(),4));
			time += deltaTime;
			RobotReport rr = getRobotReport(traj, state);
			if (!times.containsKey(rr.getPathIndex())) {
				times.put(rr.getPathIndex(), time);
				dts.add(time-times.get(rr.getPathIndex()-1));
			}
		}
		if (dts.size() < traj.getPose().length) {
			times.put(traj.getPose().length-1, time);
			dts.add(time-times.get(traj.getPose().length-2));
		}

		//System.out.println("Time: " + MetaCSPLogging.printDouble(time,4) + "\tpos: " + MetaCSPLogging.printDouble(state.getPosition(),4) + "\tvel: " + MetaCSPLogging.printDouble(state.getVelocity(),4));

		double[] ret = new double[dts.size()];
		for (int i = 0; i < dts.size(); i++) ret[i] = dts.get(i);
		return ret;

	}

	@Override
	public State getState() {
		return state;
	}

	public static boolean isEnabledForTe(TrajectoryEnvelope te) {
		if (isEnabledGlobally) {
			return true;
		}
		AbstractVehicle vehicle = VehiclesHashMap.getVehicle(te.getRobotID());
		if (vehicle == null) {
			return false;
		}
		return vehicle.isAdaptiveTracker;
	}
}
