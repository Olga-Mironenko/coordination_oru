package se.oru.coordination.coordination_oru.simulation2D;

import java.util.*;

import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.PoseSteering;
import org.metacsp.multi.spatioTemporal.paths.Trajectory;
import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelope;

import se.oru.coordination.coordination_oru.*;
import se.oru.coordination.coordination_oru.code.AbstractVehicle;
import se.oru.coordination.coordination_oru.code.VehiclesHashMap;
import se.oru.coordination.coordination_oru.util.Forcing;
import se.oru.coordination.coordination_oru.util.HumanControl;
import se.oru.coordination.coordination_oru.util.Missions;
import se.oru.coordination.coordination_oru.util.gates.GatedCalendar;
import se.oru.coordination.coordination_oru.util.gates.GatedThread;

public abstract class AdaptiveTrajectoryEnvelopeTrackerRK4 extends AbstractTrajectoryEnvelopeTracker implements Runnable {
	public static boolean isEnabledGlobally = false;

	public static double coefDeltaTimeForSlowDown = 0.1;

	public static double coefAccelerationToDeceleration = 1.7;
	/**
	 * https://www.researchgate.net/publication/233954314_Study_of_Deceleration_Behaviour_of_Different_Vehicle_Types:
	 * - mean acceleration of trucks: ~0.3 s
	 * - mean deceleration of trucks: ~0.51 s
	 */

	protected static final long WAIT_AMOUNT_AT_END = 0;
	protected static final double EPSILON = 0.01;
	protected double overallDistance = 0.0;
	protected double totalDistance = 0.0;
	protected double positionToSlowDown = Double.POSITIVE_INFINITY;
	protected double elapsedTrackingTime = 0.0;
	private Thread th = null;
	protected State state = null;
	protected double[] curvatureDampening = null;
	private ArrayList<Integer> internalCriticalPoints = new ArrayList<Integer>();
	private int numberOfReplicas = 1;
	private Random rand = new Random(1); //GatedCalendar.getInstance().getTimeInMillis());
	private TreeMap<Double,Double> slowDownProfile = null;
	private boolean slowingDown = false;
	private boolean useInternalCPs = true;
	protected ArrayList<RobotReport> reportsList = new ArrayList<RobotReport>();
	protected ArrayList<Long> reportTimeLists = new ArrayList<Long>();

	private HashMap<Integer,Integer> userCPReplacements = null;

	public static EmergencyBreaker emergencyBreaker = new EmergencyBreaker(false, false);

	public Status latestStatusForVisualization;

	public void setUseInternalCriticalPoints(boolean value) {
		this.useInternalCPs = value;
	}

	private void computeInternalCriticalPoints() {
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
		this.state = new State(0.0, 0.0);
		this.totalDistance = this.computeDistance(0, traj.getPose().length-1);
		this.overallDistance = totalDistance;
		this.computeInternalCriticalPoints();
		this.slowDownProfile = this.computeSlowdownProfile();
		this.positionToSlowDown = this.computePositionToSlowDown(totalDistance, false);
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
	protected void onTrajectoryEnvelopeUpdate() {
		synchronized(reportsList) { //FIXME not ok, all the mutex should be changed
			this.totalDistance = this.computeDistance(0, traj.getPose().length-1);
			this.overallDistance = totalDistance;
			this.internalCriticalPoints.clear();
			this.computeInternalCriticalPoints();
			this.slowDownProfile = this.computeSlowdownProfile();
			this.positionToSlowDown = this.computePositionToSlowDown(totalDistance, false);
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
			ret += traj.getPose()[i].distanceTo(traj.getPose()[i+1]);
		}
		return ret;
	}

	private double computeDistance(int startIndex, int endIndex) {
		return computeDistance(this.traj, startIndex, endIndex);
	}

	private void enqueueOneReport() {

		synchronized (reportsList) {

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
		synchronized (reportsList) {
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
		final double coef = 1.1; // slightly more than 1.0 to model speed which is greater than any actual speed

		AbstractVehicle vehicle = VehiclesHashMap.getVehicle(te.getRobotID());

		TreeMap<Double,Double> ret = new TreeMap<Double, Double>();
		State tempStateBW = new State(0.0, vehicle.getMaxVelocity()*coef);
		ret.put(tempStateBW.getVelocity(), tempStateBW.getPosition());

		double time = 0.0;
		double deltaTime = coefDeltaTimeForSlowDown * this.trackingPeriodInMillis /this.temporalResolution;
		//Compute where to slow down (can do forward here for both states...)

		while (tempStateBW.getVelocity() > 0.0) {
			double dampeningBW = getCurvatureDampening(getRobotReport(tempStateBW).getPathIndex(), true);
			//Use slightly conservative max deceleration (which is positive acceleration since we simulate FW dynamics).
			// (This is regarding dampeningBW < 1?)

			integrateRK4(tempStateBW, time, deltaTime, true, vehicle.getMaxVelocity()*coef, dampeningBW, vehicle.getMaxAcceleration(), -1);
			ret.put(tempStateBW.getVelocity(), tempStateBW.getPosition());

			time += deltaTime;
		}

		for (var entry : ret.entrySet()) {
			ret.put(entry.getKey(), tempStateBW.getPosition() - entry.getValue());
		}

		//for (Double speed : ret.keySet()) System.out.println("@speed " + speed + " --> " + ret.get(speed));
		return ret; // map each speed to the stopping distance for it
	}

	private double computePositionToSlowDown(double targetDistance, boolean isRealCriticalPoint) {
		if (state.getPosition() >= targetDistance) {
			return state.getPosition(); // essentially force slowing down
		}

		AbstractVehicle vehicle = VehiclesHashMap.getVehicle(te.getRobotID());

		State stateToBe = new State(state.getPosition(), Math.max(0, state.getVelocity()));
		double time = 0.0;
		double deltaTime = coefDeltaTimeForSlowDown * this.trackingPeriodInMillis / this.temporalResolution;
		Double prevPosition = null;

		//Compute where to slow down (can do forward here, we have the slowdown profile...)
		while (stateToBe.getPosition() < targetDistance) {
			var approximation = slowDownProfile.ceilingEntry(stateToBe.getVelocity());
			if (approximation == null) {
//				assert Math.abs(state.getVelocity()) < 0.5; // otherwise, `slowDownProfile` must contain entries for greater speeds
				return state.getPosition(); // essentially force slowing down
			}

			double speedApproximate = approximation.getKey();
			assert speedApproximate >= stateToBe.getVelocity();

			double stoppingDistanceApproximate = approximation.getValue();
			// stoppingDistanceApproximate >= actual stopping distance

			double landingPosition = stateToBe.getPosition() + stoppingDistanceApproximate;
			if (landingPosition > targetDistance) {
				if (prevPosition == null) {
					return state.getPosition();
				}
				return prevPosition; // at `prevPosition`, `landingPosition <= totalDistance`
			}

			prevPosition = stateToBe.getPosition();

			double dampeningFW = getCurvatureDampening(getRobotReport(stateToBe).getPathIndex(), true);
			integrateRK4(stateToBe, time, deltaTime, false, vehicle.getMaxVelocity(), dampeningFW, vehicle.getMaxAcceleration(), te.getRobotID());
			assert stateToBe.getPosition() > prevPosition;

			time += deltaTime;
		}

//		throw new RuntimeException("we should have returned a value in the loop");
//		assert Math.abs(state.getVelocity()) < 0.5;
		return state.getPosition(); // essentially force slowing down
	}

	public static void integrateRK4(
			State state, double time, double deltaTime, boolean slowDown,
			double MAX_VELOCITY, double MAX_VELOCITY_DAMPENING_FACTOR, double MAX_ACCELERATION,
			int robotID
	) {
		//assert MAX_VELOCITY_DAMPENING_FACTOR == 1.0; // this is just to check whether it can be different

		double MAX_DECELERATION = MAX_ACCELERATION * coefAccelerationToDeceleration;

		// Use `targetVelocity`:
		if (robotID == HumanControl.idHuman) {
			MAX_VELOCITY = Math.min(MAX_VELOCITY, HumanControl.targetVelocityHuman); // MAX_ACCELERATION or 0
			// maxAcc=2:  actual surplus 0.066
			// maxAcc=10: actual surplus 0.333
			// => 0.0333 per each maxAcc unit
			if (! slowDown && state.getVelocity() > MAX_VELOCITY + MAX_ACCELERATION * 0.034) {
				slowDown = true; // -MAX_ACCELERATION
			}
		}

		var numIntegrateCalls = TrajectoryEnvelopeCoordinatorSimulation.tec.numIntegrateCalls;
		numIntegrateCalls.put(robotID, numIntegrateCalls.getOrDefault(robotID, 0) + 1);

		synchronized(state) {
			Derivative a = Derivative.evaluate(state, time, 0.0, new Derivative(), slowDown, MAX_VELOCITY, MAX_VELOCITY_DAMPENING_FACTOR, MAX_ACCELERATION, MAX_DECELERATION);
			Derivative b = Derivative.evaluate(state, time, deltaTime/2.0, a, slowDown, MAX_VELOCITY, MAX_VELOCITY_DAMPENING_FACTOR, MAX_ACCELERATION, MAX_DECELERATION);
			Derivative c = Derivative.evaluate(state, time, deltaTime/2.0, b, slowDown, MAX_VELOCITY, MAX_VELOCITY_DAMPENING_FACTOR,MAX_ACCELERATION, MAX_DECELERATION);
			Derivative d = Derivative.evaluate(state, time, deltaTime, c, slowDown, MAX_VELOCITY, MAX_VELOCITY_DAMPENING_FACTOR, MAX_ACCELERATION, MAX_DECELERATION);

			double dxdt = (1.0f / 6.0f) * ( a.getVelocity() + 2.0f*(b.getVelocity() + c.getVelocity()) + d.getVelocity() );
			double dvdt = (1.0f / 6.0f) * (a.getAcceleration()
					+ 2.0f * (b.getAcceleration() + c.getAcceleration()) + d.getAcceleration());

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
		Thread waitToTXThread = new GatedThread("Wait to TX thread for robot " + te.getRobotID()) {

            @Override
            public void runCore() {

				int delayTx = 0;
				if (NetworkConfiguration.getMaximumTxDelay() > 0) {
					//the real delay
					int delay = (NetworkConfiguration.getMaximumTxDelay()-NetworkConfiguration.getMinimumTxDelay() > 0) ? rand.nextInt(NetworkConfiguration.getMaximumTxDelay()-NetworkConfiguration.getMinimumTxDelay()) : 0;
					delayTx = NetworkConfiguration.getMinimumTxDelay() + delay;
				}

				//Sleep for delay in communication
				try { GatedThread.sleep(delayTx); }
				catch (InterruptedException e) { e.printStackTrace(); return; }

				//if possible (according to packet loss, send
				synchronized (externalCPCounter)
				{
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
		waitToTXThread.start();

	}

	@Override
	public void setCriticalPoint(int criticalPointToSet) {
		setCriticalPoint(criticalPointToSet, true);
	}

	public void setCriticalPoint(int criticalPointToSet, boolean isRacingThroughCrossroadAllowed) {
		metaCSPLogger.finest("setCriticalPoint: (" + te.getComponent() + "): " + criticalPointToSet);
		RobotReport rr = getRobotReport();
		int robotID = rr.getRobotID();

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
			this.totalDistance = computeDistance(0, traj.getPose().length - 1);
			this.positionToSlowDown = computePositionToSlowDown(totalDistance, false);
			metaCSPLogger.finest("Set critical point (" + te.getComponent() + "): " + criticalPointToSet);
			return;
		}

		//A new intermediate index to stop at has been given
		TrajectoryEnvelopeCoordinatorSimulation tec = TrajectoryEnvelopeCoordinatorSimulation.tec;
		HashSet<CriticalSection> criticalSections = new HashSet<>();
		for (CriticalSection cs : tec.allCriticalSections) {
			Integer start = cs.getStart(robotID);
			if (start == null) {
				continue;
			}
			Integer end = cs.getEnd(robotID);
			assert end != null;

			final int margin = AbstractTrajectoryEnvelopeCoordinator.TRAILING_PATH_POINTS;

			if (start - margin <= criticalPointToSet && criticalPointToSet <= end + margin) {
				criticalSections.add(cs);
			}
		}

		//assert ! criticalSections.isEmpty();
		if (criticalSections.isEmpty()) {
			return;
		}

		if (! isRacingThroughCrossroadAllowed || criticalPointToSet > rr.getPathIndex()) {
			//TOTDIST: ---(state.getPosition)--->x--(computeDist)--->CP
			double targetDistance = computeDistance(0, criticalPointToSet);
			double positionToSlowDownTemporary = computePositionToSlowDown(targetDistance, true);

			if (! isRacingThroughCrossroadAllowed || positionToSlowDownTemporary > state.getPosition()) {
				this.setFieldCriticalPoint(criticalPointToSet);

				//assert criticalSectionsReal.size() == 1; // doesn't work when the other robot returns through the same intersection
				//this.criticalSection = criticalSectionsReal.iterator().next();
				this.criticalSections = CriticalSection.sortCriticalSections(criticalSections);
				// If there are several CSes that are close to each other
				// (contained in `criticalSectionsReal` or related to different calls of `setCriticalPoint`),
				// then they can be combined into a single artificial `this.criticalSection`.

				this.totalDistance = targetDistance;
				this.positionToSlowDown = positionToSlowDownTemporary;

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
			cs.setHigher(robotID, 2);
			// So the current robot gets higher priority.
		}
	}

	@Override
	public RobotReport getRobotReport() {
		if (state == null) return null;
		if (!this.th.isAlive()) return new RobotReport(te.getRobotID(), traj.getPose()[0], -1, 0.0, 0.0, 0.0, -1);
		synchronized(state) {
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
			rr.statusString = latestStatusForVisualization == null ? null : latestStatusForVisualization.toString();
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

	private ArrayList<CriticalSection> getCriticalSections() {
		if (criticalPoint == -1) {
			return null;
		}
		return criticalSections;
	}

	private CriticalSection getFirstOfCurrentCriticalSections() {
		if (getCriticalSections() == null) {
			return null;
		}

		TrajectoryEnvelopeCoordinator tec = TrajectoryEnvelopeCoordinatorSimulation.tec;
		for (CriticalSection cs : getCriticalSections()) {
			if (tec.allCriticalSections.contains(cs)) {
				return cs;
			}
		}
		return null;
	}

	private boolean checkFreezing() {
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
	}

	private void checkIfCanPassFirst() {
		// Forget about a CS if the robot can pass first there:
		CriticalSection criticalSection = getFirstOfCurrentCriticalSections();
		if (criticalSection == null) {
			return;
		}

		int myRobotID = te.getRobotID();

		//assert criticalSection.getInferior() == myRobotID;
		// After forcing ends, priority change affects `criticalSection` but doesn't propagate to
		// `criticalPoint` yet (at least sometimes).

		if (myRobotID == criticalSection.getSuperior() || criticalSection.canPassFirst(myRobotID)) {
			setFieldCriticalPoint(-1);
			onTrajectoryEnvelopeUpdate(); // reset `positionToSlowDown`, etc.
		}
	}

	enum Status {
		DRIVING,
		STOPPED_AT_CP,
		FULL_STOP
	}

	private Status checkIfStopped(Status status) {
		int myRobotID = te.getRobotID();

		//End condition: passed the middle AND velocity < 0 AND no criticalPoint
		//if (state.getPosition() >= totalDistance/2.0 && state.getVelocity() < 0.0) {
		boolean continueToStay = this.state.getPosition() >= this.positionToSlowDown && this.state.getVelocity() <= 0.0;

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
			//set state to final position, just in case it didn't quite get there (it's certainly close enough)
			this.state = new State(totalDistance, 0.0);
			onPositionUpdate();
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

	private void updateState(double deltaTime, double maxVelocity, double maxAcceleration) {
		slowingDown = state.getPosition() >= positionToSlowDown || checkFreezing();
		double dampening = getCurvatureDampening(getRobotReport().getPathIndex(), false);

		// Prefer to stop earlier than to cross over.
		State stateTemp = new State(state.getPosition(), state.getVelocity());
		integrateRK4(stateTemp, elapsedTrackingTime, deltaTime, slowingDown, maxVelocity, dampening, maxAcceleration, te.getRobotID());
		if (! slowingDown && stateTemp.getPosition() >= positionToSlowDown) {
			slowingDown = true;
			integrateRK4(state, elapsedTrackingTime, deltaTime, slowingDown, maxVelocity, dampening, maxAcceleration, te.getRobotID());
		} else {
			state = stateTemp;
		}
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

	@Override
	public void run() {
		this.elapsedTrackingTime = 0.0;

		double deltaTime = 0.0;
		int myRobotID = te.getRobotID();
		AbstractVehicle vehicle = VehiclesHashMap.getVehicle(myRobotID);
		int myTEID = te.getID();
		Status status = Status.DRIVING;

		while (true) {
			long timeStart = GatedCalendar.getInstance().getTimeInMillis();

			if (emergencyBreaker.isStopped(myRobotID)) {
				// TODO: just skip the current time step?
				status = Status.FULL_STOP;
				// TODO: Make a new status where we decelerate the robot and exit
				// (rather than just stop its control).
				break;
			}

			if (status == Status.DRIVING) {
				checkIfCanPassFirst();
			}

			status = checkIfStopped(status);
			if (status == Status.FULL_STOP) {
				break;
			}

			//Update the robot's state via RK4 numerical integration
			if (status == Status.DRIVING) { // TODO: skip if `deltaTime == 0.0`
				updateState(deltaTime, vehicle.getMaxVelocity(), vehicle.getMaxAcceleration());
			}

			latestStatusForVisualization = status;
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
		metaCSPLogger.info("RK4 tracking thread terminates (Robot " + myRobotID + ", TrajectoryEnvelope " + myTEID + ")");
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
