package se.oru.coordination.coordination_oru;

import org.metacsp.multi.spatioTemporal.paths.Trajectory;
import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelope;
import se.oru.coordination.coordination_oru.code.VehiclesHashMap;
import se.oru.coordination.coordination_oru.simulation2D.TrajectoryEnvelopeCoordinatorSimulation;
import se.oru.coordination.coordination_oru.simulation2D.TrajectoryEnvelopeTrackerRK4;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Comparator;
import java.util.List;
import java.util.function.Function;

/**
 * A critical section is defined as a quadruple (te1, te2, [start1,end1], [start2,end2]), where
 * te1 and te2 are {@link TrajectoryEnvelope}s that overlap when robot 1 is in pose
 * with index start1 and robot 2 is in pose with index start2, until the two robots reach poses
 * end1 and end2 respectively.
 *
 * @author fpa
 *
 */
public class CriticalSection {
	public static boolean isCanPassFirstActiveHum = true;
	public static boolean isCanPassFirstActiveAut = false;

	private TrajectoryEnvelope te1;
	private TrajectoryEnvelope te2;
	private int te1Start = -1;
	private int te2Start = -1;
	private int te1End = -1;
	private int te2End = -1;
	private int te1Break = -1;
	private int te2Break = -1;
	public enum Weight { WEIGHT_NORMAL, WEIGHT_UNUSED, WEIGHT_RACING, WEIGHT_FORCING }
	public Weight te1HigherWeight = Weight.WEIGHT_NORMAL;
	public Weight te2HigherWeight = Weight.WEIGHT_NORMAL;

	public static boolean isCanPassFirstActiveForRobot(int robotID) {
		return VehiclesHashMap.isHuman(robotID) ? isCanPassFirstActiveHum : isCanPassFirstActiveAut;
	}

	public CriticalSection(TrajectoryEnvelope te1, TrajectoryEnvelope te2, int te1Start, int te2Start, int te1End, int te2End) {
		this.te1 = te1;
		this.te2 = te2;
		this.te1Start = te1Start;
		this.te2Start = te2Start;
		this.te1End = te1End;
		this.te2End = te2End;
	}

	@Override
	public boolean equals(Object obj) {
		if (this == obj)
			return true;
		if (obj == null)
			return false;
		if (getClass() != obj.getClass())
			return false;
		CriticalSection other = (CriticalSection) obj;
		if (te1 == null || te2 == null) {
			if (other.te1 != null && other.te2 != null)
				return false;
			else if (te1 == null)
				return te2.equals((other.te1 == null) ? other.te2 : other.te1);
		} else {
			if (!(te1.equals(other.te1) && te2.equals(other.te2) || te1.equals(other.te2) && te2.equals(other.te1)))
				return false;
			if (te1.equals(other.te1) && te2.equals(other.te2)) {
				if (te1End != other.te1End || te2End != other.te2End || te1Start != other.te1Start || te2Start != other.te2Start)
					return false;
			}
			else if (te1.equals(other.te2) && te2.equals(other.te1)) {
				if (te1End != other.te2End || te2End != other.te1End || te1Start != other.te2Start || te2Start != other.te1Start)
					return false;
			}
		}
		return true;
	}

	@Override
	public int hashCode() {
		final int prime = 31;
		int result = 1;
		result = prime * result + ((te1 == null) ? 0 : te1.hashCode()) + ((te2 == null) ? 0 : te2.hashCode());
		result = prime * result + te1End + te2End;
		result = prime * result + Math.abs(te1End - te2End);
		result = prime * result + te1Start + te2Start;
		result = prime * result + Math.abs(te1Start - te2Start);
		return result;
	}

	public TrajectoryEnvelope getTe1() {
		return te1;
	}

	public TrajectoryEnvelope getTe2() {
		return te2;
	}

	public int getTe1Start() {
		return te1Start;
	}

	public int getTe2Start() {
		return te2Start;
	}

	public int getTe1End() {
		return te1End;
	}

	public int getTe2End() {
		return te2End;
	}

	public int getTe1Break() {
		return te1Break;
	}

	public int getTe2Break() {
		return te2Break;
	}

	public void setTe1Break(int te1Break) {
		this.te1Break = te1Break;
	}

	public void setTe2Break(int te2Break) {
		this.te2Break = te2Break;
	}

	public Integer getTe1RobotID() {
		if (getTe1() == null) {
			return null;
		}
		return getTe1().getRobotID();
	}

	public Integer getTe2RobotID() {
		if (getTe2() == null) {
			return null;
		}
		return getTe2().getRobotID();
	}

	public int[] getRobotIDs() {
		if (getTe1() == null) {
			assert getTe2() == null;
			return new int[0];
		}
		return new int[] {getTe1().getRobotID(), getTe2().getRobotID()};
	}

	public boolean isTe1(int robotID) {
		return getTe1RobotID() == robotID;
	}

	public boolean isTe2(int robotID) {
		return getTe2RobotID() == robotID;
	}

	public int getOtherRobotID(int robotID) {
		return isTe1(robotID) ? getTe2RobotID() : getTe1RobotID();
	}

	public Integer getStart(int robotID) {
		if (isTe1(robotID)) {
			return getTe1Start();
		}
		if (isTe2(robotID)) {
			return getTe2Start();
		}
		return null;
	}

	public Integer getEnd(int robotID) {
		if (isTe1(robotID)) {
			return getTe1End();
		}
		if (isTe2(robotID)) {
			return getTe2End();
		}
		return null;
	}

	public boolean isRobotOnCS(int robotID) {
		int index = TrajectoryEnvelopeCoordinatorSimulation.tec.getRobotReport(robotID).getPathIndex();
		return getStart(robotID) <= index && index <= getEnd(robotID);
	}

	public void setHigher(int robotID, Weight weight) {
		if (isTe1(robotID)) {
			te1HigherWeight = weight;
		} else if (isTe2(robotID)) {
			te2HigherWeight = weight;
		} else {
			throw new RuntimeException();
		}
	}

	@Override
	public String toString() {
		// Note: Passing `true` may lead to infinite recursion.
		return toString(false);
	}

	class DistanceEstimation {
		int currentIndex;
		int goalIndex;
		int deltaIndexes;
		double distance;
		double velocity = 0.0;
		double time;

		/**
		 * This is an estimation of distance, velocity, time for the current CS and a given robot.
		 * For the inferior robot, the distance estimation is from the current point until the end of the CS (`getEnd`).
		 * For the superior robot, the distance estimation is from the current point until the start of the CS (`getStart`).
		 * The resulting time estimation is used in {@link CriticalSection#canPassFirst(int)}.
		 *
		 * @param isInferior Whether the robot should normally give way to the other one within the CS.
		 */
		public DistanceEstimation(int robotID, boolean isInferior) {
			TrajectoryEnvelopeCoordinator tec = TrajectoryEnvelopeCoordinatorSimulation.tec;
			RobotReport rr = tec.getRobotReport(robotID);

			currentIndex = rr.getPathIndex();
			goalIndex = currentIndex == -1 ? -1 : (isInferior ? getEnd(robotID) + 1 : getStart(robotID));
			deltaIndexes = Math.max(0, goalIndex - currentIndex);

			Trajectory traj = tec.trackers.get(robotID).traj;
			distance =
					deltaIndexes == 0
							? 0.0
							: goalIndex == traj.getPoseSteering().length
							? Double.POSITIVE_INFINITY
							: TrajectoryEnvelopeTrackerRK4.computeDistance(traj, currentIndex, goalIndex);
			if (distance == 0.0) {
				time = 0.0;
			} else {
				velocity = estimateVelocity(robotID, isInferior);
				time = distance / velocity; // m / m/s = s
			}
		}

		/**
		 * This is an estimation of velocity (useful before slipping through a CS).
		 * The more the speed reserve (`delta`), the greater the estimation is.
		 * So this is averageish between `currentVelocity` and `maxVelocity`.
		 *
		 * The primary reason for this calculation is to avoid zero estimation when the current velocity
		 * is temporarily zero.
		 */
		protected double estimateVelocity(int robotID, boolean isInferior) {
			TrajectoryEnvelopeCoordinator tec = TrajectoryEnvelopeCoordinatorSimulation.tec;
			double maxVelocity = tec.getRobotMaxVelocity(robotID);

			if (! isInferior) {
				return maxVelocity;
			}

			RobotReport rr = tec.getRobotReport(robotID);
			double currentVelocity = rr.getVelocity();

			/*
			double futureVelocity = Forcing.isRobotFrozen(robotID) ? 0 : maxVelocity;
			double delta = Math.abs(futureVelocity - currentVelocity);

			// This formula works for the following case:
			// - priority: human first
			// - we are an automated vehicle
			// - we stop before a crossroad
			// - the human stops before the crossroad abruptly
			// - so we should consider to cross the road
 //			return Math.min(currentVelocity, futureVelocity) + 0.4 * delta;
 			// v=2, max=12 -> 2 + 0.4 * 10 = 6
			 */

			return currentVelocity;
		}

		@Override
		public String toString() {
			return String.format(
					"%d pt = %.1f m ≈ %.1f s at %.1f m/s",
					deltaIndexes, distance, time, velocity
			);
		}
	}

	public String toString(boolean isExtra) {
		String ret = "";
		String robot1 = getTe1() == null ? "null" : String.valueOf(getTe1().getRobotID());
		String robot2 = getTe2() == null ? "null" : String.valueOf(getTe2().getRobotID());
		ret += robot1 + makeLabel(te1HigherWeight) + " [" + getTe1Start() + ";" + getTe1End() + "], ";
		ret += robot2 + makeLabel(te2HigherWeight) + " [" + getTe2Start() + ";" + getTe2End() + "]";

		if (isExtra) {
			if (! isCanPassFirstActiveForRobot(getInferior())) {
				ret += String.format(
						" (inferior %d)",
						getInferior()
				);
			} else {
				DistanceEstimation estimationInferior = new DistanceEstimation(getInferior(), true);
				DistanceEstimation estimationSuperior = new DistanceEstimation(getSuperior(), false);
				ret += String.format(
						" (inferior %d %s make %s before superior %d makes %s)",
						getInferior(), canPassFirst(getInferior()) ? "will" : "will not", estimationInferior,
						getSuperior(), estimationSuperior
				);
			}
		}
		return ret;
	}

	public Boolean canPassFirst(int myID) {
		if (! (isCanPassFirstActiveHum || isCanPassFirstActiveAut)) {
			return null;
		}

		boolean isInferior = ! isSuperior(myID);
		assert isInferior;
		int otherID = getOtherRobotID(myID);

		DistanceEstimation myEstimation = new DistanceEstimation(myID, true);
		DistanceEstimation otherEstimation = new DistanceEstimation(otherID, false);

		return myEstimation.time < otherEstimation.time;
	}

	public boolean is1Before2() {
		TrajectoryEnvelopeCoordinator tec = TrajectoryEnvelopeCoordinatorSimulation.tec;

		for (int id : List.of(getTe1RobotID(), getTe2RobotID())) {
			Dependency dep = tec.getCurrentDependencies().get(id);
			if (dep != null) {
				if (dep.getDrivingRobotID() == getTe1RobotID() && dep.getWaitingRobotID() == getTe2RobotID()) {
					if (dep.getReleasingPoint() == getTe1End()) { // an extra check of whether the dependency relates to this CS
						return true;
					}
				}
				if (dep.getDrivingRobotID() == getTe2RobotID() && dep.getWaitingRobotID() == getTe1RobotID()) {
					if (dep.getReleasingPoint() == getTe2End()) {
						return false;
					}
				}
			}
		}

		return tec.getOrder(
				tec.getRobotReport(getTe1RobotID()),
				tec.getRobotReport(getTe2RobotID()),
				this
		);
	}

	public boolean isSuperior(int robotID) {
		assert robotID == getTe1RobotID() || robotID == getTe2RobotID();
		return is1Before2() ? robotID == getTe1RobotID() : robotID == getTe2RobotID();
	}

	public int getInferior() {
		return is1Before2() ? getTe2RobotID() : getTe1RobotID();
	}

	public int getSuperior() {
		return is1Before2() ? getTe1RobotID() : getTe2RobotID();
	}

	private String makeLabel(Weight weight) {
		switch (weight) {
			case WEIGHT_NORMAL: return "";
			case WEIGHT_RACING: return "(r)";
			case WEIGHT_FORCING: return "(f)";
			default: return "(?)";
		}
	}

	public boolean isWithForcing() {
		return te1HigherWeight == Weight.WEIGHT_FORCING || te2HigherWeight == Weight.WEIGHT_FORCING;
	}

	public static ArrayList<CriticalSection> sortCriticalSections(Collection<CriticalSection> criticalSectionsUnsorted) {
		ArrayList<CriticalSection> criticalSections = new ArrayList<>(criticalSectionsUnsorted);
		criticalSections.sort(new Comparator<CriticalSection>() {
			@Override
			public int compare(CriticalSection cs1, CriticalSection cs2) {
				int[] list1 = csToInts(cs1);
				int[] list2 = csToInts(cs2);
				for (int i = 0; i < list1.length; i++) {
					int value = Integer.compare(list1[i], list2[i]);
					if (value != 0)
						return value;
				}
				return 0;
			}

			private int[] csToInts(CriticalSection cs) {
				return new int[] {
						(cs.getTe1() == null ? -1 : cs.getTe1().getRobotID()),
						cs.getTe1Start(),
						cs.getTe1End(),
						(cs.getTe2() == null ? -1 : cs.getTe2().getRobotID()),
						cs.getTe2Start(),
						cs.getTe2End(),
				};
			}
		});
		return criticalSections;
	}

	public static ArrayList<CriticalSection> sortCriticalSectionsForRobotID(Collection<CriticalSection> criticalSectionsUnsorted, int robotID) {
		ArrayList<CriticalSection> criticalSections = new ArrayList<>(criticalSectionsUnsorted);

		Function<CriticalSection, Integer[]> csToInts = (CriticalSection cs) -> {
			if (cs.getTe1RobotID() == robotID) {
				return new Integer[] { cs.getTe1Start(), cs.getTe1End(), 1 };
			}
			if (cs.getTe2RobotID() == robotID) {
				return new Integer[] { cs.getTe2Start(), cs.getTe2End(), 2 };
			}
			return new Integer[] { Integer.MAX_VALUE, Integer.MAX_VALUE, Integer.MAX_VALUE };
		};

		criticalSections.sort(new Comparator<CriticalSection>() {
			@Override
			public int compare(CriticalSection cs1, CriticalSection cs2) {
				Integer[] list1 = csToInts.apply(cs1);
				Integer[] list2 = csToInts.apply(cs2);
				for (int i = 0; i < list1.length; i++) {
					int value = Integer.compare(list1[i], list2[i]);
					if (value != 0)
						return value;
				}
				return 0;
			}

		});
		while (! criticalSections.isEmpty() &&
				csToInts.apply(criticalSections.get(criticalSections.size() - 1))[0] == Integer.MAX_VALUE) {
			criticalSections.remove(criticalSections.size() - 1);
		}
		return criticalSections;
	}
}
