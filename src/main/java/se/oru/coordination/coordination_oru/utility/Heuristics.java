package se.oru.coordination.coordination_oru.utility;

import java.util.Comparator;
import java.util.HashMap;

/**
 * This class provides various heuristics for determining the order in which robots move through critical sections.
 *
 * @author anm
 */
public class Heuristics {
    /**
     * Mapping of robot IDs to their precedence values.
     */
    public HashMap<Integer, Integer> robotIDToPrecedence = new HashMap<>();

    /**
     * Returns a comparator for determining the order based on the robot closest to a critical section.
     * @return The comparator for the closest heuristic.
     */
    public Comparator<RobotAtCriticalSection> closest() {
        return (o1, o2) -> {
            CriticalSection cs = o1.getCriticalSection();
            RobotReport robotReport1 = o1.getRobotReport();
            RobotReport robotReport2 = o2.getRobotReport();
            return ((cs.getTe1Start() - robotReport1.getPathIndex()) - (cs.getTe2Start() - robotReport2.getPathIndex()));
        };
    }

    /**
     * Returns a comparator for determining the order based on the robot with more distance traveled.
     * @return The comparator for mostDistanceToTravel heuristic.
     */
    public Comparator<RobotAtCriticalSection> mostDistanceToTravel() {
        return (o1, o2) -> (int) Math.signum(o1.getRobotReport().getDistanceTraveled() - o2.getRobotReport().getDistanceTraveled());
    }

    /**
     * Returns a comparator for determining the order based on the robot with the lowest ID number.
     * @return The comparator for lowestIDNumber heuristic.
     */
    public Comparator<RobotAtCriticalSection> lowestIDNumber() {
        return (o1, o2) -> o1.getRobotReport().getRobotID() - o2.getRobotReport().getRobotID();
    }

    /**
     * Returns a comparator for determining the order based on the robot with the highest ID number.
     * @return The comparator for highestIDNumber heuristic.
     */
    public Comparator<RobotAtCriticalSection> highestIDNumber() {
        return (o1, o2) -> o2.getRobotReport().getRobotID() - o1.getRobotReport().getRobotID();
    }

    /**
     * Returns a comparator for determining the order based on the robot with the highest precedence value.
     * @return The comparator for highestPrecedence heuristic.
     */
    public Comparator<RobotAtCriticalSection> highestPrecedence() {
        return (o1, o2) -> robotIDToPrecedence.get(o2.getRobotReport().getRobotID()) - robotIDToPrecedence.get(o1.getRobotReport().getRobotID());
    }
}
