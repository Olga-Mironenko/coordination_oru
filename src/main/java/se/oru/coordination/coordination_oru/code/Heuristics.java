package se.oru.coordination.coordination_oru.code;

import se.oru.coordination.coordination_oru.CriticalSection;
import se.oru.coordination.coordination_oru.RobotAtCriticalSection;
import se.oru.coordination.coordination_oru.RobotReport;

import java.util.ArrayList;
import java.util.Comparator;

public class Heuristics {

    // A robot closest to a critical section moves first via critical section: "closest"
    public Comparator<RobotAtCriticalSection> closest() {
        return (o1, o2) -> {
            CriticalSection cs = o1.getCriticalSection();
            RobotReport robotReport1 = o1.getRobotReport();
            RobotReport robotReport2 = o2.getRobotReport();
            return ((cs.getTe1Start() - robotReport1.getPathIndex()) - (cs.getTe2Start() - robotReport2.getPathIndex()));
        };
    }

    // A robot with more distance travelled moves first via critical section: "mostDistanceToTravel"
    public Comparator<RobotAtCriticalSection> mostDistanceToTravel() {
        return (o1, o2) -> (int) Math.signum(o1.getRobotReport().getDistanceTraveled() - o2.getRobotReport().getDistanceTraveled());
    }

    // A robot with the lowest ID number moves first via critical section: "lowestIDNumber"
    public Comparator<RobotAtCriticalSection> lowestIDNumber() {
        return (o1, o2) -> o1.getRobotReport().getRobotID() - o2.getRobotReport().getRobotID();
    }

    // A robot with given priority on ID number moves first via critical section: "priorityToID"
//    public Comparator<RobotAtCriticalSection> priorityToID (ArrayList<Vehicle> vehicles) {
//        return Comparator.comparingInt(o -> vehicles.get(o.getRobotReport().getRobotID()).getPriorityID());
//    }
}
