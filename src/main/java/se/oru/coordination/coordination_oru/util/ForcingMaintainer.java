package se.oru.coordination.coordination_oru.util;

import se.oru.coordination.coordination_oru.AbstractTrajectoryEnvelopeTracker;
import se.oru.coordination.coordination_oru.RobotReport;
import se.oru.coordination.coordination_oru.code.HumanDrivenVehicle;
import se.oru.coordination.coordination_oru.code.VehiclesHashMap;
import se.oru.coordination.coordination_oru.simulation2D.AdaptiveTrajectoryEnvelopeTrackerRK4;
import se.oru.coordination.coordination_oru.simulation2D.TrajectoryEnvelopeCoordinatorSimulation;

public class ForcingMaintainer {
    public KnobsAfterForcing knobsAfterForcing;
    private RobotReport rrAtForcingStart;
    private double distanceOfLastForcingStart;

    public static KnobsAfterForcing getKnobsOfTheHuman() {
        int humanID = VehiclesHashMap.getTheHuman().getID();
        AbstractTrajectoryEnvelopeTracker trackerHuman = TrajectoryEnvelopeCoordinatorSimulation.tec.trackers.get(humanID);
        if (! (trackerHuman instanceof AdaptiveTrajectoryEnvelopeTrackerRK4)) {
            return null;
        }

        ForcingMaintainer forcingMaintainer = ((AdaptiveTrajectoryEnvelopeTrackerRK4) trackerHuman).forcingMaintainer;
        if (forcingMaintainer == null) {
            return null;
        }

        return forcingMaintainer.knobsAfterForcing;
    }

    public ForcingMaintainer() {
        // Setting `Forcing.priorityDistance`, etc. should be done beforehand.
    }

    public boolean isForcingOngoing() {
        return rrAtForcingStart != null;
    }

    public void update(int robotID, Double distanceToCP, boolean isForcingNow, boolean isResumingNow, boolean isRestoringNow) {
        assert VehiclesHashMap.isHuman(robotID);
        HumanDrivenVehicle human = (HumanDrivenVehicle) VehiclesHashMap.getVehicle(robotID);

        if (isForcingNow) {
            assert ! isResumingNow;
            assert ! isRestoringNow;
            // Because that's perhaps not fully supported.

            if (knobsAfterForcing != null) {
                distanceOfLastForcingStart = human.getCurrentRobotReport().getDistanceTraveled(); // renewing the forcing
            } else {
                KnobsAfterForcing knobsAfterForcingNew = Forcing.forceDriving(robotID);
                if (knobsAfterForcingNew != null) {
                    knobsAfterForcing = knobsAfterForcingNew;
                    knobsAfterForcing.distanceToCP = distanceToCP;
                    rrAtForcingStart = human.getCurrentRobotReport();
                    distanceOfLastForcingStart = rrAtForcingStart.getDistanceTraveled();
                }
            }
        }

        assert (knobsAfterForcing != null) == (rrAtForcingStart != null);

        if (rrAtForcingStart == null) {
            return;
        }

        RobotReport rr = human.getCurrentRobotReport();
        if (rr.getPathIndex() < rrAtForcingStart.getPathIndex()) {
            // A new mission has started but forcing hasn't stopped. Let's stop it now.
            // This is a hack: we should rely not on explicit positions like
            // `ysUpwardsRestoringPriorities` (which are sometimes not reached when mission ends)
            // but rather on events like "the first crossroad has passed" and "the mission has ended".
            // A (slightly) better approach would be to compare mission IDs (that can be stored in RRs).
            knobsAfterForcing.resumeRobots();
            knobsAfterForcing.restorePriorities();
        } else {
            double distanceTraveled = rr.getDistanceTraveled() - distanceOfLastForcingStart;
            assert distanceTraveled >= 0; // otherwise, we are in a new mission, but restoring/resuming hasn't happened
            if (!knobsAfterForcing.updateForcing(distanceTraveled)) {
                knobsAfterForcing = null;
                rrAtForcingStart = null;
            }
        }
    }
}
