package se.oru.coordination.coordination_oru.tests;

import org.metacsp.multi.spatioTemporal.paths.Pose;
import se.oru.coordination.coordination_oru.CriticalSection;
import se.oru.coordination.coordination_oru.RobotReport;
import se.oru.coordination.coordination_oru.code.*;
import se.oru.coordination.coordination_oru.motionplanning.ompl.ReedsSheppCarPlanner;
import se.oru.coordination.coordination_oru.simulation2D.TrajectoryEnvelopeCoordinatorSimulation;
import se.oru.coordination.coordination_oru.tests.util.Demo;
import se.oru.coordination.coordination_oru.tests.util.GridMapConstants;
import se.oru.coordination.coordination_oru.util.*;
import se.oru.coordination.coordination_oru.util.gates.GatedThread;
import se.oru.coordination.coordination_oru.util.gates.Timekeeper;

import java.awt.*;

public class GridTestSimplified {
    enum Scenario {
        BASELINE_IDEAL_DRIVER_AUTOMATED_FIRST,
        BASELINE_IDEAL_DRIVER_HUMAN_FIRST,
        BASELINE_IDEAL_DRIVER_FIRST_COME,

        DOWN_PRIORITY_GLOBAL_CROSSROAD,
        DOWN_STOP_GLOBAL_CROSSROAD,
        /**
         * Variations of forcing:
         * - (DOWN) where forcing happens: when moving downwards
         * - which reaction to forcing:
         *   (PRIORITY) priority change
         *   (STOP) stops
         * - (GLOBAL) who is affected: all robots regardless of their positions (globally)
         * - (CROSSROAD) when everything comes back to normal: after the first crossroad
         */
    }

    public static void main(String[] args) {
        new Demo() {
            @Override
            protected void run(String scenarioString) {
                runDemo(scenarioString);
            }
        }.exec();
    }
    protected static void runDemo(String scenarioString) {
        if (scenarioString == null) {
            scenarioString = Scenario.DOWN_STOP_GLOBAL_CROSSROAD.toString();
        }
        Scenario scenario = Scenario.valueOf(scenarioString);
        AbstractVehicle.scenarioId = String.valueOf(scenario);

//        HumanControl.isEnabledForBrowser = true;
//        BrowserVisualization.isExtendedText = false;

//        Timekeeper.setVirtualSecondsPassedMax(15);
        Timekeeper.setVirtualMinutesPassedMax(60);
//        Timekeeper.realMillisPassedMax = 10 * 1000;

        final String YAML_FILE = "maps/map-grid.yaml";

        final Pose humStart = GridMapConstants.column1Row1Down;
        final Pose humFinish = GridMapConstants.column2Row1Right;

        final Pose aut1Start = GridMapConstants.row1LeftStart;
        final Pose aut1Finish = GridMapConstants.turnAround(GridMapConstants.row1RightStart);

        final Pose aut2Start = GridMapConstants.row2LeftStart;
        final Pose aut2Finish = GridMapConstants.turnAround(GridMapConstants.row2RightStart);

        final Pose aut3Start = GridMapConstants.row3LeftStart;
        final Pose aut3Finish = GridMapConstants.turnAround(GridMapConstants.row3RightStart);

        final double maxVelocityHum = 5.0;
        final double maxAccelerationHum = 0.3;
        final double maxAccelerationAut = 0.3;

        double lengthVehicle = 3.0;
        double widthVehicle = 2.0;
        VehicleSize vehicleSizeHum = new VehicleSize(3, 2, 1, 1, 0.5, 0.5);
        VehicleSize vehicleSizeAut1 = new VehicleSize(3, 2, 1, 1, 0.5, 0.5);
        VehicleSize vehicleSizeAut2 = vehicleSizeAut1;
        VehicleSize vehicleSizeAut3 = vehicleSizeAut1;

        //AutonomousVehicle.planningAlgorithm = ReedsSheppCarPlanner.PLANNING_ALGORITHM.RRTConnect; // default
        //AutonomousVehicle.planningAlgorithm = ReedsSheppCarPlanner.PLANNING_ALGORITHM.PRMstar; // too slow
        //AutonomousVehicle.planningAlgorithm = ReedsSheppCarPlanner.PLANNING_ALGORITHM.SPARS; // too slow

        // TODO: `maxAcceleration` passed here is not used by `tec`.
        AutonomousVehicle hum0 = new HumanDrivenVehicle(0, 0, Color.ORANGE, Color.ORANGE, maxVelocityHum, maxAccelerationHum);
        AutonomousVehicle aut1 = new AutonomousVehicle(1, 0, Color.BLUE, Color.BLUE, 3, maxAccelerationAut);
        AutonomousVehicle aut2 = new AutonomousVehicle(2, 0, Color.BLUE, Color.BLUE, 5, maxAccelerationAut);
        AutonomousVehicle aut3 = new AutonomousVehicle(3, 0, Color.BLUE, Color.BLUE, 7, maxAccelerationAut);

        TrajectoryEnvelopeCoordinatorSimulation tec = new TrajectoryEnvelopeCoordinatorSimulation(2000, 1000, 0, 0);
        tec.setupSolver(0, 100000000);
        tec.startInference();

        hum0.registerInTec(tec, vehicleSizeHum);
        aut1.registerInTec(tec, vehicleSizeAut1);
        aut2.registerInTec(tec, vehicleSizeAut2);
        aut3.registerInTec(tec, vehicleSizeAut3);

        tec.placeRobot(hum0.getID(), humStart);
        tec.placeRobot(aut1.getID(), aut1Start);
        tec.placeRobot(aut2.getID(), aut2Start);
        tec.placeRobot(aut3.getID(), aut3Start);

        Heuristics heuristics = new Heuristics();
        switch (scenario) {
            case BASELINE_IDEAL_DRIVER_AUTOMATED_FIRST:
            default:
                tec.addComparator(heuristics.highestIDNumber());
                break;
            case BASELINE_IDEAL_DRIVER_HUMAN_FIRST:
                tec.addComparator(heuristics.lowestIDNumber());
                break;
            case BASELINE_IDEAL_DRIVER_FIRST_COME:
                tec.addComparator(heuristics.closest());
                break;
        }

        tec.setUseInternalCriticalPoints(false);
        tec.setYieldIfParking(false);
        tec.setBreakDeadlocks(true, false, false);

        Missions.setMap(YAML_FILE); // before BrowserVisualization

        var viz = new BrowserVisualization();
        viz.setMap(YAML_FILE);
        viz.setInitialTransform(7.0, 5.0, 5.0);
        tec.setVisualization(viz);

        Missions.startMissionDispatcher(tec);

        Missions.loopMissions.put(hum0.getID(), true);
        Missions.loopMissions.put(aut1.getID(), true);
        Missions.loopMissions.put(aut2.getID(), true);
        Missions.loopMissions.put(aut3.getID(), true);

        Missions.enqueueMissions(hum0, humStart, humFinish, false);
        Missions.enqueueMissions(aut1, aut1Start, aut1Finish, false);
        Missions.enqueueMissions(aut2, aut2Start, aut2Finish, false);
        Missions.enqueueMissions(aut3, aut3Start, aut3Finish, false);

        new GatedThread("forcing thread") {
            @Override
            public void runCore() {
                switch (scenario) {
                    case BASELINE_IDEAL_DRIVER_AUTOMATED_FIRST:
                    case BASELINE_IDEAL_DRIVER_HUMAN_FIRST:
                    case BASELINE_IDEAL_DRIVER_FIRST_COME:
                        return;
                }

                assert(Forcing.priorityDistance == Double.NEGATIVE_INFINITY);
                assert(Forcing.stopDistance == Double.NEGATIVE_INFINITY);
                assert(! Forcing.isGlobalTemporaryStop);
                assert(Forcing.isResetAfterCurrentCrossroad);

                // Derive information from traits:
                assert scenario == Scenario.DOWN_PRIORITY_GLOBAL_CROSSROAD || scenario == Scenario.DOWN_STOP_GLOBAL_CROSSROAD;
                boolean isStop = scenario == Scenario.DOWN_STOP_GLOBAL_CROSSROAD;

                Forcing.priorityDistance = Double.POSITIVE_INFINITY;
                // Even on `isStop` (for `isResetAfterCurrentCrossroad` to work).

                if (isStop) {
                    Forcing.isGlobalTemporaryStop = true;
                }
                if (isStop) {
                    Forcing.stopDistance = Forcing.priorityDistance;
                }

                // Propagate the information:
                double[] ysDownwardsForcing = new double[]{humStart.getY() - 4.0};

                // Apply the information:
                KnobsAfterForcing knobsAfterForcing = null;
                RobotReport rrAtForcingStart = null;
                while (true) {
                    boolean isForcingNow = false;
                    boolean isResumingNow = false;
                    boolean isRestoringNow = false;

                    for (double y : ysDownwardsForcing) {
                        if (hum0.isYPassedDownwards(y)) {
                            isForcingNow = true;
                        }
                    }

                    if (isForcingNow) {
                        assert ! isResumingNow;
                        assert ! isRestoringNow;
                        // Because that's perhaps not fully supported.
                    }

                    if (isForcingNow) {
                        knobsAfterForcing = Forcing.forceDriving(hum0.getID());
                        if (knobsAfterForcing != null) {
                            rrAtForcingStart = hum0.getCurrentRobotReport();
                        }
                    }

                    if (rrAtForcingStart != null) {
                        if (hum0.getCurrentRobotReport().getPathIndex() < rrAtForcingStart.getPathIndex()) {
                            // A new mission has started but forcing hasn't stopped. Let's stop it now.
                            // This is a hack: we should rely not on explicit positions like
                            // `ysUpwardsRestoringPriorities` (which are sometimes not reached when mission ends)
                            // but rather on events like "the first crossroad has passed" and "the mission has ended".
                            // A (slightly) better approach would be to compare mission IDs (that can be stored in RRs).
                            knobsAfterForcing.resumeRobots();
                            knobsAfterForcing.restorePriorities();
                        } else {
                            double distanceTraveled = hum0.getCurrentRobotReport().getDistanceTraveled() - rrAtForcingStart.getDistanceTraveled();
                            assert distanceTraveled >= 0; // otherwise, we are in a new mission, but restoring/resuming hasn't happened
                            if (!knobsAfterForcing.updateForcing(distanceTraveled)) {
                                rrAtForcingStart = null;
                            }
                        }
                    }

                    GatedThread.sleepWithoutTryCatch(100);
                }
            }
        }.start();
    }
}
