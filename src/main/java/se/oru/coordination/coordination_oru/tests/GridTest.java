package se.oru.coordination.coordination_oru.tests;

import java.awt.Color;

import org.metacsp.multi.spatioTemporal.paths.Pose;

import se.oru.coordination.coordination_oru.CriticalSection;
import se.oru.coordination.coordination_oru.RobotReport;
import se.oru.coordination.coordination_oru.code.*;
import se.oru.coordination.coordination_oru.motionplanning.ompl.ReedsSheppCarPlanner;
import se.oru.coordination.coordination_oru.simulation2D.EmergencyBreaker;
import se.oru.coordination.coordination_oru.simulation2D.TrajectoryEnvelopeCoordinatorSimulation;
import se.oru.coordination.coordination_oru.simulation2D.AdaptiveTrajectoryEnvelopeTrackerRK4;
import se.oru.coordination.coordination_oru.tests.util.Demo;
import se.oru.coordination.coordination_oru.tests.util.GridMapConstants;
import se.oru.coordination.coordination_oru.util.*;
import se.oru.coordination.coordination_oru.util.gates.GatedThread;
import se.oru.coordination.coordination_oru.util.gates.Timekeeper;

public class GridTest {
    enum TraitA { AD, A3, AU, A_DOWN_EVERYWHERE, A_DOWN_UP_EVERYWHERE }
    enum TraitB { BP, BS }

    enum TraitC { C1, C2, C3, CG }
    enum TraitD { DC, DM }

    enum Scenario {
        BASELINE_IDEAL_DRIVER_AUTOMATED_FIRST_COL1,

        BASELINE_IDEAL_DRIVER_AUTOMATED_FIRST,
        BASELINE_AUTOMATED_ONLY,
        BASELINE_IDEAL_DRIVER_HUMAN_FIRST,
        BASELINE_IDEAL_DRIVER_FIRST_COME,

        CROSS_FIRST_COME_FORCING_DOWN_EVERYWHERE,
        CROSS_AUTOMATED_FIRST_FORCING_DOWN_EVERYWHERE,
        CROSS_FIRST_COME_FORCING_DOWN_UP_EVERYWHERE,
        CROSS_AUTOMATED_FIRST_FORCING_DOWN_UP_EVERYWHERE,

        S_DP1C, S_DP1M, S_DP2C, S_DP2M, S_DP3C, S_DP3M, S_DPGC, S_DPGM,
        S_3P1C,
        S_DS1C, S_DS1M, S_DS2C, S_DS2M, S_DS3C, S_DS3M, S_DSGC, S_DSGM,
        S_UP1C, S_UP1M, S_UP2C, S_UP2M, S_UP3C, S_UP3M, S_UPGC, S_UPGM,
        S_US1C, S_US1M, S_US2C, S_US2M, S_US3C, S_US3M, S_USGC, S_USGM,
        /**
         *
         * Crossroads:
         *
         *      v   ^
         *     1D  1U
         *     2D  2U
         *     3D  3U
         *      v   ^
         *
         * Variations of forcing:
         * - (A) where forcing happens:
         *   D) before crossroad 1D
         *   U) before crossroads 1D, 3U
         *   3) before crossroad 3D
         * - (B) which reaction to forcing:
         *   P) priority change
         *   S) stops
         * - (C) who is affected:
         *   1) the robot before the first (the nearest) crossroad
         *   2) the robots before the first and second crossroads
         *   3) the robots before all three crossroads
         *   G) all robots regardless of their positions (globally)
         * - (D) when everything comes back to normal:
         *   C) after the first crossroad
         *   M) after the end of mission
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

    protected static double computeDistanceToStop(double velocity, double acceleration) {
        // velocity = 12 m/s
        // deceleration = acceleration * coefAccelerationToDeceleration = 2 * 3 = 6 m/s^2
        double deceleration = acceleration * AdaptiveTrajectoryEnvelopeTrackerRK4.coefAccelerationToDeceleration;
        // => timeToStop = velocity / deceleration = 2 s (6 m/s^2 * 2 s = 12 m/s)
        double timeToStop = velocity / deceleration;

        // avgSpeed (while stopping) = velocity / 2 = 6 m/s
        double avgSpeed = velocity / 2;
        // => distanceToStop = timeToStop * avgSpeed = 12 m
        double distanceToStop = timeToStop * avgSpeed;

        return distanceToStop;
    }

    protected static void runDemo(String scenarioString) {
        // Settings
        boolean isCrossMode = true;

        if (scenarioString == null) {
            scenarioString = Scenario.CROSS_AUTOMATED_FIRST_FORCING_DOWN_UP_EVERYWHERE.toString();
        }
        Scenario scenario = Scenario.valueOf(scenarioString);
        AbstractVehicle.scenarioId = String.valueOf(scenario);

//        Timekeeper.setVirtualSecondsPassedMax(20 * 60 + 6);
        Timekeeper.setVirtualMinutesPassedMax(60);
//        Timekeeper.realMillisPassedMax = 1000 * 60 * 60; // 1h

        final boolean ishumLoop = true;

        final String YAML_FILE = "maps/map-grid.yaml";

        final Pose humStart = isCrossMode ? GridMapConstants.row1LeftStart : scenario == Scenario.BASELINE_IDEAL_DRIVER_AUTOMATED_FIRST_COL1 ? GridMapConstants.column1TopStart : GridMapConstants.column2TopStart;
//        final Pose humFinish = GridMapConstants.turnAround(GridMapConstants.column2BottomStart);
        final Pose humMiddle = isCrossMode ? GridMapConstants.column3Row1Down : null;
        final Pose humFinish = GridMapConstants.turnAround(isCrossMode ? GridMapConstants.row3RightStart : GridMapConstants.column2BottomStart);

        final Pose aut1Start = isCrossMode ? GridMapConstants.column2TopStart : GridMapConstants.row1LeftStart;
        final Pose aut1Finish = GridMapConstants.turnAround(isCrossMode ? GridMapConstants.column2BottomStart : GridMapConstants.row1RightStart);

        final Pose aut2Start = GridMapConstants.row2LeftStart;
        final Pose aut2Finish = GridMapConstants.turnAround(GridMapConstants.row2RightStart);

        final Pose aut3Start = isCrossMode ? null : GridMapConstants.row3LeftStart;
        final Pose aut3Finish = isCrossMode ? null : GridMapConstants.turnAround(GridMapConstants.row3RightStart);

        final Pose aut4Start = null;
        final Pose aut4Finish = null;

        final Pose aut5Start = null;
        final Pose aut5Finish = null;

        // v = maxVelocityHum = 12 m/s
        // a = -maxAccelerationHum * coefAccelerationToDeceleration = -2 * 3 = -6 m/s^2
        // => timeToStop = 2 s (-6 m/s^2 * 2 s = -12 m/s)
        // avg. speed (while stopping): vAvg = maxVelocityHum / 2 = 6 m/s
        // => distanceToStop = timeToStop * vAvg = 12 m
        final double maxVelocityHum = 5;
        final double maxAccelerationHum = 0.3;
        final double maxAccelerationAut = 0.3;
        final int trackingPeriod = 100; // ms

        double distanceToStop = computeDistanceToStop(maxVelocityHum, maxAccelerationHum);

        VehicleSize vehicleSizeHum = new VehicleSize(3, 2, 1, 1, 0.5, 0.5);
        VehicleSize vehicleSizeAut1 = new VehicleSize(3, 2, 1, 1, 0.5, 0.5);
        VehicleSize vehicleSizeAut2 = vehicleSizeAut1;
        VehicleSize vehicleSizeAut3 = vehicleSizeAut1;

        AutonomousVehicle.planningAlgorithm = ReedsSheppCarPlanner.PLANNING_ALGORITHM.RRTConnect; // default
        //AutonomousVehicle.planningAlgorithm = ReedsSheppCarPlanner.PLANNING_ALGORITHM.PRMstar; // too slow
        //AutonomousVehicle.planningAlgorithm = ReedsSheppCarPlanner.PLANNING_ALGORITHM.SPARS; // too slow

        AutonomousVehicle hum0 = null;
        AutonomousVehicle aut1 = null;
        AutonomousVehicle aut2 = null;
        AutonomousVehicle aut3 = null;
        AutonomousVehicle aut4 = null;
        AutonomousVehicle aut5 = null;

        Color colorHum = Color.ORANGE;
        Color colorAut = Color.BLUE; // light blue: new Color(147, 187, 230)

        // TODO: `maxAcceleration` passed here is not used by `tec`.
        if (scenario != Scenario.BASELINE_AUTOMATED_ONLY) {
            hum0 = new HumanDrivenVehicle(0, 0, colorHum, colorHum, maxVelocityHum, maxAccelerationHum);
        } else if (isCrossMode) {
            hum0 = new AutonomousVehicle(0, 0, colorAut, colorAut, isCrossMode ? maxVelocityHum : 2, maxAccelerationAut);
        }
        aut1 = new AutonomousVehicle(1, 0, colorAut, colorAut, isCrossMode ? 5 : 2, maxAccelerationAut);
        aut2 = new AutonomousVehicle(2, 0, colorAut, colorAut, isCrossMode ? 5 : 3, maxAccelerationAut);
        if (aut3Start != null) {
            aut3 = new AutonomousVehicle(3, 0, colorAut, colorAut, isCrossMode ? 5 : 4, maxAccelerationAut);
        }
        //aut4 = new AutonomousVehicle(4, 0, Color.YELLOW, Color.YELLOW, maxVelocityAut, maxAccelerationAut, xLength, yLength);
        //aut5 = new AutonomousVehicle(5, 0, Color.YELLOW, Color.YELLOW, maxVelocityAut, maxAccelerationAut, xLength, yLength);

        //hum0.isAdaptiveTracker = true;
        //aut1.isAdaptiveTracker = true;
        //aut2.isAdaptiveTracker = true;
        //aut3.isAdaptiveTracker = true;

        AdaptiveTrajectoryEnvelopeTrackerRK4.emergencyBreaker = new EmergencyBreaker(false, false);

        TrajectoryEnvelopeCoordinatorSimulation tec = new TrajectoryEnvelopeCoordinatorSimulation(2000, 1000, 0, 0, trackingPeriod);
        tec.robotIDToParkingDuration.put(0, -10000); // -1000 means a random value in the range 0..1000 ms
        tec.setupSolver(0, 100000000);
        tec.startInference();

        if (hum0 != null) hum0.registerInTec(tec, vehicleSizeHum);
        if (aut1 != null) aut1.registerInTec(tec, vehicleSizeAut1);
        if (aut2 != null) aut2.registerInTec(tec, vehicleSizeAut2);
        if (aut3 != null) aut3.registerInTec(tec, vehicleSizeAut3);

        if (hum0 != null) tec.placeRobot(hum0.getID(), humStart);
        if (aut1 != null) tec.placeRobot(aut1.getID(), aut1Start);
        if (aut2 != null) tec.placeRobot(aut2.getID(), aut2Start);
        if (aut3 != null) tec.placeRobot(aut3.getID(), aut3Start);
        if (aut4 != null) tec.placeRobot(aut4.getID(), aut4Start);
        if (aut5 != null) tec.placeRobot(aut5.getID(), aut5Start);

        Heuristics heuristics = new Heuristics();
        switch (scenario) {
            case BASELINE_IDEAL_DRIVER_AUTOMATED_FIRST_COL1:
            case BASELINE_IDEAL_DRIVER_AUTOMATED_FIRST:
            case CROSS_AUTOMATED_FIRST_FORCING_DOWN_EVERYWHERE:
            case CROSS_AUTOMATED_FIRST_FORCING_DOWN_UP_EVERYWHERE:
            default:
                tec.addComparator(heuristics.automatedFirst());
                tec.addComparator(heuristics.closest());
                break;
            case BASELINE_IDEAL_DRIVER_HUMAN_FIRST:
                tec.addComparator(heuristics.humanFirst());
                tec.addComparator(heuristics.closest());
                break;
            case BASELINE_IDEAL_DRIVER_FIRST_COME:
            case BASELINE_AUTOMATED_ONLY:
            case CROSS_FIRST_COME_FORCING_DOWN_EVERYWHERE:
            case CROSS_FIRST_COME_FORCING_DOWN_UP_EVERYWHERE:
                tec.addComparator(heuristics.closest());
                break;
        }

        tec.setUseInternalCriticalPoints(false);
        tec.setYieldIfParking(true);
        tec.setBreakDeadlocks(true, false, false);

        var viz = new BrowserVisualization();
        viz.setMap(YAML_FILE);
        viz.setInitialTransform(7.0, 5.0, 5.0);
        tec.setVisualization(viz);

        Missions.setMap(YAML_FILE);
        Missions.startMissionDispatcher(tec);
        if (hum0 != null) Missions.loopMissions.put(hum0.getID(), ishumLoop);

        final boolean isSingleMissionInBothDirections = false;
        if (hum0 != null) Missions.enqueueMissions(hum0, humStart, humMiddle, humFinish, isSingleMissionInBothDirections);
        if (aut1 != null) Missions.enqueueMissions(aut1, aut1Start, aut1Finish, isSingleMissionInBothDirections);
        if (aut2 != null) Missions.enqueueMissions(aut2, aut2Start, aut2Finish, isSingleMissionInBothDirections);
        if (aut3 != null) Missions.enqueueMissions(aut3, aut3Start, aut3Finish, isSingleMissionInBothDirections);
        if (aut4 != null) Missions.enqueueMissions(aut4, aut4Start, aut4Finish, isSingleMissionInBothDirections);
        if (aut5 != null) Missions.enqueueMissions(aut5, aut5Start, aut5Finish, isSingleMissionInBothDirections);

        final AutonomousVehicle hum0Final = hum0;
        new GatedThread("forcing thread") {
            @Override
            public void runCore() {
                switch (scenario) {
                    case BASELINE_IDEAL_DRIVER_AUTOMATED_FIRST_COL1:
                    case BASELINE_IDEAL_DRIVER_AUTOMATED_FIRST:
                    case BASELINE_AUTOMATED_ONLY:
                    case BASELINE_IDEAL_DRIVER_HUMAN_FIRST:
                    case BASELINE_IDEAL_DRIVER_FIRST_COME:
                        return;
                }
                if (hum0Final == null) {
                    return;
                }

                TraitA traitA;
                TraitB traitB;
                TraitC traitC;
                TraitD traitD;
                switch (scenario) {
                    case CROSS_FIRST_COME_FORCING_DOWN_EVERYWHERE:
                    case CROSS_AUTOMATED_FIRST_FORCING_DOWN_EVERYWHERE:
                        traitA = TraitA.A_DOWN_EVERYWHERE;
                        traitB = TraitB.BP;
                        traitC = TraitC.C1;
                        traitD = TraitD.DC;
                        break;
                    case CROSS_FIRST_COME_FORCING_DOWN_UP_EVERYWHERE:
                    case CROSS_AUTOMATED_FIRST_FORCING_DOWN_UP_EVERYWHERE:
                        traitA = TraitA.A_DOWN_UP_EVERYWHERE;
                        traitB = TraitB.BP;
                        traitC = TraitC.C1;
                        traitD = TraitD.DC;
                        break;
                    default:
                        traitA = TraitA.valueOf("A" + scenario.toString().charAt(2));
                        traitB = TraitB.valueOf("B" + scenario.toString().charAt(3));
                        traitC = TraitC.valueOf("C" + scenario.toString().charAt(4));
                        traitD = TraitD.valueOf("D" + scenario.toString().charAt(5));
                        break;
                }

                assert(Forcing.priorityDistance == Double.NEGATIVE_INFINITY);
                assert(Forcing.priorityDistanceMin == Double.NEGATIVE_INFINITY);
                assert(Forcing.stopDistance == Double.NEGATIVE_INFINITY);
                assert(Forcing.stopDistanceMin == Double.NEGATIVE_INFINITY);
                assert(! Forcing.isGlobalTemporaryStop);
                assert(Forcing.isResetAfterCurrentCrossroad);

                // Derive information from traits:
                boolean isStop = traitB == TraitB.BS;

                switch (traitC) {
                    case C1:
                        Forcing.priorityDistance = 10;
                        break;
                    case C2:
                        Forcing.priorityDistance = 20;
                        break;
                    case C3:
                        Forcing.priorityDistance = 30;
                        break;
                    case CG:
                        Forcing.priorityDistance = Double.POSITIVE_INFINITY;
                        // Even on `isStop` (for `isResetAfterCurrentCrossroad` to work).

                        if (isStop) {
                            Forcing.isGlobalTemporaryStop = true;
                        }
                        break;
                }
                if (isStop) {
                    Forcing.stopDistance = Forcing.priorityDistance;
                }

                boolean isManualRestoring = traitD == TraitD.DM;

                // Propagate the information:
                Checkpoint[] checkpointsForcing = new Checkpoint[]{};
                Checkpoint[] checkpointsResumingRobots = new Checkpoint[]{};
                Checkpoint[] checkpointsRestoringPriorities = new Checkpoint[]{};

                switch (traitA) {
                    case AD:
                        assert ! isCrossMode;
                        checkpointsForcing = new Checkpoint[]{
                                new Checkpoint(humStart.getY() - 4.0, false, false),
                        };
                        break;
                    case AU:
                        assert ! isCrossMode;
                        checkpointsForcing = new Checkpoint[]{
                                new Checkpoint(humStart.getY() - 4.0, false, false),
                                new Checkpoint(humFinish.getY() + 4.0, false, true),
                        };
                        break;
                    case A3:
                        assert ! isCrossMode;
                        checkpointsForcing = new Checkpoint[]{
                                new Checkpoint(22.5, false, false),
                        };
                        break;
                    case A_DOWN_EVERYWHERE:
                        assert isCrossMode;
                        checkpointsForcing = new Checkpoint[]{
                                new Checkpoint(25.95 - vehicleSizeHum.length, true, true),
                                new Checkpoint(34.05 + vehicleSizeHum.length, false, false),
                        };
                        break;
                    case A_DOWN_UP_EVERYWHERE:
                        assert isCrossMode;
                        checkpointsForcing = new Checkpoint[]{
                                new Checkpoint(25.95 - vehicleSizeHum.length, true, true),
                                new Checkpoint(34.05 + vehicleSizeHum.length, false, false),
                                new Checkpoint(25.76 - vehicleSizeHum.length, false, true),
                                new Checkpoint(33.95 + vehicleSizeHum.length, true, false),
                        };
                        break;
                }

                if (isManualRestoring) {
                    Forcing.isResetAfterCurrentCrossroad = false;
                    switch (traitA) {
                        case AD:
                        case A3:
                            checkpointsRestoringPriorities = checkpointsResumingRobots = new Checkpoint[]{
                                    new Checkpoint(humFinish.getY(), false, false),
                            };
                            break;
                        case AU:
                            checkpointsRestoringPriorities = checkpointsResumingRobots = new Checkpoint[]{
                                    new Checkpoint(humFinish.getY(), false, false),
                                    new Checkpoint(humStart.getY(), false, true),
                            };
                            break;
                    }
                }

                if (!isStop) {
                    checkpointsRestoringPriorities = checkpointsResumingRobots = new Checkpoint[] {};
                }

                // Apply the information:
                KnobsAfterForcing knobsAfterForcing = null;
                RobotReport rrAtForcingStart = null;
                while (true) {
                    boolean isForcingNow = false;
                    boolean isResumingNow = false;
                    boolean isRestoringNow = false;

                    for (Checkpoint c : checkpointsForcing) {
                        if (c.isPassed(hum0Final)) {
                            isForcingNow = true;
                        }
                    }

                    for (Checkpoint c : checkpointsResumingRobots) {
                        if (c.isPassed(hum0Final)) {
                            isResumingNow = true;
                        }
                    }

                    for (Checkpoint c : checkpointsRestoringPriorities) {
                        if (c.isPassed(hum0Final)) {
                            isRestoringNow = true;
                        }
                    }

                    if (isForcingNow) {
                        assert ! isResumingNow;
                        assert ! isRestoringNow;
                        // Because that's perhaps not fully supported.
                    }

                    if (isForcingNow) {
                        knobsAfterForcing = Forcing.forceDriving(hum0Final.getID());
                        if (knobsAfterForcing != null) {
                            rrAtForcingStart = hum0Final.getCurrentRobotReport();
                        }
                    }

                    if (isResumingNow) {
                        assert rrAtForcingStart != null;
                        knobsAfterForcing.resumeRobots();
                    }
                    if (isRestoringNow) {
                        assert rrAtForcingStart != null;
                        knobsAfterForcing.restorePriorities();
                    }
                    if (isResumingNow || isRestoringNow) {
                        rrAtForcingStart = null;
                    }

                    if (rrAtForcingStart != null) {
                        if (hum0Final.getCurrentRobotReport().getPathIndex() < rrAtForcingStart.getPathIndex()) {
                            // A new mission has started but forcing hasn't stopped. Let's stop it now.
                            // This is a hack: we should rely not on explicit positions like
                            // `ysUpwardsRestoringPriorities` (which are sometimes not reached when mission ends)
                            // but rather on events like "the first crossroad has passed" and "the mission has ended".
                            // A (slightly) better approach would be to compare mission IDs (that can be stored in RRs).
                            knobsAfterForcing.resumeRobots();
                            knobsAfterForcing.restorePriorities();
                        } else {
                            double distanceTraveled = hum0Final.getCurrentRobotReport().getDistanceTraveled() - rrAtForcingStart.getDistanceTraveled();
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
