package se.oru.coordination.coordination_oru.tests;

import java.awt.Color;

import com.vividsolutions.jts.geom.Coordinate;
import org.metacsp.multi.spatioTemporal.paths.Pose;

import se.oru.coordination.coordination_oru.code.*;
import se.oru.coordination.coordination_oru.motionplanning.ompl.ReedsSheppCarPlanner;
import se.oru.coordination.coordination_oru.simulation2D.EmergencyBreaker;
import se.oru.coordination.coordination_oru.simulation2D.TrajectoryEnvelopeCoordinatorSimulation;
import se.oru.coordination.coordination_oru.simulation2D.AdaptiveTrajectoryEnvelopeTrackerRK4;
import se.oru.coordination.coordination_oru.tests.util.Demo;
import se.oru.coordination.coordination_oru.tests.util.GridMapConstants;
import se.oru.coordination.coordination_oru.util.*;
import se.oru.coordination.coordination_oru.util.gates.GatedThread;

/**
 * Variations of forcing:
 * - (A) where forcing happens:
 *   D) first downwards
 *   U) both first downwards and first upwards
 * - (B) which reaction to forcing:
 *   P) priority change
 *   S) stops
 * - (C) who is affected:
 *   1) the robot before the first crossroad
 *   2) the robots before the first and second crossroads
 *   3) the robots before all three crossroads
 *   G) all robots regardless of their positions (globally)
 * - (D) when everything comes back to normal:
 *   C) after the first crossroad
 *   M) after the end of mission
 *
 * + baselines
 */
public class GridTest {
    enum TraitA { AD, AU }
    enum TraitB { BP, BS }

    enum TraitC { C1, C2, C3, CG }
    enum TraitD { DC, DM }

    enum Scenario {
        BASELINE_IDEAL_DRIVER_AUTOMATED_FIRST_COL1,

        BASELINE_IDEAL_DRIVER_AUTOMATED_FIRST, // DONE
        BASELINE_IDEAL_DRIVER_HUMAN_FIRST, // DONE
        BASELINE_IDEAL_DRIVER_FIRST_COME, // DONE

        S_DP1C, S_DP1M, S_DP2C, S_DP2M, S_DP3C, S_DP3M, S_DPGC, S_DPGM,
        S_DS1C, S_DS1M, S_DS2C, S_DS2M, S_DS3C, S_DS3M, S_DSGC, S_DSGM,
        S_UP1C, S_UP1M, S_UP2C, S_UP2M, S_UP3C, S_UP3M, S_UPGC, S_UPGM,
        S_US1C, S_US1M, S_US2C, S_US2M, S_US3C, S_US3M, S_USGC, S_USGM,
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
            scenarioString = Scenario.S_USGM.toString();
        }
        Scenario scenario = Scenario.valueOf(scenarioString);

        AbstractVehicle.scenarioId = String.valueOf(scenario);

        final double loopMinutes = 60;
        final long loopTime = System.currentTimeMillis() + Math.round(loopMinutes * 60 * 1000);

        final boolean ishumLoop = true;

        final String YAML_FILE = "maps/map-grid.yaml";

        final Pose humStart = scenario == Scenario.BASELINE_IDEAL_DRIVER_AUTOMATED_FIRST_COL1 ? GridMapConstants.column1Top : GridMapConstants.column2Top;
        final Pose humFinish = GridMapConstants.column2Bottom;

        final Pose aut1Start = GridMapConstants.row1Left;
        final Pose aut1Finish = GridMapConstants.row1Right;

        final Pose aut2Start = GridMapConstants.row2Left;
        final Pose aut2Finish = GridMapConstants.row2Right;

        final Pose aut3Start = GridMapConstants.row3Left;
        final Pose aut3Finish = GridMapConstants.row3Right;

        final Pose aut4Start = GridMapConstants.column1Top;
        final Pose aut4Finish = GridMapConstants.column1Bottom;

        final Pose aut5Start = GridMapConstants.row3Left;
        final Pose aut5Finish = GridMapConstants.row1Right;

        final double precisionCoefficient = 1;
        final double maxVelocity = 5.0 * precisionCoefficient;
        final double maxAcceleration = 2.0 * precisionCoefficient;
        final int trackingPeriod = (int) Math.round(100 / precisionCoefficient);

        double xLength = 2.5;
        double yLength = 1.5;
        double xLengthInner = 1.5;
        double yLengthInner = 1.0;

        HumanControl.targetVelocityHumanInitial = maxVelocity;
        HumanControl.targetVelocityHuman = maxVelocity;

        AutonomousVehicle.planningAlgorithm = ReedsSheppCarPlanner.PLANNING_ALGORITHM.RRTConnect; // default
        //AutonomousVehicle.planningAlgorithm = ReedsSheppCarPlanner.PLANNING_ALGORITHM.PRMstar; // too slow
        //AutonomousVehicle.planningAlgorithm = ReedsSheppCarPlanner.PLANNING_ALGORITHM.SPARS; // too slow

        AutonomousVehicle aut1 = null;
        AutonomousVehicle aut2 = null;
        AutonomousVehicle aut3 = null;
        AutonomousVehicle aut4 = null;
        AutonomousVehicle aut5 = null;

        // TODO: `maxAcceleration` passed here is not used by `tec`.
        AutonomousVehicle hum0 = new HumanDrivenVehicle(0, Color.GREEN, Color.BLUE, maxVelocity, maxAcceleration, xLength, yLength);
        aut1 = new AutonomousVehicle(1, 0, Color.YELLOW, Color.YELLOW, maxVelocity, maxAcceleration, xLength, yLength);
        aut2 = new AutonomousVehicle(2, 0, Color.YELLOW, Color.YELLOW, maxVelocity, maxAcceleration, xLength, yLength);
        aut3 = new AutonomousVehicle(3, 0, Color.YELLOW, Color.YELLOW, maxVelocity, maxAcceleration, xLength, yLength);
        //aut4 = new AutonomousVehicle(4, 0, Color.YELLOW, Color.YELLOW, maxVelocity, maxAcceleration, xLength, yLength);
        //aut5 = new AutonomousVehicle(5, 0, Color.YELLOW, Color.YELLOW, maxVelocity, maxAcceleration, xLength, yLength);

        AdaptiveTrajectoryEnvelopeTrackerRK4.emergencyBreaker = new EmergencyBreaker(false, false);

        TrajectoryEnvelopeCoordinatorSimulation tec = new TrajectoryEnvelopeCoordinatorSimulation(2000, 1000, maxVelocity, maxAcceleration, trackingPeriod);
        tec.setupSolver(0, 100000000);
        tec.startInference();

        Coordinate[] innerFootprint = AbstractVehicle.makeFootprint(xLengthInner, yLengthInner);
        for (AbstractVehicle vehicle : new AbstractVehicle[] { hum0, aut1, aut2, aut3, aut4, aut5 }) {
            if (vehicle != null) {
                vehicle.innerFootprint = innerFootprint;
                tec.setFootprint(vehicle.getID(), vehicle.getFootprint());
                tec.setInnerFootprint(vehicle.getID(), vehicle.innerFootprint);
            }
        }

        tec.placeRobot(hum0.getID(), humStart);
        if (aut1 != null) tec.placeRobot(aut1.getID(), aut1Start);
        if (aut2 != null) tec.placeRobot(aut2.getID(), aut2Start);
        if (aut3 != null) tec.placeRobot(aut3.getID(), aut3Start);
        if (aut4 != null) tec.placeRobot(aut4.getID(), aut4Start);
        if (aut5 != null) tec.placeRobot(aut5.getID(), aut5Start);

        Heuristics heuristics = new Heuristics();
        switch (scenario) {
            case BASELINE_IDEAL_DRIVER_AUTOMATED_FIRST_COL1:
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
        tec.setYieldIfParking(true);
        tec.setBreakDeadlocks(true, false, false);

        var viz = new BrowserVisualization();
        viz.setMap(YAML_FILE);
        viz.setInitialTransform(7.0, 5.0, 5.0);
        tec.setVisualization(viz);

        Missions.setMap(YAML_FILE);
        Missions.startMissionDispatcher(tec, loopTime);
        Missions.loopMissions.put(hum0.getID(), ishumLoop);

        final boolean isInverse = false;
        Missions.enqueueMissions(hum0, humStart, humFinish, isInverse);
        if (aut1 != null) Missions.enqueueMissions(aut1, aut1Start, aut1Finish, isInverse);
        if (aut2 != null) Missions.enqueueMissions(aut2, aut2Start, aut2Finish, isInverse);
        if (aut3 != null) Missions.enqueueMissions(aut3, aut3Start, aut3Finish, isInverse);
        if (aut4 != null) Missions.enqueueMissions(aut4, aut4Start, aut4Finish, isInverse);
        if (aut5 != null) Missions.enqueueMissions(aut5, aut5Start, aut5Finish, isInverse);

        new GatedThread("forcing thread") {
            @Override
            public void runCore() {
                switch (scenario) {
                    case BASELINE_IDEAL_DRIVER_AUTOMATED_FIRST_COL1:
                    case BASELINE_IDEAL_DRIVER_AUTOMATED_FIRST:
                    case BASELINE_IDEAL_DRIVER_HUMAN_FIRST:
                    case BASELINE_IDEAL_DRIVER_FIRST_COME:
                        return;
                }

                TraitA traitA = TraitA.valueOf("A" + scenario.toString().charAt(2));
                TraitB traitB = TraitB.valueOf("B" + scenario.toString().charAt(3));
                TraitC traitC = TraitC.valueOf("C" + scenario.toString().charAt(4));
                TraitD traitD = TraitD.valueOf("D" + scenario.toString().charAt(5));

                assert(Forcing.priorityDistance == Double.NEGATIVE_INFINITY);
                assert(Forcing.stopDistance == Double.NEGATIVE_INFINITY);
                assert(! Forcing.isGlobalTemporaryStop);
                assert(Forcing.isResetAfterCurrentCrossroad);

                // Derive information from traits:
                boolean isUpwards = traitA == TraitA.AU;
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
                double[] ysDownwardsForcing = new double[]{humStart.getY() - 4.0};
                double[] ysDownwardsResumingRobots = new double[]{};
                double[] ysDownwardsRestoringPriorities = new double[]{};

                double[] ysUpwardsForcing = new double[]{humFinish.getY() + 4.0};
                double[] ysUpwardsResumingRobots = new double[]{};
                double[] ysUpwardsRestoringPriorities = new double[]{};

                if (isManualRestoring) {
                    Forcing.isResetAfterCurrentCrossroad = false;
                    ysDownwardsRestoringPriorities = ysDownwardsResumingRobots = new double[]{humFinish.getY()};
                    ysUpwardsRestoringPriorities = ysUpwardsResumingRobots = new double[]{humStart.getY()};
                }

                if (!isUpwards) {
                    ysUpwardsForcing = ysUpwardsRestoringPriorities = ysUpwardsResumingRobots = new double[] {};
                }
                if (!isStop) {
                    ysDownwardsResumingRobots = ysUpwardsResumingRobots = new double[] {};
                }

                // Apply the information:
                KnobsAfterForcing knobsAfterForcing = null;
                Double positionAtForcingStart = null;
                while (true) {
                    boolean isForcingNow = false;
                    boolean isResumingNow = false;
                    boolean isRestoringNow = false;

                    for (double y : ysDownwardsForcing) {
                        if (hum0.isYPassedDownwards(y)) {
                            isForcingNow = true;
                        }
                    }
                    for (double y : ysUpwardsForcing) {
                        if (hum0.isYPassedUpwards(y)) {
                            isForcingNow = true;
                        }
                    }

                    for (double y : ysDownwardsResumingRobots) {
                        if (hum0.isYPassedDownwards(y)) {
                            isResumingNow = true;
                        }
                    }
                    for (double y : ysUpwardsResumingRobots) {
                        if (hum0.isYPassedUpwards(y)) {
                            isResumingNow = true;
                        }
                    }

                    for (double y : ysDownwardsRestoringPriorities) {
                        if (hum0.isYPassedDownwards(y)) {
                            isRestoringNow = true;
                        }
                    }
                    for (double y : ysUpwardsRestoringPriorities) {
                        if (hum0.isYPassedUpwards(y)) {
                            isRestoringNow = true;
                        }
                    }

                    if (isForcingNow) {
                        assert ! isResumingNow;
                        assert ! isRestoringNow;
                        // Because that's perhaps not fully supported.
                    }

                    if (isForcingNow) {
                        knobsAfterForcing = Forcing.forceDriving(hum0.getID());
                        positionAtForcingStart = hum0.getCurrentRobotReport().getDistanceTraveled();
                    }

                    if (isResumingNow) {
                        assert positionAtForcingStart != null;
                        knobsAfterForcing.resumeRobots();
                    }
                    if (isRestoringNow) {
                        assert positionAtForcingStart != null;
                        knobsAfterForcing.restorePriorities();
                    }
                    if (isResumingNow || isRestoringNow) {
                        positionAtForcingStart = null;
                    }

                    if (positionAtForcingStart != null) {
                        int index = hum0.getCurrentRobotReport().getPathIndex();
                        if (index == -1) {
                            // A new mission has started but forcing hasn't stopped. Let's stop it now.
                            // This is a hack: we should rely not on explicit positions like
                            // `ysUpwardsRestoringPriorities` (which are sometimes not reached when mission ends)
                            // but rather on events like "the first crossroad has passed" and "the mission has ended".
                            knobsAfterForcing.resumeRobots();
                            knobsAfterForcing.restorePriorities();
                        } else {
                            double distanceTraveled = hum0.getCurrentRobotReport().getDistanceTraveled() - positionAtForcingStart;
                            assert distanceTraveled >= 0; // otherwise, we are in a new mission, but restoring/resuming hasn't happened
                            if (!knobsAfterForcing.updateForcing(distanceTraveled)) {
                                positionAtForcingStart = null;
                            }
                        }
                    }

                    GatedThread.sleepWithoutTryCatch(100);
                }
            }
        }.start();
    }
}
