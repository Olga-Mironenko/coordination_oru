package se.oru.coordination.coordination_oru.tests;

import org.metacsp.multi.spatioTemporal.paths.Pose;
import se.oru.coordination.coordination_oru.code.*;
import se.oru.coordination.coordination_oru.simulation2D.TrajectoryEnvelopeCoordinatorSimulation;
import se.oru.coordination.coordination_oru.tests.util.Demo;
import se.oru.coordination.coordination_oru.tests.util.GridMapConstants;
import se.oru.coordination.coordination_oru.util.*;
import se.oru.coordination.coordination_oru.util.gates.GatedThread;
import se.oru.coordination.coordination_oru.util.gates.Timekeeper;

import java.awt.*;

public class GridTestInteractive {
    enum Scenario {
        AUTOMATED_FIRST,
        HUMAN_FIRST,
        FIRST_COME,
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
            scenarioString = Scenario.AUTOMATED_FIRST.toString();
        }
        Scenario scenario = Scenario.valueOf(scenarioString);
        AbstractVehicle.scenarioId = String.valueOf(scenario);

        boolean isMapCleaning = false;

        HumanControl.isEnabledForBrowser = true;
//        BrowserVisualization.isExtendedText = false;

//        Timekeeper.setVirtualSecondsPassedMax(15);
        Timekeeper.setVirtualMinutesPassedMax(60);
//        Timekeeper.realMillisPassedMax = 10 * 1000;

        final String YAML_FILE = isMapCleaning ? "maps/map-grid-gradient.yaml" : "maps/map-grid.yaml";

//        final Pose humStart = GridMapConstants.column1Row1Down;
//        final Pose humStart = GridMapConstants.column2Row1;
//        final Pose humStart = GridMapConstants.column3Row1Down;
//        final Pose humFinish = GridMapConstants.column2Bottom;
//        final Pose humFinish = GridMapConstants.column2Row2;

        final Pose humStart = GridMapConstants.column2TopStart;
 //        final Pose humStart = GridMapConstants.turnAround(GridMapConstants.column2BottomStart);
//        final Pose humFinish = GridMapConstants.column2Row1Right;
        final Pose humFinish = GridMapConstants.turnAround(GridMapConstants.column3Row2Down);

//        final Pose humFinish = GridMapConstants.turnAround(GridMapConstants.row1RightStart);
//        final Pose humFinish = GridMapConstants.turnAround(
//                GridMapConstants.between(
//                        GridMapConstants.column2Row1Down,
//                        GridMapConstants.column3Row1Down
//                )
//        );

        final Pose aut1Start = GridMapConstants.row1LeftStart;
//        final Pose aut1Start = GridMapConstants.turnAround(
//                GridMapConstants.shiftX(
//                        GridMapConstants.row1LeftStart,
//                        -1.35
//                )
//        );
        final Pose aut1Finish = GridMapConstants.turnAround(GridMapConstants.column3Row2Down);

        final Pose aut2Start = GridMapConstants.row2LeftStart;
        final Pose aut2Finish = GridMapConstants.turnAround(GridMapConstants.column3Row2Down);

        final Pose aut3Start = GridMapConstants.row3LeftStart;
        final Pose aut3Finish = GridMapConstants.turnAround(GridMapConstants.column3Row2Down);

        final double maxVelocityHum = 3.0;
        final double maxAccelerationHum = 2.0;
        final double maxAccelerationAut = 0.4;

        VehicleSize vehicleSizeHum = new VehicleSize(3, 2, 1, 1, 0.5, 0.5);
        VehicleSize vehicleSizeAut1 = new VehicleSize(3, 2, 1, 1, 0.5, 0.5);
        VehicleSize vehicleSizeAut2 = vehicleSizeAut1;
        VehicleSize vehicleSizeAut3 = vehicleSizeAut1;

//        AutonomousVehicle.planningAlgorithm = ReedsSheppCarPlanner.PLANNING_ALGORITHM.RRTConnect; // default
//        AutonomousVehicle.planningAlgorithm = ReedsSheppCarPlanner.PLANNING_ALGORITHM.PRMstar; // too slow
//        AutonomousVehicle.planningAlgorithm = ReedsSheppCarPlanner.PLANNING_ALGORITHM.SPARS; // too slow
//        AutonomousVehicle.planningAlgorithm = ReedsSheppCarPlanner.PLANNING_ALGORITHM.RRTstar; // too slow

        // TODO: `maxAcceleration` passed here is not used by `tec`.
        AutonomousVehicle hum0 = new HumanDrivenVehicle(0, 0, Color.ORANGE, Color.ORANGE, maxVelocityHum, maxAccelerationHum);
//        AutonomousVehicle hum0 = new AutonomousVehicle(0, 0, Color.BLUE, Color.BLUE, maxVelocityHum, maxAccelerationHum);
        AutonomousVehicle aut1 = new AutonomousVehicle(1, 0, Color.BLUE, Color.BLUE, 10, maxAccelerationAut);
        AutonomousVehicle aut2 = new AutonomousVehicle(2, 0, Color.BLUE, Color.BLUE, 10, maxAccelerationAut);
        AutonomousVehicle aut3 = new AutonomousVehicle(3, 0, Color.BLUE, Color.BLUE, 10, maxAccelerationAut);

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
            case AUTOMATED_FIRST:
            default:
                tec.addComparator(heuristics.automatedFirst());
                break;
            case HUMAN_FIRST:
                tec.addComparator(heuristics.humanFirst());
                break;
            case FIRST_COME:
                tec.addComparator(heuristics.closest());
                break;
        }

        tec.setUseInternalCriticalPoints(false);
        tec.setYieldIfParking(false);
        tec.setBreakDeadlocks(true, false, false);

        Missions.setMap(YAML_FILE); // before BrowserVisualization

        var viz = new BrowserVisualization();
        viz.setInitialTransform(7.0, 5.0, 5.0);
        tec.setVisualization(viz);

        Missions.startMissionDispatcher(tec);

        new GatedThread("enqueue thread") { // path planning takes a while
            @Override
            public void runCore() {
                Missions.loopMissions.put(hum0.getID(), false);
                Missions.enqueueMissions(
                        new MissionBlueprint(hum0, humStart, humFinish)
                );

                double xMaxClean = Missions.getDynamicMap().getWidthMeters() - GridMapConstants.xLeft;

                Missions.loopMissions.put(aut1.getID(), ! isMapCleaning);
                MissionBlueprint mb1 = new MissionBlueprint(aut1, aut1Start, aut1Finish).setDirection(
                        MissionBlueprint.Direction.FORWARD_BACKWARD_SINGLE_MISSION
                );
                if (isMapCleaning) {
                    mb1.setIsToCleanForward(true).setRadiusClean(6).setDxClean(2).setXMaxClean(xMaxClean);
                }
                Missions.enqueueMissions(mb1);

                Missions.loopMissions.put(aut2.getID(), true);
                Missions.enqueueMissions(
                        new MissionBlueprint(aut2, aut2Start, aut2Finish).setDirection(
                                MissionBlueprint.Direction.FORWARD_BACKWARD_SINGLE_MISSION
                        )
                );

                Missions.loopMissions.put(aut3.getID(), ! isMapCleaning);
                MissionBlueprint mb3 = new MissionBlueprint(aut3, aut3Start, aut3Finish).setDirection(
                        MissionBlueprint.Direction.FORWARD_BACKWARD_SINGLE_MISSION
                );
                if (isMapCleaning) {
                    mb3.setIsToCleanForward(true).setRadiusClean(5).setDxClean(1).setXMaxClean(xMaxClean);
                }
                Missions.enqueueMissions(mb3);
            }
        }.start();

        /*
        new GatedThread("cleanCircle thread") {
            @Override
            public void runCore() {
                while (true) {
                    int millis = Timekeeper.getVirtualMillisPassed();

                    for (int i = 1; i <= 15; i += 1) {
                        if (i * 1000 <= millis && millis < i * 1000 + 100) {
                            Missions.getDynamicMap().cleanCircle(
                                    GridMapConstants.shiftX(GridMapConstants.column3Row1Down, i).getPosition(),
                                    4
                            );
                            Missions.onDynamicMapUpdate();
                            break;
                        }
                    }

                    GatedThread.sleepWithoutTryCatch(100);
                }
            }
        }.start();
         */

        new GatedThread("clicking thread") {
            @Override
            public void runCore() {
                while (true) {
                    int millis = Timekeeper.getVirtualMillisPassed();

                    if (millis >= 50 * 1000) {
                        // Artificial rerouting (imitation of a click):
//                        while (true) {
//                            boolean isOk = HumanControl.moveRobot(
//                                    hum0.getID(),
//                                    GridMapConstants.turnAround(GridMapConstants.column3Row2Down)
//                            );
//                            if (isOk) {
//                                break;
//                            }
//                        }

                        break;
                    }

                    GatedThread.sleepWithoutTryCatch(100);
                }
            }
        }.start();
    }
}
