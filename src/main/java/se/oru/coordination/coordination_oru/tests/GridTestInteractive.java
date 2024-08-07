package se.oru.coordination.coordination_oru.tests;

import org.metacsp.multi.spatioTemporal.paths.Pose;
import se.oru.coordination.coordination_oru.CriticalSection;
import se.oru.coordination.coordination_oru.code.*;
import se.oru.coordination.coordination_oru.motionplanning.ompl.ReedsSheppCarPlanner;
import se.oru.coordination.coordination_oru.simulation2D.TrajectoryEnvelopeCoordinatorSimulation;
import se.oru.coordination.coordination_oru.tests.util.Demo;
import se.oru.coordination.coordination_oru.tests.util.GridMapConstants;
import se.oru.coordination.coordination_oru.util.*;
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

        HumanControl.isEnabledForBrowser = true;
//        BrowserVisualization.isExtendedText = false;

//        Timekeeper.setVirtualSecondsPassedMax(15);
        Timekeeper.setVirtualMinutesPassedMax(60);
//        Timekeeper.realMillisPassedMax = 10 * 1000;

        final String YAML_FILE = "maps/map-grid.yaml";

//        final Pose humStart = GridMapConstants.column1Row1Down;
//        final Pose humStart = GridMapConstants.column2Row1;
//        final Pose humStart = GridMapConstants.column3Row1Down;
//        final Pose humFinish = GridMapConstants.column2Bottom;
//        final Pose humFinish = GridMapConstants.column2Row2;

        final Pose humStart = GridMapConstants.column1Row1Down;
 //        final Pose humStart = GridMapConstants.turnAround(GridMapConstants.column2BottomStart);
        final Pose humFinish = GridMapConstants.column2Row1Right;

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
        final Pose aut1Finish = GridMapConstants.turnAround(GridMapConstants.row1RightStart);

        final Pose aut2Start = GridMapConstants.row2LeftStart;
        final Pose aut2Finish = GridMapConstants.turnAround(GridMapConstants.row2RightStart);

        final Pose aut3Start = GridMapConstants.row3LeftStart;
        final Pose aut3Finish = GridMapConstants.turnAround(GridMapConstants.row3RightStart);

        final double maxVelocityHum = 3.0;
        final double maxAccelerationHum = 0.3;
        final double maxAccelerationAut = 0.3;

        double lengthVehicle = 3.0;
        double widthVehicle = 2.0;
        VehicleSize vehicleSizeHum = new VehicleSize(3, 2, 1, 1, 0.5, 0.5);
        VehicleSize vehicleSizeAut1 = new VehicleSize(3, 2, 1, 1, 0.5, 0.5);
        VehicleSize vehicleSizeAut2 = vehicleSizeAut1;
        VehicleSize vehicleSizeAut3 = vehicleSizeAut1;

        AutonomousVehicle.planningAlgorithm = ReedsSheppCarPlanner.PLANNING_ALGORITHM.RRTConnect; // default
//        AutonomousVehicle.planningAlgorithm = ReedsSheppCarPlanner.PLANNING_ALGORITHM.PRMstar; // too slow
//        AutonomousVehicle.planningAlgorithm = ReedsSheppCarPlanner.PLANNING_ALGORITHM.SPARS; // too slow
//        AutonomousVehicle.planningAlgorithm = ReedsSheppCarPlanner.PLANNING_ALGORITHM.RRTstar; // too slow

        // TODO: `maxAcceleration` passed here is not used by `tec`.
        AutonomousVehicle hum0 = new HumanDrivenVehicle(0, Color.ORANGE, Color.ORANGE, maxVelocityHum, maxAccelerationHum);
        AutonomousVehicle aut1 = new AutonomousVehicle(1, 0, Color.BLUE, Color.BLUE, 5, maxAccelerationAut);
        AutonomousVehicle aut2 = new AutonomousVehicle(2, 0, Color.BLUE, Color.BLUE, 5, maxAccelerationAut);
        AutonomousVehicle aut3 = new AutonomousVehicle(3, 0, Color.BLUE, Color.BLUE, 5, maxAccelerationAut);

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
                tec.addComparator(heuristics.highestIDNumber());
                break;
            case HUMAN_FIRST:
                tec.addComparator(heuristics.lowestIDNumber());
                break;
            case FIRST_COME:
                tec.addComparator(heuristics.closest());
                break;
        }

        tec.setUseInternalCriticalPoints(false);
        tec.setYieldIfParking(false);
        tec.setBreakDeadlocks(true, false, false);

        var viz = new BrowserVisualization();
        viz.setMap(YAML_FILE);
        viz.setInitialTransform(7.0, 5.0, 5.0);
        tec.setVisualization(viz);

        Missions.setMap(YAML_FILE);
        Missions.startMissionDispatcher(tec);

        Missions.loopMissions.put(hum0.getID(), false);
        Missions.loopMissions.put(aut1.getID(), true);
        Missions.loopMissions.put(aut2.getID(), true);
        Missions.loopMissions.put(aut3.getID(), true);

        Missions.enqueueMissions(hum0, humStart, null, humFinish, false, true);
        Missions.enqueueMissions(aut1, aut1Start, aut1Finish, false);
        Missions.enqueueMissions(aut2, aut2Start, aut2Finish, false);
        Missions.enqueueMissions(aut3, aut3Start, aut3Finish, false);
    }
}
