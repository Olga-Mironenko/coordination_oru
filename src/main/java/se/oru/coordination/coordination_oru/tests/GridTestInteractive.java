package se.oru.coordination.coordination_oru.tests;

import org.metacsp.multi.spatioTemporal.paths.Pose;
import se.oru.coordination.coordination_oru.code.AbstractVehicle;
import se.oru.coordination.coordination_oru.code.AutonomousVehicle;
import se.oru.coordination.coordination_oru.code.Heuristics;
import se.oru.coordination.coordination_oru.code.HumanDrivenVehicle;
import se.oru.coordination.coordination_oru.motionplanning.ompl.ReedsSheppCarPlanner;
import se.oru.coordination.coordination_oru.simulation2D.TrajectoryEnvelopeCoordinatorSimulation;
import se.oru.coordination.coordination_oru.tests.util.Demo;
import se.oru.coordination.coordination_oru.tests.util.GridMapConstants;
import se.oru.coordination.coordination_oru.util.*;

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
            scenarioString = Scenario.HUMAN_FIRST.toString();
        }
        Scenario scenario = Scenario.valueOf(scenarioString);
        AbstractVehicle.scenarioId = String.valueOf(scenario);

        HumanControl.isEnabledForBrowser = true;
//        BrowserVisualization.isExtendedText = false;

        final double workMinutes = 60;
        final long endTimestamp = System.currentTimeMillis() + Math.round(workMinutes * 60 * 1000);

        final String YAML_FILE = "maps/map-grid.yaml";

//        final Pose humStart = GridMapConstants.column2Top;
//        final Pose humStart = GridMapConstants.column2Row1;
        final Pose humStart = GridMapConstants.column1TopStart;
//        final Pose humFinish = GridMapConstants.column2Bottom;
//        final Pose humFinish = GridMapConstants.column2Row2;
        final Pose humFinish = GridMapConstants.turnAround(GridMapConstants.row1RightStart);

        final Pose aut1Start = GridMapConstants.row1LeftStart;
        final Pose aut1Finish = GridMapConstants.turnAround(GridMapConstants.row1RightStart);

        final Pose aut2Start = GridMapConstants.row2LeftStart;
        final Pose aut2Finish = GridMapConstants.turnAround(GridMapConstants.row2RightStart);

        final Pose aut3Start = GridMapConstants.row3LeftStart;
        final Pose aut3Finish = GridMapConstants.turnAround(GridMapConstants.row3RightStart);

        final double maxVelocityHum = 2.0;
        final double maxAccelerationHum = 2.0;
        final double maxVelocityAut = 5.0;
        final double maxAccelerationAut = 2.0;
        final int trackingPeriod = 100; // ms

        double xLength = 2.5;
        double yLength = 1.5;
        double xLengthInner = 1.5;
        double yLengthInner = 1.0;
//        double xLengthInner = 2.4;
//        double yLengthInner = 1.4;

        AutonomousVehicle.planningAlgorithm = ReedsSheppCarPlanner.PLANNING_ALGORITHM.RRTConnect; // default
//        AutonomousVehicle.planningAlgorithm = ReedsSheppCarPlanner.PLANNING_ALGORITHM.PRMstar; // too slow
//        AutonomousVehicle.planningAlgorithm = ReedsSheppCarPlanner.PLANNING_ALGORITHM.SPARS; // too slow

        // TODO: `maxAcceleration` passed here is not used by `tec`.
        AutonomousVehicle hum0 = new HumanDrivenVehicle(0, Color.GREEN, Color.BLUE, maxVelocityHum, maxAccelerationHum, xLength, yLength);
        AutonomousVehicle aut1 = new AutonomousVehicle(1, 0, Color.YELLOW, Color.YELLOW, maxVelocityAut, maxAccelerationAut, xLength, yLength);
        AutonomousVehicle aut2 = new AutonomousVehicle(2, 0, Color.YELLOW, Color.YELLOW, maxVelocityAut, maxAccelerationAut, xLength, yLength);
        AutonomousVehicle aut3 = new AutonomousVehicle(3, 0, Color.YELLOW, Color.YELLOW, maxVelocityAut, maxAccelerationAut, xLength, yLength);

        TrajectoryEnvelopeCoordinatorSimulation tec = new TrajectoryEnvelopeCoordinatorSimulation(2000, 1000, 0, 0, trackingPeriod);
        tec.setupSolver(0, 100000000);
        tec.startInference();

        for (AbstractVehicle vehicle : new AbstractVehicle[] { hum0, aut1, aut2, aut3 }) {
            if (vehicle != null) {
                vehicle.registerInTec(tec, xLengthInner, yLengthInner);
            }
        }

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
        Missions.startMissionDispatcher(tec, endTimestamp);

        Missions.loopMissions.put(hum0.getID(), false);
        Missions.loopMissions.put(aut1.getID(), true);
        Missions.loopMissions.put(aut2.getID(), true);
        Missions.loopMissions.put(aut3.getID(), true);

        Missions.enqueueMissions(hum0, humStart, humFinish, false, true);
        Missions.enqueueMissions(aut1, aut1Start, aut1Finish, false);
        Missions.enqueueMissions(aut2, aut2Start, aut2Finish, false);
        Missions.enqueueMissions(aut3, aut3Start, aut3Finish, false);
    }
}
