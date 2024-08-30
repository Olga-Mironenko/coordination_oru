package se.oru.coordination.coordination_oru.tests;

import org.metacsp.multi.spatioTemporal.paths.Pose;
import se.oru.coordination.coordination_oru.code.AbstractVehicle;
import se.oru.coordination.coordination_oru.code.AutonomousVehicle;
import se.oru.coordination.coordination_oru.code.Heuristics;
import se.oru.coordination.coordination_oru.code.HumanDrivenVehicle;
import se.oru.coordination.coordination_oru.motionplanning.ompl.ReedsSheppCarPlanner;
import se.oru.coordination.coordination_oru.simulation2D.AdaptiveTrajectoryEnvelopeTrackerRK4;
import se.oru.coordination.coordination_oru.simulation2D.EmergencyBreaker;
import se.oru.coordination.coordination_oru.simulation2D.TrajectoryEnvelopeCoordinatorSimulation;
import se.oru.coordination.coordination_oru.tests.util.Demo;
import se.oru.coordination.coordination_oru.util.*;

import java.awt.*;


public class ProductionLine {
    enum Scenario {
        MAIN,
    }

    public static void main(String[] args) {
        new Demo() {
            @Override
            protected void run(String scenarioString) {
                assert scenarioString == null;
                runDemo();
            }
        }.exec();
    }

    protected static void runDemo() {
//        final String scenarioString = System.getenv().get("SCENARIO");
        final String scenarioString = null;
        final Scenario scenario = scenarioString == null ? Scenario.MAIN :
                Scenario.valueOf(scenarioString);

        AbstractVehicle.scenarioId = String.valueOf(scenario);

        final double loopMinutes = 60;
        final long loopTime = System.currentTimeMillis() + Math.round(loopMinutes * 60 * 1000);

        final String YAML_FILE = "maps/production-map.yaml";

        final double xLeft = 4.0;
        final double xRight = 56.0;
        final double yTop = 56.5;
        final double yBottom = 3.0;

        final double xColumn1 = 14.5;
        final double xColumn2 = 30.0;
        final double xColumn3 = 45.7;
        final double yRow1 = 44.0;
        final double yRow2 = 30.0;
        final double yRow3 = 15.5;

        final double thetaDown = -Math.PI/2;
        final double thetaUp = Math.PI/2;
        final double thetaRight = 0;
        final double thetaLeft = Math.PI;

        final Pose column0Top = new Pose(35,135.00, thetaDown);
        final Pose column01Middle = new Pose(46.5,102, thetaDown);
        final Pose column1Top = new Pose(57.50,135.00, thetaDown);
        final Pose column2Top = new Pose(79.5, 135, thetaDown);
        final Pose column3Top = new Pose(99, 135, thetaDown);
        final Pose column4Top = new Pose(119.5, 135, thetaDown);
        final Pose column45Middle = new Pose(130, 102, thetaDown);
        final Pose column5Top = new Pose(141, 135, thetaDown);
        final Pose column6Top = new Pose(176, 136, thetaDown);

        final Pose column1Bottom = new Pose(57.5, 35, thetaUp);
        final Pose column2Bottom = new Pose(79.5, 87, thetaUp); // left is for the robot 0 to look down
        final Pose column3Bottom = new Pose(99, 35, thetaUp);
        final Pose column4Bottom = new Pose(119.5, 87, thetaUp);
        final Pose column5Middle = new Pose(141.0, 95, thetaUp);
        final Pose bottomTunnel = new Pose(90, 15, thetaUp);

        final Pose row1Left = new Pose(xLeft, yRow1, thetaRight);
        final Pose row2Left = new Pose(xLeft, yRow2, thetaRight);
        final Pose row3Left = new Pose(xLeft, yRow3, thetaRight);

        final Pose row1Right = new Pose(xRight, yRow1, thetaLeft);
        final Pose row2Right = new Pose(xRight, yRow2, thetaLeft);
        final Pose row3Right = new Pose(xRight, yRow3, thetaLeft);

        final Pose centerDownward = new Pose(xColumn2, yRow2, thetaDown);

        final Pose mainTunnelLeft = new Pose(23.00,74.50, -Math.PI);
        final Pose mainTunnelRight = new Pose(200.05,74.50, Math.PI);

//        final Pose humStart = mainTunnelRight;
//        final Pose humFinish = mainTunnelLeft;
        final Pose humStart = column5Top;
//        final Pose humFinish = bottomTunnel;
//        final Pose humFinish = null;
        final Pose humFinish = column5Middle;

//        final Pose aut5Start = column5Top;
//        final Pose aut5Start = column45Middle;
//        final Pose aut5Start = aut4Start;
//        final Pose aut5Finish = bottomTunnel;
//        final Pose aut5Finish = column01Middle;
//        final Pose aut5Finish = aut4Finish;
        final Pose aut5Start = mainTunnelRight;
        final Pose aut5Finish = mainTunnelLeft;

        final boolean ishumLoop = true;

        final Pose aut1Start = column1Top;
        final Pose aut1Finish = column1Bottom;

        final Pose aut2Start = column2Top;
        final Pose aut2Finish = column2Bottom;

        final Pose aut3Start = column3Top;
        final Pose aut3Finish = column3Bottom;

        final Pose aut4Start = column4Top;
        final Pose aut4Finish = column4Bottom;


        final double precisionCoefficient = 1;
        final double maxVelocity = 15.0 * precisionCoefficient;
        final double maxAcceleration = 6.0 * precisionCoefficient;
        final double maxVelocityBig = maxVelocity / 3;
        final double maxAccelerationBig = maxAcceleration / 3;
        final int trackingPeriod = (int) Math.round(100 / precisionCoefficient);

        double xLengthBigOuter = 8.0;
        double yLengthBigOuter = 8.0;
        double xLengthBigInner = 7.0;
        double yLengthBigInner = 7.0;

        double xLengthSmallOuter = 4.0;
        double yLengthSmallOuter = 3.5;
        double xLengthSmallInner = 3.0;
        double yLengthSmallInner = 3.0;

        double xLengthHumanOuter = 3.0;
        double yLengthHumanOuter = 2.5;
        double xLengthHumanInner = 2.0; // -2..2 -> length=4
        double yLengthHumanInner = 2.0;

        HumanControl.targetVelocityHumanInitial = maxVelocity;
        HumanControl.targetVelocityHuman = maxVelocity;

        AutonomousVehicle.planningAlgorithm = ReedsSheppCarPlanner.PLANNING_ALGORITHM.RRTConnect; // default
        //AutonomousVehicle.planningAlgorithm = ReedsSheppCarPlanner.PLANNING_ALGORITHM.PRMstar; // too slow
        //AutonomousVehicle.planningAlgorithm = ReedsSheppCarPlanner.PLANNING_ALGORITHM.SPARS; // too slow

        AutonomousVehicle hum0 = null;
        AutonomousVehicle aut1 = null;
        AutonomousVehicle aut2 = null;
        AutonomousVehicle aut3 = null;
        AutonomousVehicle aut4 = null;
        AutonomousVehicle aut5 = null;

        AdaptiveTrajectoryEnvelopeTrackerRK4.emergencyBreaker = new EmergencyBreaker(false, false);

        TrajectoryEnvelopeCoordinatorSimulation tec = new TrajectoryEnvelopeCoordinatorSimulation(2000, 1000, maxVelocity, maxAcceleration, trackingPeriod);
        tec.setupSolver(0, 100000000);
        tec.startInference();

        hum0 = new HumanDrivenVehicle(0, Color.GREEN, Color.GREEN, maxVelocity, maxAcceleration, xLengthHumanOuter, yLengthHumanOuter);
        aut1 = new AutonomousVehicle(1, 0, Color.YELLOW, Color.YELLOW, maxVelocity * 2, maxAcceleration * 2, xLengthSmallOuter, yLengthSmallOuter);
        aut2 = new AutonomousVehicle(2, 0, Color.YELLOW, Color.YELLOW, maxVelocity, maxAcceleration, xLengthSmallOuter, yLengthSmallOuter);
        aut3 = new AutonomousVehicle(3, 0, Color.YELLOW, Color.YELLOW, maxVelocity, maxAcceleration, xLengthSmallOuter, yLengthSmallOuter);
        aut4 = new AutonomousVehicle(4, 0, Color.YELLOW, Color.YELLOW, maxVelocity * 2, maxAcceleration, xLengthSmallOuter, yLengthSmallOuter);
        aut5 = new AutonomousVehicle(5, 0, Color.RED, Color.RED, maxVelocityBig, maxAccelerationBig, xLengthBigOuter, yLengthBigOuter);

        if (hum0 != null) hum0.registerInTec(tec, xLengthHumanInner, yLengthHumanInner);
        if (aut5 != null) aut5.registerInTec(tec, xLengthBigInner, yLengthBigInner);
        for (AbstractVehicle vehicle : new AbstractVehicle[] { aut1, aut2, aut3, aut4 }) {
            if (vehicle != null) {
                vehicle.registerInTec(tec, xLengthSmallInner, yLengthSmallInner);
            }
        }

        if (hum0 != null) tec.placeRobot(hum0.getID(), humStart);
        if (aut1 != null) tec.placeRobot(aut1.getID(), aut1Start);
        if (aut2 != null) tec.placeRobot(aut2.getID(), aut2Start);
        if (aut3 != null) tec.placeRobot(aut3.getID(), aut3Start);
        if (aut4 != null) tec.placeRobot(aut4.getID(), aut4Start);
        if (aut5 != null) tec.placeRobot(aut5.getID(), aut5Start);

        Heuristics heuristics = new Heuristics();
        switch (scenario) {
            case MAIN:
                tec.addComparator(heuristics.highestIDNumber());
                break;
        }

        tec.setUseInternalCriticalPoints(false);
        tec.setYieldIfParking(true);
        tec.setBreakDeadlocks(true, false, false);

        var viz = new BrowserVisualization();
        viz.setMap(YAML_FILE);
        viz.setFontScale(4);
        viz.setInitialTransform(4.0, 5.0, 5.0);
        tec.setVisualization(viz);

        Missions.setMap(YAML_FILE);
        Missions.startMissionDispatchers(tec, loopTime);
        if (hum0 != null) Missions.loopMissions.put(hum0.getID(), ishumLoop);

        if (hum0 != null) Missions.enqueueMissions(hum0, humStart, humFinish, false);
        final boolean isInverse = false;
        if (aut1 != null) Missions.enqueueMissions(aut1, aut1Start, aut1Finish, isInverse);
        if (aut2 != null) Missions.enqueueMissions(aut2, aut2Start, aut2Finish, isInverse);
        if (aut3 != null) Missions.enqueueMissions(aut3, aut3Start, aut3Finish, isInverse);
        if (aut4 != null) Missions.enqueueMissions(aut4, aut4Start, aut4Finish, isInverse);
        if (aut5 != null) Missions.enqueueMissions(aut5, aut5Start, aut5Finish, isInverse);
    }
}
