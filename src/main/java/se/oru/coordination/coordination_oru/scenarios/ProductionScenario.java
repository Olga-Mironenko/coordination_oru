package se.oru.coordination.coordination_oru.scenarios;

import com.vividsolutions.jts.geom.Coordinate;
import org.metacsp.multi.spatioTemporal.paths.Pose;
import se.oru.coordination.coordination_oru.ConstantAccelerationForwardModel;
import se.oru.coordination.coordination_oru.Mission;
import se.oru.coordination.coordination_oru.code.AutonomousVehicle;
import se.oru.coordination.coordination_oru.code.Heuristics;
import se.oru.coordination.coordination_oru.code.LookAheadVehicle;
import se.oru.coordination.coordination_oru.simulation2D.TrajectoryEnvelopeCoordinatorSimulation;
import se.oru.coordination.coordination_oru.util.BrowserVisualization;
import se.oru.coordination.coordination_oru.util.Missions;

import java.awt.*;
import java.io.FileNotFoundException;

public class ProductionScenario {
    public static void main(String[] args) throws FileNotFoundException, InterruptedException {

        String absolutePath = System.getProperty("user.dir");
        String resultsDirectory = absolutePath + "/src/main/java/se/oru/coordination/coordination_oru/results/lookAheadPaper_2023";
        final String YAML_FILE = "maps/map_production_scenario.yaml";
        double drillLookAheadDistance = 30;
        int intervalInSeconds = 1;
        int terminationInMinutes = 15;
        int numOfCallsForLookAheadRobot = 5;
        boolean visualization = true;
        boolean writeRobotReports = false;

        final Pose mainTunnelLeft = new Pose(4.05, 42.95, Math.PI);
        final Pose mainTunnelRight = new Pose(120.55, 40.75, Math.PI);
        final Pose drawPoint1 = new Pose(33.05, 77.35, -Math.PI / 2);
        final Pose drawPoint2 = new Pose(45.85, 77.35, -Math.PI / 2);
        final Pose drawPoint3 = new Pose(56.85, 77.35, -Math.PI / 2);
        final Pose drawPoint4 = new Pose(69.15, 77.35, -Math.PI / 2);
        final Pose drillPoint = new Pose(120.85, 78.65, -Math.PI / 2);
        final Pose drillPoint2 = new Pose(19.25, 9.75, -Math.PI / 2);
        final Pose orePass1 = new Pose(39.15, 9.95, -Math.PI / 2.7);
        final Pose orePass2 = new Pose(45.75, 53.25, -Math.PI / 2);
        final Pose orePass3 = new Pose(62.95, 9.75, -Math.PI / 2);
        final Pose orePass4 = new Pose(69.15, 53.15, -Math.PI / 2);

        final Pose[] autonomousRobotGoal1 = {orePass1};
        final Pose[] autonomousRobotGoal2 = {orePass2};
        final Pose[] autonomousRobotGoal3 = {orePass3};
        final Pose[] autonomousRobotGoal4 = {orePass4};
        final Pose[] autonomousRobotGoal5 = {mainTunnelRight};
        final Pose[] drillRigGoal = {drillPoint2};

        var autonomousRobot1 = new AutonomousVehicle(2, Color.YELLOW, 5, 2, 2, 1);
        var autonomousRobot2 = new AutonomousVehicle(2, Color.YELLOW, 5, 2, 2, 1);
        var autonomousRobot3 = new AutonomousVehicle(2, Color.YELLOW, 5, 2, 2, 1);
        var autonomousRobot4 = new AutonomousVehicle(2, Color.YELLOW, 5, 2, 2, 1);
        var autonomousRobot5 = new AutonomousVehicle(1, Color.RED, 0.05, 0.02, 3.5, 3.5);
        var drillRig = new LookAheadVehicle(2, drillLookAheadDistance, Color.GREEN, 5, 2, 2, 1);

        autonomousRobot1.getPlan(drawPoint1, autonomousRobotGoal1, YAML_FILE, true);
        autonomousRobot2.getPlan(drawPoint2, autonomousRobotGoal2, YAML_FILE, true);
        autonomousRobot3.getPlan(drawPoint3, autonomousRobotGoal3, YAML_FILE, true);
        autonomousRobot4.getPlan(drawPoint4, autonomousRobotGoal4, YAML_FILE, true);
        autonomousRobot5.getPlan(mainTunnelLeft, autonomousRobotGoal5, YAML_FILE, true);
        drillRig.getPlan(drillPoint, drillRigGoal, YAML_FILE, false);

        // Instantiate a trajectory envelope coordinator.
        var tec = new TrajectoryEnvelopeCoordinatorSimulation(2000, 1000, 5, 2);
        tec.setupSolver(0, 100000000);
        tec.startInference();

        tec.setForwardModel(autonomousRobot1.getID(), new ConstantAccelerationForwardModel(autonomousRobot1.getMaxAcceleration(),
                autonomousRobot1.getMaxVelocity(), tec.getTemporalResolution(), tec.getControlPeriod(),
                tec.getRobotTrackingPeriodInMillis(autonomousRobot1.getID())));
        tec.setForwardModel(autonomousRobot2.getID(), new ConstantAccelerationForwardModel(autonomousRobot2.getMaxAcceleration(),
                autonomousRobot2.getMaxVelocity(), tec.getTemporalResolution(), tec.getControlPeriod(),
                tec.getRobotTrackingPeriodInMillis(autonomousRobot2.getID())));
        tec.setForwardModel(autonomousRobot3.getID(), new ConstantAccelerationForwardModel(autonomousRobot3.getMaxAcceleration(),
                autonomousRobot3.getMaxVelocity(), tec.getTemporalResolution(), tec.getControlPeriod(),
                tec.getRobotTrackingPeriodInMillis(autonomousRobot3.getID())));
        tec.setForwardModel(autonomousRobot4.getID(), new ConstantAccelerationForwardModel(autonomousRobot4.getMaxAcceleration(),
                autonomousRobot4.getMaxVelocity(), tec.getTemporalResolution(), tec.getControlPeriod(),
                tec.getRobotTrackingPeriodInMillis(autonomousRobot4.getID())));
        tec.setForwardModel(autonomousRobot5.getID(), new ConstantAccelerationForwardModel(autonomousRobot5.getMaxAcceleration(),
                autonomousRobot5.getMaxVelocity(), tec.getTemporalResolution(), tec.getControlPeriod(),
                tec.getRobotTrackingPeriodInMillis(autonomousRobot5.getID())));

        Coordinate[] fp5 = new Coordinate[]{
                new Coordinate(-4.5, 4.5),
                new Coordinate(4.5, 4.5),
                new Coordinate(4.5, -4.5),
                new Coordinate(-4.5, -4.5)
        };
        tec.setDefaultFootprint(drillRig.getFootprint());
        tec.setFootprint(autonomousRobot5.getID(), fp5);
        tec.placeRobot(autonomousRobot1.getID(), drawPoint1);
        tec.placeRobot(autonomousRobot2.getID(), drawPoint2);
        tec.placeRobot(autonomousRobot3.getID(), drawPoint3);
        tec.placeRobot(autonomousRobot4.getID(), drawPoint4);
        tec.placeRobot(autonomousRobot5.getID(), mainTunnelLeft);
        tec.placeRobot(drillRig.getID(), drillPoint);

        // Set Heuristics
        var heuristic = new Heuristics();
        tec.addComparator(heuristic.mostDistanceToTravel());
        String heuristicName = heuristic.getHeuristicName();

        // Set Local Re-ordering and Local Re-Planning to break Deadlocks
        tec.setBreakDeadlocks(true, false, false);

        // Set up a simple GUI (null means an empty map, otherwise provide yaml file)
        var viz = new BrowserVisualization();
        viz.setMap(YAML_FILE);
        viz.setFontScale(0.5);
        viz.setInitialTransform(9.6, 30.2, -0.73);
        tec.setVisualization(viz);

        var m1 = new Mission(autonomousRobot1.getID(), autonomousRobot1.getPath());
        var m2 = new Mission(autonomousRobot2.getID(), autonomousRobot2.getPath());
        var m3 = new Mission(autonomousRobot3.getID(), autonomousRobot3.getPath());
        var m4 = new Mission(autonomousRobot4.getID(), autonomousRobot4.getPath());
        var m5 = new Mission(autonomousRobot5.getID(), autonomousRobot5.getPath());
        var m6 = new Mission(drillRig.getID(), drillRig.getLimitedPath(drillRig.getID(), drillLookAheadDistance, tec));
//        m4.setStoppingPoint(orePass3, 10000); //FIXME I think it does not work.

//        var randomRobotCaller = new RandomRobotCaller(numOfCallsForLookAheadRobot, terminationInMinutes);
//        randomRobotCaller.scheduleRandomCalls(m4);

        Missions.enqueueMission(m1);
        Missions.enqueueMission(m2);
        Missions.enqueueMission(m3);
        Missions.enqueueMission(m4);
        Missions.enqueueMission(m5);
        Missions.enqueueMission(m6);
        Missions.setMap(YAML_FILE);

//        Missions.startMissionDispatchers(tec, drillLookAheadDistance, writeRobotReports,
//                intervalInSeconds, terminationInMinutes, heuristicName, resultsDirectory);

//        Thread.sleep(10000);
//        tec.placeRobot(drillRig.getID(), mainTunnelLeft);
//        drillRig.getPlan(mainTunnelLeft, drillRigGoal, YAML_FILE, false);
//        PoseSteering[] drillInitialPath = drillRig.getLimitedPath(drillRig.getID(), drillRig.getLookAheadDistance(), tec);
//        var m6 = new Mission(drillRig.getID(), drillInitialPath);
//        tec.addMissions(m6);
    }
}

