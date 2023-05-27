package se.oru.coordination.coordination_oru.scenarios;

import org.metacsp.multi.spatioTemporal.paths.Pose;
import se.oru.coordination.coordination_oru.coordinator.ConstantAccelerationForwardModel;
import se.oru.coordination.coordination_oru.robots.AutonomousRobot;
import se.oru.coordination.coordination_oru.robots.LookAheadRobot;
import se.oru.coordination.coordination_oru.simulator.BrowserVisualization;
import se.oru.coordination.coordination_oru.simulator.TrajectoryEnvelopeCoordinatorSimulation;
import se.oru.coordination.coordination_oru.utility.*;

import java.awt.*;
import java.io.FileNotFoundException;

public class PaperScenario_3A {
    public static void main(String[] args) throws FileNotFoundException {

        String absolutePath = System.getProperty("user.dir");
        String resultsDirectory = absolutePath + "/src/main/java/se/oru/coordination/coordination_oru/results/lookAheadPaper_2023";
        final String YAML_FILE = "maps/mine-map-paper-2023.yaml";
        double lookAheadDistance = 30;
        int intervalInSeconds = 1;
        int terminationInMinutes = 30;
        int numOfCallsForLookAheadRobot = 10;
        boolean visualization = true;
        boolean writeRobotReports = false;

        final Pose mainTunnelLeft = new Pose(14.25, 22.15, Math.PI);
        final Pose mainTunnelRight = new Pose(114.15, 40.05, Math.PI);
        final Pose entrance = new Pose(115.35, 3.75, Math.PI);
        final Pose drawPoint12 = new Pose(88.35, 101.05, -Math.PI / 2);
        final Pose drawPoint13 = new Pose(95.75, 100.85, Math.PI);
        final Pose drawPoint14 = new Pose(102.45, 98.05, Math.PI);
        final Pose drawPoint27 = new Pose(17.95, 54.35, Math.PI);
        final Pose drawPoint28 = new Pose(25.05, 58.35, -Math.PI / 2);
        final Pose drawPoint29 = new Pose(31.95, 58.75, Math.PI);
        final Pose drawPoint29A = new Pose(39.35, 54.15, Math.PI);
        final Pose drawPoint30 = new Pose(46.25, 49.85, -Math.PI / 2);
        final Pose drawPoint31 = new Pose(53.25, 49.25, -Math.PI / 2);
        final Pose drawPoint32 = new Pose(60.35, 53.05, -Math.PI / 2);
        final Pose drawPoint32A = new Pose(67.55, 55.45, -Math.PI / 2);
        final Pose drawPoint33 = new Pose(74.25, 73.45, -Math.PI / 2);
        final Pose drawPoint34 = new Pose(81.35, 79.45, -Math.PI / 2);
        final Pose drawPoint35 = new Pose(88.45, 81.95, -Math.PI / 2);
        final Pose orePass1 = new Pose(28.45, 15.05, -Math.PI / 2);
        final Pose orePass2 = new Pose(76.35, 31.05, -Math.PI / 2.7);
        final Pose orePass3 = new Pose(92.65, 33.15, -Math.PI / 2);

        final Pose[] autonomousRobotGoal1 = {orePass1};
        final Pose[] autonomousRobotGoal2 = {orePass2};
        final Pose[] autonomousRobotGoal3 = {orePass3};
        final Pose[] limitedLookAheadRobotGoal = {mainTunnelLeft};

        var autonomousRobot1 = new AutonomousRobot();
        var autonomousRobot2 = new AutonomousRobot();
        var autonomousRobot3 = new AutonomousRobot();
        var lookAheadRobot = new LookAheadRobot(1, lookAheadDistance, Color.GREEN, 5, 2, 0.9, 0.5);

        autonomousRobot1.getPlan(drawPoint28, autonomousRobotGoal1, YAML_FILE, true);
        autonomousRobot2.getPlan(drawPoint32A, autonomousRobotGoal2, YAML_FILE, true);
        autonomousRobot3.getPlan(drawPoint35, autonomousRobotGoal3, YAML_FILE, true);
        lookAheadRobot.getPlan(entrance, limitedLookAheadRobotGoal, YAML_FILE, true);

        // Instantiate a trajectory envelope coordinator.
        var tec = new TrajectoryEnvelopeCoordinatorSimulation(2000, 1000, 5, 2.5);
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
        tec.setForwardModel(lookAheadRobot.getID(), new ConstantAccelerationForwardModel(lookAheadRobot.getMaxAcceleration(),
                lookAheadRobot.getMaxVelocity(), tec.getTemporalResolution(), tec.getControlPeriod(),
                tec.getRobotTrackingPeriodInMillis(lookAheadRobot.getID())));

        tec.setDefaultFootprint(lookAheadRobot.getFootPrint());
        tec.placeRobot(autonomousRobot1.getID(), drawPoint28);
        tec.placeRobot(autonomousRobot2.getID(), drawPoint32A);
        tec.placeRobot(autonomousRobot3.getID(), drawPoint35);
        tec.placeRobot(lookAheadRobot.getID(), entrance);

        // Set Heuristics
        var heuristic = new Heuristics();
        tec.addComparator(heuristic.closest());
        String heuristicName = heuristic.getHeuristicName();

        // Set Local Re-ordering and Local Re-Planning to break Deadlocks
        tec.setBreakDeadlocks(false, true, true);

        // Set up a simple GUI (null means an empty map, otherwise provide yaml file)
        if (visualization) {
            var viz = new BrowserVisualization();
            viz.setMap(YAML_FILE);
            viz.setFontScale(2.5);
            viz.setInitialTransform(9.6, 30.2, -0.73);
            tec.setVisualization(viz);
        }

        var m1 = new Mission(autonomousRobot1.getID(), autonomousRobot1.getPath());
        var m2 = new Mission(autonomousRobot2.getID(), autonomousRobot2.getPath());
        var m3 = new Mission(autonomousRobot3.getID(), autonomousRobot3.getPath());
        var m4 = new Mission(lookAheadRobot.getID(), lookAheadRobot.getLimitedPath(lookAheadRobot.getID(), lookAheadDistance, tec));
//        m4.setStoppingPoint(orePass3, 10000); //FIXME I think it does not work.

        var randomRobotCaller = new RandomRobotCaller(numOfCallsForLookAheadRobot, terminationInMinutes);
        randomRobotCaller.scheduleRandomCalls(m4);

        Missions.enqueueMission(m1);
        Missions.enqueueMission(m2);
        Missions.enqueueMission(m3);
        Missions.setMap(YAML_FILE);

        Missions.startMissionDispatchers(tec, lookAheadDistance, writeRobotReports,
                intervalInSeconds, terminationInMinutes, heuristicName, resultsDirectory);
    }
}

