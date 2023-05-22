package se.oru.coordination.coordination_oru.scenarios;

import org.metacsp.multi.spatioTemporal.paths.Pose;
import se.oru.coordination.coordination_oru.robots.AutonomousRobot;
import se.oru.coordination.coordination_oru.robots.LookAheadRobot;
import se.oru.coordination.coordination_oru.simulator.BrowserVisualization;
import se.oru.coordination.coordination_oru.simulator.TrajectoryEnvelopeCoordinatorSimulation;
import se.oru.coordination.coordination_oru.utility.Heuristics;
import se.oru.coordination.coordination_oru.utility.Mission;
import se.oru.coordination.coordination_oru.utility.Missions;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.Random;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

public class OneAutonomousOneLookAheadRobots {
    public static void handleException(Exception e) {
        String errorMessage = e.getMessage();
        String absolutePath = System.getProperty("user.dir");
        String resultsDir = absolutePath + "/src/main/java/se/oru/coordination/coordination_oru/results";
        Path directoryPath = Paths.get(resultsDir);
        String fileName = directoryPath+ "/" +"Error.txt";
        Path filePath = Paths.get(fileName);

        try {
            Files.writeString(filePath, errorMessage);
            System.out.println("The error message was written to " + fileName);
        } catch (IOException ioException) {
            System.out.println("An error occurred while writing the error message to " + fileName);
            ioException.printStackTrace();
        }
    }

    public static void main(String[] args) {

        final String YAML_FILE = "maps/mine-map-test.yaml";
        double lookAheadDistance = 25;
        int intervalInSeconds = 1;
        int terminationInMinutes = 30;

        final Pose mainTunnelLeft = new Pose(4.25, 15.35, -Math.PI + Math.PI);
        final Pose mainTunnelRight = new Pose(80.05, 24.75, Math.PI);
        final Pose drawPoint21 = new Pose(52.95, 87.75, -Math.PI / 2);
        final Pose orePass = new Pose(54.35, 11.25, -Math.PI / 2);

        final Pose[] autonomousRobotGoal = {orePass};
        final Pose[] limitedLookAheadRobotGoal = {mainTunnelRight};

        var autonomousRobot = new AutonomousRobot();
        var lookAheadRobot = new LookAheadRobot(lookAheadDistance);
        autonomousRobot.getPlan(drawPoint21, autonomousRobotGoal, YAML_FILE, true);
        lookAheadRobot.getPlan(mainTunnelLeft, limitedLookAheadRobotGoal, YAML_FILE, true);

        // Instantiate a trajectory envelope coordinator.
        var tec = new TrajectoryEnvelopeCoordinatorSimulation(2000, 1000, 5, 2);
        // Need to set up infrastructure that maintains the representation
        tec.setupSolver(0, 100000000);
        // Start the thread that checks and enforces dependencies at every clock tick
        tec.startInference();

        var heuristics = new Heuristics().closest();
        tec.addComparator(heuristics);
        tec.setDefaultFootprint(autonomousRobot.getFootPrint());
        tec.placeRobot(autonomousRobot.getID(), drawPoint21);
        tec.placeRobot(lookAheadRobot.getID(), mainTunnelLeft);
        tec.addComparator(new Heuristics().closest());
        tec.setUseInternalCriticalPoints(false);
        tec.setYieldIfParking(true);
        tec.setBreakDeadlocks(true, false, false);

        // Set Heuristics
        var heuristic = new Heuristics();
        tec.addComparator(heuristic.closest());
        String heuristicName = heuristic.getHeuristicName();

        // Set up a simple GUI (null means an empty map, otherwise provide yaml file)
        var viz = new BrowserVisualization();
        viz.setMap(YAML_FILE);
        viz.setFontScale(4);
        viz.setInitialTransform(11, 45, -3.5);
        tec.setVisualization(viz);

        var lookAheadRobotInitialPlan = lookAheadRobot.getLimitedPath(lookAheadRobot.getID(), lookAheadDistance, tec);
        var m1 = new Mission(autonomousRobot.getID(), autonomousRobot.getPath());
        var m2 = new Mission(lookAheadRobot.getID(), lookAheadRobotInitialPlan);

        Missions.enqueueMission(m1);
        Missions.enqueueMission(m2);
        Missions.setMap(YAML_FILE);

        try {
            Missions.startMissionDispatchers(tec);
        } catch (Exception e) {
            handleException(e);
        }
    }
}