package se.oru.coordination.coordination_oru.scenarios;

import org.metacsp.multi.spatioTemporal.paths.Pose;
import se.oru.coordination.coordination_oru.Mission;
import se.oru.coordination.coordination_oru.code.AutonomousVehicle;
import se.oru.coordination.coordination_oru.simulation2D.TrajectoryEnvelopeCoordinatorSimulation;
import se.oru.coordination.coordination_oru.util.BrowserVisualization;
import se.oru.coordination.coordination_oru.util.Missions;

public class TwoAutonomousRobots {
    public static void main(String[] args) {

        final String YAML_FILE = "maps/mine-map-test.yaml";

        final Pose mainTunnelLeft = new Pose(4.25, 15.35, -Math.PI + Math.PI);
        final Pose mainTunnelRight = new Pose(80.05, 24.75, Math.PI);
        final Pose drawPoint21 = new Pose(52.95, 87.75, -Math.PI / 2);
        final Pose orePass = new Pose(54.35, 11.25, -Math.PI / 2);

        final Pose[] autonomousRobotGoal1 = {orePass};
        final Pose[] autonomousRobotGoal2 = {mainTunnelRight};

        var autonomousRobot1 = new AutonomousVehicle();
        var autonomousRobot2 = new AutonomousVehicle();
        autonomousRobot1.getPlan(drawPoint21, autonomousRobotGoal1, YAML_FILE, true);
        autonomousRobot2.getPlan(mainTunnelLeft, autonomousRobotGoal1, YAML_FILE, true);

        // Instantiate a trajectory envelope coordinator.
        var tec = new TrajectoryEnvelopeCoordinatorSimulation(2000, 1000, 5, 2);

        // Sets up MetaCSP solver
        tec.setupSolver(0, 100000000);

        // Start the thread that checks and enforces dependencies at every clock tick
        tec.startInference();

//        var heuristics = new Heuristics().closest();
//        tec.addComparator(heuristics);
//        tec.setDefaultFootprint(autonomousRobot1.getFootPrint());
        tec.placeRobot(autonomousRobot1.getID(), drawPoint21);
        tec.placeRobot(autonomousRobot2.getID(), mainTunnelLeft);
//        tec.addComparator(new Heuristics().closest());
//        tec.setUseInternalCriticalPoints(false);
//        tec.setYieldIfParking(true);
//        tec.setBreakDeadlocks(true, false, false);

        // Set up a simple GUI (null means an empty map, otherwise provide yaml file)
        var viz = new BrowserVisualization();
        viz.setMap(YAML_FILE);
        viz.setFontScale(4);
        viz.setInitialTransform(9, 45, -3.5);
        tec.setVisualization(viz);

        var m1 = new Mission(autonomousRobot1.getID(), autonomousRobot1.getPath());
        var m2 = new Mission(autonomousRobot2.getID(), autonomousRobot2.getPath());

        Missions.enqueueMission(m1);
        Missions.enqueueMission(m2);
        Missions.setMap(YAML_FILE);

        Missions.startMissionDispatchers(tec);
    }
}