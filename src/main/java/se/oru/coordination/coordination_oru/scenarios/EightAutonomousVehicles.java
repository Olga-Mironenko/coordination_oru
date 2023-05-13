package se.oru.coordination.coordination_oru.scenarios;

import org.metacsp.multi.spatioTemporal.paths.Pose;
import se.oru.coordination.coordination_oru.utility.Mission;
import se.oru.coordination.coordination_oru.vehicles.AutonomousVehicle;
import se.oru.coordination.coordination_oru.utility.Heuristics;
import se.oru.coordination.coordination_oru.simulator.TrajectoryEnvelopeCoordinatorSimulation;
import se.oru.coordination.coordination_oru.simulator.BrowserVisualization;
import se.oru.coordination.coordination_oru.utility.Missions;

public class EightAutonomousVehicles {
    public static void main(String[] args) {

        final int simulationTimeMinutes = 10;
        final long simulationTime = System.currentTimeMillis() + (simulationTimeMinutes * 60 * 1000);
        final String YAML_FILE = "maps/mine-map-test.yaml";

        final Pose mainTunnelLeft = new Pose(4.25, 15.35, -Math.PI);
        final Pose mainTunnelRight = new Pose(80.05, 24.75, Math.PI);
        final Pose drawPoint17 = new Pose(24.15, 85.55, -Math.PI / 2);
        final Pose drawPoint19 = new Pose(38.75, 86.35, -Math.PI / 2);
        final Pose drawPoint20 = new Pose(45.85, 86.15, -Math.PI / 2);
        final Pose drawPoint21 = new Pose(52.95, 87.75, -Math.PI / 2);
        final Pose drawPoint22 = new Pose(60.35, 87.85, -Math.PI / 2);
        final Pose drawPoint23 = new Pose(67.75, 86.95, -Math.PI / 2);
        final Pose drawPoint24 = new Pose(74.85, 84.45, -Math.PI / 2);
        final Pose orePass = new Pose(54.35, 11.25, -Math.PI / 2);

        final Pose[] autonomousVehicleGoal = {orePass};
        final Pose[] autonomousVehicle8Goal = {mainTunnelRight};

        //TODO I think controller kills everything up
        var autonomousVehicle1 = new AutonomousVehicle();
        var autonomousVehicle2 = new AutonomousVehicle();
        var autonomousVehicle3 = new AutonomousVehicle();
        var autonomousVehicle4 = new AutonomousVehicle();
        var autonomousVehicle5 = new AutonomousVehicle();
        var autonomousVehicle6 = new AutonomousVehicle();
        var autonomousVehicle7 = new AutonomousVehicle();
        var autonomousVehicle8 = new AutonomousVehicle();
        autonomousVehicle1.getPlan(drawPoint17, autonomousVehicleGoal, YAML_FILE, true);
        autonomousVehicle2.getPlan(drawPoint19, autonomousVehicleGoal, YAML_FILE, true);
        autonomousVehicle3.getPlan(drawPoint20, autonomousVehicleGoal, YAML_FILE, true);
        autonomousVehicle4.getPlan(drawPoint21, autonomousVehicleGoal, YAML_FILE, true);
        autonomousVehicle5.getPlan(drawPoint22, autonomousVehicleGoal, YAML_FILE, true);
        autonomousVehicle6.getPlan(drawPoint23, autonomousVehicleGoal, YAML_FILE, true);
        autonomousVehicle7.getPlan(drawPoint24, autonomousVehicleGoal, YAML_FILE, true);
        autonomousVehicle8.getPlan(mainTunnelLeft, autonomousVehicle8Goal, YAML_FILE, true);

        // Instantiate a trajectory envelope coordinator.
        final var tec = new TrajectoryEnvelopeCoordinatorSimulation(2000, 1000, 5, 2);
        // Need to set up infrastructure that maintains the representation
        tec.setupSolver(0, 100000000);
        // Start the thread that checks and enforces dependencies at every clock tick
        tec.startInference();

        tec.setDefaultFootprint(autonomousVehicle1.getFootPrint());
        tec.placeRobot(autonomousVehicle1.getID(), drawPoint17);
        tec.placeRobot(autonomousVehicle2.getID(), drawPoint19);
        tec.placeRobot(autonomousVehicle3.getID(), drawPoint20);
        tec.placeRobot(autonomousVehicle4.getID(), drawPoint21);
        tec.placeRobot(autonomousVehicle5.getID(), drawPoint22);
        tec.placeRobot(autonomousVehicle6.getID(), drawPoint23);
        tec.placeRobot(autonomousVehicle7.getID(), drawPoint24);
        tec.placeRobot(autonomousVehicle8.getID(), mainTunnelLeft);
        tec.addComparator(new Heuristics().closest());
        tec.setUseInternalCriticalPoints(false);
        tec.setYieldIfParking(true);
        tec.setBreakDeadlocks(true, false, false);

        // Set up a simple GUI (null means empty map, otherwise provide yaml file)
        var viz = new BrowserVisualization();
        viz.setMap(YAML_FILE);
//        viz.setFontScale(4);
        viz.setInitialTransform(11, 45, -3.5);
        tec.setVisualization(viz);

        var m1 = new Mission(autonomousVehicle1.getID(), autonomousVehicle1.getPath());
        var m2 = new Mission(autonomousVehicle2.getID(), autonomousVehicle2.getPath());
        var m3 = new Mission(autonomousVehicle3.getID(), autonomousVehicle3.getPath());
        var m4 = new Mission(autonomousVehicle4.getID(), autonomousVehicle4.getPath());
        var m5 = new Mission(autonomousVehicle5.getID(), autonomousVehicle5.getPath());
        var m6 = new Mission(autonomousVehicle6.getID(), autonomousVehicle6.getPath());
        var m7 = new Mission(autonomousVehicle7.getID(), autonomousVehicle7.getPath());
        var m8 = new Mission(autonomousVehicle8.getID(), autonomousVehicle8.getPath());

        Missions.enqueueMission(m1);
        Missions.enqueueMission(m2);
        Missions.enqueueMission(m3);
        Missions.enqueueMission(m4);
        Missions.enqueueMission(m5);
        Missions.enqueueMission(m6);
        Missions.enqueueMission(m7);
        Missions.enqueueMission(m8);
        Missions.setMap(YAML_FILE);
        Missions.startMissionDispatchers(tec, simulationTime);

    }
}
