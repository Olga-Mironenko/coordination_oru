package se.oru.coordination.coordination_oru.scenarios;

import org.metacsp.multi.spatioTemporal.paths.Pose;
import se.oru.coordination.coordination_oru.utility.Mission;
import se.oru.coordination.coordination_oru.vehicles.AutonomousVehicle;
import se.oru.coordination.coordination_oru.utility.Heuristics;
import se.oru.coordination.coordination_oru.vehicles.LookAheadVehicle;
import se.oru.coordination.coordination_oru.simulator.TrajectoryEnvelopeCoordinatorSimulation;
import se.oru.coordination.coordination_oru.simulator.BrowserVisualization;
import se.oru.coordination.coordination_oru.utility.Missions;

public class OneAutonomousOneLookAheadVehicles {
    public static void main(String[] args) {

        final String YAML_FILE = "maps/mine-map-test.yaml";
        final int simulationTimeMinutes = 2;
        double predictableDistance = 25.0;
        long simulationTime = System.currentTimeMillis() + (simulationTimeMinutes * 60 * 1000);
        final Pose mainTunnelLeft = new Pose(4.25, 15.35, -Math.PI);
        final Pose mainTunnelRight = new Pose(80.05, 24.75, Math.PI);
        final Pose drawPoint21 = new Pose(52.95, 87.75, -Math.PI / 2);
        final Pose orePass = new Pose(54.35, 11.25, -Math.PI / 2);

        final Pose[] autonomousVehicleGoal = {orePass};
        final Pose[] limitedLookAheadVehicleGoal = {mainTunnelRight};

        var autonomousVehicle = new AutonomousVehicle();
        var lookAheadVehicle = new LookAheadVehicle(predictableDistance);
        autonomousVehicle.getPlan(drawPoint21, autonomousVehicleGoal, YAML_FILE, true);
        lookAheadVehicle.getPlan(mainTunnelLeft, limitedLookAheadVehicleGoal, YAML_FILE, true);

        // Instantiate a trajectory envelope coordinator.
        var tec = new TrajectoryEnvelopeCoordinatorSimulation(2000, 1000, 5, 2);
        // Need to set up infrastructure that maintains the representation
        tec.setupSolver(0, 100000000);
        // Start the thread that checks and enforces dependencies at every clock tick
        tec.startInference();

        tec.setDefaultFootprint(autonomousVehicle.getFootPrint());
        tec.placeRobot(autonomousVehicle.getID(), drawPoint21);
        tec.placeRobot(lookAheadVehicle.getID(), mainTunnelLeft);
        tec.addComparator(new Heuristics().closest());
        tec.setUseInternalCriticalPoints(false);
        tec.setYieldIfParking(true);
        tec.setBreakDeadlocks(true, false, false);

        // Set up a simple GUI (null means empty map, otherwise provide yaml file)
        var viz = new BrowserVisualization();
        viz.setMap(YAML_FILE);
        viz.setFontScale(4);
        viz.setInitialTransform(11, 45, -3.5);
        tec.setVisualization(viz);

        var lookAheadVehicleInitialPlan = lookAheadVehicle.getLimitedPath(lookAheadVehicle.getID(), predictableDistance, tec);
        var m1 = new Mission(autonomousVehicle.getID(), autonomousVehicle.getPath());
        var m2 = new Mission(lookAheadVehicle.getID(), lookAheadVehicleInitialPlan);

        Missions.enqueueMission(m1);
        Missions.enqueueMission(m2);
        Missions.setMap(YAML_FILE);
        Missions.startMissionDispatchers(tec, simulationTime);
    }
}
