package se.oru.coordination.coordination_oru.tests;

import java.awt.Color;

import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.PoseSteering;

import se.oru.coordination.coordination_oru.Mission;
import se.oru.coordination.coordination_oru.code.AutonomousVehicle;
import se.oru.coordination.coordination_oru.code.Heuristics;
import se.oru.coordination.coordination_oru.code.HumanDrivenVehicle;
import se.oru.coordination.coordination_oru.code.VehiclesHashMap;
import se.oru.coordination.coordination_oru.simulation2D.TrajectoryEnvelopeCoordinatorSimulation;
import se.oru.coordination.coordination_oru.util.BrowserVisualization;
import se.oru.coordination.coordination_oru.util.Missions;
import se.oru.coordination.coordination_oru.util.NoPathFound;

public class TestClick {
    public static TrajectoryEnvelopeCoordinatorSimulation tec = null;

    public static void main(String[] args) throws NoPathFound {

        final int loopMinutes = 5;
        final long loopTime = System.currentTimeMillis() + (loopMinutes * 60 * 1000);
        final Pose mainTunnelLeft = new Pose(4.25,15.35, -Math.PI);
        final Pose mainTunnelRight = new Pose(80.05,24.75, Math.PI);
        final Pose drawPoint19 = new Pose(39.05,85.45,-Math.PI/2);
        final String YAML_FILE = "maps/mine-map-test.yaml"; // TODO: create OccupancyMap now once (for efficiency)
        final int maxVelocity = 15;

        AutonomousVehicle hum1 = new HumanDrivenVehicle(1, 0, Color.GREEN, maxVelocity, 2, YAML_FILE, 0.5, 0.5);
        AutonomousVehicle aut2 = new AutonomousVehicle(2, 0, Color.RED, maxVelocity, 2, YAML_FILE, 0.5, 0.5);
        System.out.println(VehiclesHashMap.getInstance().getList());

        // Instantiate a trajectory envelope coordinator.
        tec = new TrajectoryEnvelopeCoordinatorSimulation(2000, 1000, maxVelocity, 2);
        // Need to set up infrastructure that maintains the representation
        tec.setupSolver(0, 100000000);
        // Start the thread that checks and enforces dependencies at every clock tick
        tec.startInference();

        tec.setDefaultFootprint(hum1.getFootPrint());
        tec.placeRobot(hum1.getID(), mainTunnelLeft);
        tec.placeRobot(aut2.getID(), mainTunnelRight);
        tec.addComparator(new Heuristics().closest());
        tec.setUseInternalCriticalPoints(false);
        tec.setYieldIfParking(true);
        tec.setBreakDeadlocks(true, false, false);
        tec.setMotionPlanner(1, hum1.makePlanner()); // needed for re-planning

        // Set up a simple GUI (null means empty map, otherwise provide yaml file)
        var viz = new BrowserVisualization();
        viz.setMap(YAML_FILE);
        viz.setInitialTransform(7.0, 83.0, 15.5);
        tec.setVisualization(viz);

        Missions.setMap(YAML_FILE);
        Missions.startMissionDispatchers(tec, true, loopTime);
        Missions.loopMissions.put(1, false);

        PoseSteering[] path2 = aut2.getPath(mainTunnelRight, drawPoint19, true);
        var m2 = new Mission(aut2.getID(), path2);
        Missions.enqueueMission(m2);
    }
}
