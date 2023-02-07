package se.oru.coordination.coordination_oru.scenarios;

import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.PoseSteering;
import se.oru.coordination.coordination_oru.Mission;
import se.oru.coordination.coordination_oru.code.AutonomousVehicle;
import se.oru.coordination.coordination_oru.code.Heuristics;
import se.oru.coordination.coordination_oru.code.VehiclesHashMap;
import se.oru.coordination.coordination_oru.simulation2D.TrajectoryEnvelopeCoordinatorSimulation;
import se.oru.coordination.coordination_oru.util.BrowserVisualization;
import se.oru.coordination.coordination_oru.util.Missions;

import java.awt.*;

public class TestClick {
	public static TrajectoryEnvelopeCoordinatorSimulation tec = null;
	
    public static void main(String[] args) {

        final int loopMinutes = 1;
        final long loopTime = System.currentTimeMillis() + (loopMinutes * 60 * 1000);
        final Pose mainTunnelLeft = new Pose(4.25,15.35, -Math.PI);
        final Pose mainTunnelRight = new Pose(80.05,24.75, Math.PI);
        final Pose drawPoint19 = new Pose(39.05,85.45,-Math.PI/2);
        final String YAML_FILE = "maps/mine-map-test.yaml";

        AutonomousVehicle aut = new AutonomousVehicle(1, 0, Color.GREEN, 5, 2, YAML_FILE, 0.5, 0.5);
        AutonomousVehicle aut2 = new AutonomousVehicle(2, 0, Color.RED, 5, 2, YAML_FILE, 0.5, 0.5);
        System.out.println(VehiclesHashMap.getInstance().getList());
        PoseSteering[] path = aut.getPath(mainTunnelLeft, mainTunnelRight, YAML_FILE, true);
        PoseSteering[] path2 = aut2.getPath(mainTunnelRight, drawPoint19, YAML_FILE, true);

        // Instantiate a trajectory envelope coordinator.
        tec = new TrajectoryEnvelopeCoordinatorSimulation(2000, 1000, 5, 2);
        // Need to set up infrastructure that maintains the representation
        tec.setupSolver(0, 100000000);
        // Start the thread that checks and enforces dependencies at every clock tick
        tec.startInference();

        tec.setDefaultFootprint(aut.getFootPrint());
        tec.placeRobot(aut.getID(), mainTunnelLeft);
        tec.placeRobot(aut2.getID(), mainTunnelRight);
        tec.addComparator(new Heuristics().closest());
        tec.setUseInternalCriticalPoints(false);
        tec.setYieldIfParking(true);
        tec.setBreakDeadlocks(true, false, false);
        
        // Set up a simple GUI (null means empty map, otherwise provide yaml file)
        var viz = new BrowserVisualization();
        viz.setMap(YAML_FILE);
        viz.setInitialTransform(7.0, 83.0, 15.5);
        tec.setVisualization(viz);

        var m1 = new Mission(aut.getID(), path);
        var m2 = new Mission(aut2.getID(), path2);

        Missions.enqueueMission(m1);
        Missions.enqueueMission(m2);
        Missions.setMap(YAML_FILE);
        Missions.startMissionDispatchers(tec, true, loopTime);

    }
}
