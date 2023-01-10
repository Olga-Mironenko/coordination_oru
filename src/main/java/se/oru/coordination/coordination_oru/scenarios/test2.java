package se.oru.coordination.coordination_oru.scenarios;

import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.PoseSteering;
import se.oru.coordination.coordination_oru.Mission;
import se.oru.coordination.coordination_oru.code.AutonomousVehicle;
import se.oru.coordination.coordination_oru.code.Heuristics;
import se.oru.coordination.coordination_oru.simulation2D.TrajectoryEnvelopeCoordinatorSimulation;
import se.oru.coordination.coordination_oru.util.BrowserVisualization;
import se.oru.coordination.coordination_oru.util.Missions;

import java.awt.*;
import java.util.Arrays;

public class test2 {

    public static void main(String[] args) {

        Pose mainTunnelLeft = new Pose(4.25,15.35, -Math.PI);
        Pose mainTunnelRight = new Pose(80.05,24.75, Math.PI);
        String YAML_FILE = "maps/mine-map-test.yaml";

        AutonomousVehicle aut = new AutonomousVehicle();
        PoseSteering[] path = aut.getPath(mainTunnelLeft, mainTunnelRight, YAML_FILE, true);
        System.out.println(Arrays.toString(path));

        // Instantiate a trajectory envelope coordinator.
        final var tec = new TrajectoryEnvelopeCoordinatorSimulation(2000, 1000, 5, 2);
        // Need to set up infrastructure that maintains the representation
        tec.setupSolver(0, 100000000);
        // Start the thread that checks and enforces dependencies at every clock tick
        tec.startInference();

        tec.getRobotReport(aut.getID());
        tec.setDefaultFootprint(aut.getFootPrint());
        tec.placeRobot(aut.getID(), mainTunnelLeft);
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

        Missions.enqueueMission(m1);

        Missions.setMap(YAML_FILE);
        Missions.startMissionDispatchers(tec, true, 1);

        // FIXME integrate color for robot via visualization
//        String color = "#" + String.format("%06X", 0xFFFFFF & Color.CYAN.getRGB());

    }
}
