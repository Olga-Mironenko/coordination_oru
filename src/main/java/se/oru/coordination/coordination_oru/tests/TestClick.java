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
import se.oru.coordination.coordination_oru.util.Printer;
import se.oru.coordination.coordination_oru.util.gates.GatedThread;

public class TestClick {
    public static TrajectoryEnvelopeCoordinatorSimulation tec = null;

    public static void main(String[] args) {
        Printer.resetTime();
        Printer.print("started");

        GatedThread.enable();

        new GatedThread("runDemo") {
            @Override
            public void runCore() {
                try {
                    runDemo();
                } catch (NoPathFound e) {
                    throw new RuntimeException(e);
                }
            }
        }.start();

        GatedThread.runGatekeeper();
    }

    protected static void runDemo() throws NoPathFound {

        final int loopMinutes = 5;
        final long loopTime = System.currentTimeMillis() + (loopMinutes * 60 * 1000);

        final Pose mainTunnelLeft = new Pose(4.25,15.35, -Math.PI);
        final Pose mainTunnelRight = new Pose(80.05,24.75, Math.PI);
        final Pose orePass = new Pose(54.11,11.34,-Math.PI/2);
        final Pose drawPoint15 = new Pose(9.9,84.5,-Math.PI/2);
        final Pose drawPoint16 = new Pose(17.1,84.6,-Math.PI/2);
        final Pose drawPoint17 = new Pose(24.3,85.45,-Math.PI/2);
        final Pose drawPoint18 = new Pose(31.6,84.6,-Math.PI/2);
        final Pose drawPoint19 = new Pose(39.05,85.45,-Math.PI/2);
        final Pose drawPoint19_bottom = new Pose(38.8,28.6,-Math.PI/2);
        final Pose drawPoint20 = new Pose(46.0,85.2,-Math.PI/2);
        final Pose drawPoint20_bottom = new Pose(46.0,31.0,-Math.PI/2);
        final Pose drawPoint21 = new Pose(53.3,86.8,-Math.PI/2);
        final Pose drawPoint21_bottom = new Pose(53.3,32.9,-Math.PI/2);
        final Pose drawPoint22 = new Pose(60.3,86.9,-Math.PI/2);
        final Pose drawPoint22_bottom = new Pose(60.3,33.5,-Math.PI/2);
        final Pose drawPoint23 = new Pose(67.8,85.9,-Math.PI/2);
        final Pose drawPoint23_bottom = new Pose(67.8,37.9,-Math.PI/2);
        final Pose drawPoint24 = new Pose(75.1,83.5,-Math.PI/2);
        final Pose drawPoint24_bottom = new Pose(75.1,39.1,-Math.PI/2);
        final Pose drawPoint36 = new Pose(7.5,45.4,-Math.PI/2);
        final Pose drawPoint37 = new Pose(19.4,34.5,-Math.PI/2);
        final Pose drawPoint38 = new Pose(20.1,25.7,-Math.PI/2);
        final Pose orePassOppositePoint = new Pose(53,32.4,-Math.PI/2);

        final String YAML_FILE = "maps/mine-map-test.yaml"; // TODO: create OccupancyMap now once (for efficiency)
        final int maxVelocity = 8;

        AutonomousVehicle hum1 = new HumanDrivenVehicle(1, 0, Color.GREEN, Color.BLUE, maxVelocity, 2, YAML_FILE, 0.5, 0.5);
        AutonomousVehicle aut2 = new AutonomousVehicle(2, 0, Color.YELLOW, Color.YELLOW, maxVelocity, 2, YAML_FILE, 0.5, 0.5);
        // TODO: maxVelocity(2)=7, maxVelocity(tec)=15 -> v(2)=15
        System.out.println(VehiclesHashMap.getInstance().getList());

        // Instantiate a trajectory envelope coordinator.
        tec = new TrajectoryEnvelopeCoordinatorSimulation(2000, 1000, maxVelocity, 2);
        // Need to set up infrastructure that maintains the representation
        tec.setupSolver(0, 100000000);
        // Start the thread that checks and enforces dependencies at every clock tick
        tec.startInference();

        tec.setDefaultFootprint(hum1.getFootPrint());
        tec.placeRobot(hum1.getID(), orePassOppositePoint);
        tec.placeRobot(aut2.getID(), mainTunnelRight);

        Heuristics heuristics = new Heuristics();
        heuristics.robotIDToPrecedence.put(1, 20);
        heuristics.robotIDToPrecedence.put(2, 10);
        // tec.addComparator(heuristics.highestPrecedence());
        //tec.addComparator(heuristics.lowestIDNumber());
        tec.addComparator(heuristics.highestIDNumber());
        // TODO: demos (regarding precedence)

        tec.setUseInternalCriticalPoints(false);
        tec.setYieldIfParking(true);
        tec.setBreakDeadlocks(true, false, false);
        tec.setMotionPlanner(1, hum1.makePlanner()); // needed for re-planning

        // Set up a simple GUI (null means empty map, otherwise provide yaml file)
        var viz = new BrowserVisualization();
        viz.setMap(YAML_FILE);
        viz.setInitialTransform(7.0, 7.0, 0.0);
        tec.setVisualization(viz);

        Missions.setMap(YAML_FILE);
        Missions.startMissionDispatchers(tec, true, loopTime);
        Missions.loopMissions.put(1, false);

        PoseSteering[] path2 = aut2.getPath(mainTunnelRight, drawPoint19, true);
        var m2 = new Mission(aut2.getID(), path2);
        Missions.enqueueMission(m2);
    }
}
