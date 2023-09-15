package se.oru.coordination.coordination_oru.tests;

import java.awt.Color;

import org.metacsp.multi.spatioTemporal.paths.Pose;

import se.oru.coordination.coordination_oru.Mission;
import se.oru.coordination.coordination_oru.code.AutonomousVehicle;
import se.oru.coordination.coordination_oru.code.Heuristics;
import se.oru.coordination.coordination_oru.code.HumanDrivenVehicle;
import se.oru.coordination.coordination_oru.code.VehiclesHashMap;
import se.oru.coordination.coordination_oru.simulation2D.EmergencyBreaker;
import se.oru.coordination.coordination_oru.simulation2D.TrajectoryEnvelopeCoordinatorSimulation;
import se.oru.coordination.coordination_oru.simulation2D.TrajectoryEnvelopeTrackerRK4;
import se.oru.coordination.coordination_oru.util.*;
import se.oru.coordination.coordination_oru.util.gates.GatedThread;

public class TestClick {
    public static void main(String[] args) {
        new Demo() {
            @Override
            protected void run(String scenarioString) {
                assert scenarioString == null;
                runDemo();
            }
        }.exec();
    }

    protected static void runDemo() {
        final int loopMinutes = 5;
        final long loopTime = System.currentTimeMillis() + (loopMinutes * 60 * 1000);
        final String YAML_FILE = "maps/mine-map-test.yaml"; // TODO: create OccupancyMap now once (for efficiency)

        final Pose mainTunnelLeft = new Pose(4.25,15.35, -Math.PI);
        final Pose mainTunnelRight = new Pose(80.05,24.75, Math.PI);
        final Pose orePass = new Pose(54.11,11.34,-Math.PI/2);
        final Pose drawPoint15 = new Pose(9.9,84.5,-Math.PI/2);
        final Pose drawPoint16 = new Pose(17.1,84.6,-Math.PI/2);
        final Pose drawPoint17 = new Pose(24.3,85.45,-Math.PI/2);
        final Pose drawPoint18 = new Pose(31.6,84.6,-Math.PI/2);
        final Pose drawPoint19 = new Pose(39.05,85.45,-Math.PI/2);
        final Pose drawPoint19_bottom = new Pose(38.8,28.6,-Math.PI/2);
        final Pose mainTunnelBetween19And20 = new Pose(43.57,17.85, -Math.PI);
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

        final Pose humStart = mainTunnelBetween19And20;
        final Pose humFinish = null;
        final boolean ishumReturn = false;
        final boolean ishumLoop = false;
        final Pose aut1Start = mainTunnelRight;
        final Pose aut1Finish = drawPoint19_bottom;
        final Pose aut2Start = mainTunnelLeft;
        final Pose aut2Finish = drawPoint18;

        final int maxVelocity = 8;

        AutonomousVehicle hum0 = new HumanDrivenVehicle(0, Color.GREEN, Color.BLUE, maxVelocity, 2, 0.5, 0.5);
        AutonomousVehicle aut1 = new AutonomousVehicle(0, Color.YELLOW, Color.YELLOW, maxVelocity, 2, 0.5, 0.5);
        AutonomousVehicle aut2 = new AutonomousVehicle(0, Color.RED, Color.RED, maxVelocity, 2, 0.5, 0.5);
        // TODO: maxVelocity(2)=7, maxVelocity(tec)=15 -> v(2)=15
        System.out.println(VehiclesHashMap.getInstance().getList());

        TrajectoryEnvelopeTrackerRK4.emergencyBreaker = new EmergencyBreaker(false, true);

        // Instantiate a trajectory envelope coordinator.
        TrajectoryEnvelopeCoordinatorSimulation tec = new TrajectoryEnvelopeCoordinatorSimulation(2000, 1000, maxVelocity, 2);
        // Need to set up infrastructure that maintains the representation
        tec.setupSolver(0, 100000000);
        // Start the thread that checks and enforces dependencies at every clock tick
        tec.startInference();

        tec.setDefaultFootprint(hum0.getFootprint());
        tec.placeRobot(hum0.getID(), humStart);
        tec.placeRobot(aut1.getID(), aut1Start);
        tec.placeRobot(aut2.getID(), aut2Start);

        Heuristics heuristics = new Heuristics();
        tec.addComparator(heuristics.highestIDNumber());

        tec.setUseInternalCriticalPoints(false);
        tec.setYieldIfParking(true);
        tec.setBreakDeadlocks(true, false, false);
        //tec.setMotionPlanner(1, hum0.makePlanner()); // needed for re-planning

        // Set up a simple GUI (null means empty map, otherwise provide yaml file)
        var viz = new BrowserVisualization();
        viz.setMap(YAML_FILE);
        viz.setInitialTransform(7.0, 7.0, 0.0);
        tec.setVisualization(viz);

        Missions.setMap(YAML_FILE);
        Missions.startMissionDispatchers(tec, loopTime);
        Missions.loopMissions.put(hum0.getID(), ishumLoop);

        if (humFinish != null) {
            HumanControl.targetVelocityHuman = 10;
            hum0.getPlan(humStart, new Pose[] { humFinish }, YAML_FILE, ishumReturn);
            Missions.enqueueMission(new Mission(hum0.getID(), (hum0.getPath())));
        }

        aut1.getPlan(aut1Start, new Pose[] { aut1Finish }, YAML_FILE,  true);
        aut2.getPlan(aut2Start, new Pose[] { aut2Finish }, YAML_FILE, true);
        Missions.enqueueMission(new Mission(aut1.getID(), aut1.getPath()));
        Missions.enqueueMission(new Mission(aut2.getID(), aut2.getPath()));

        final boolean isChangeVelocity = false;
        if (isChangeVelocity) {
            new GatedThread("change velocity") {
                @Override
                public void runCore() {
                    for (int i = 0; i < 10; i++) {
                        try {
                            GatedThread.sleep(i);
                        } catch (InterruptedException e) {
                            throw new RuntimeException(e);
                        }
                    }
                    HumanControl.changeTargetVelocityHuman(1);
                }
            }.start();
        }

        final boolean isNewMission = true;
        if (isNewMission) {
            new GatedThread("new mission") {
                @Override
                public void runCore() {
                    GatedThread.skipCycles(100);
                    HumanControl.moveRobot(hum0.getID(), drawPoint20_bottom);
                    GatedThread.skipCycles(10);
                    HumanControl.changeTargetVelocityHuman(1); // requires emergency break
                }
            }.start();
        }
    }
}