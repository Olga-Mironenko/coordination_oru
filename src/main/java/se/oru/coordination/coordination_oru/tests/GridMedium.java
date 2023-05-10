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


public class GridMedium {
    public static void main(String[] args) {
        Printer.resetTime();
        Printer.print("started");

        BrowserVisualization.isStatusText = true;
        TrajectoryEnvelopeTrackerRK4.constantDelayTime = 100;
    //    GatedThread.enable();

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
        final String YAML_FILE = "maps/map-grid.yaml"; // TODO: create OccupancyMap now once (for efficiency)

        final Pose column1Top = new Pose(14.5,57.4, -Math.PI/2);
        final Pose column2Top = new Pose(30.0,57.4, -Math.PI/2);
        final Pose column3Top = new Pose(45.7,57.4,-Math.PI/2);
        final Pose column1Bottom = new Pose(14.5,3.0, -Math.PI);
        final Pose column2Bottom = new Pose(30.0,3.0, Math.PI);
        final Pose column3Bottom = new Pose(45.7,3.0,-Math.PI/2);
        final Pose row1Left = new Pose(4.0,44.0, -Math.PI/2);
        final Pose row2Left = new Pose(4.0,30.0,Math.PI/2);
        final Pose row3Left = new Pose(4.0,15.5,Math.PI/2);
        final Pose row1Right = new Pose(57.0,44.0,-Math.PI/2);
        final Pose row2Right = new Pose(57.0,30.0,-Math.PI/2);
        final Pose row3Right = new Pose(57.0,15.5,-Math.PI/2);


        final Pose humStart = column2Top;
        //    final Pose humFinish = null;
        final Pose humFinish = column2Bottom;
        final boolean ishumReturn = true;
        final boolean ishumLoop = true;
        final Pose aut1Start = column1Top;
        final Pose aut1Finish = column1Bottom;
        final Pose aut2Start = column3Top;
        final Pose aut2Finish = column3Bottom;
        final Pose aut3Start = row1Left;
        final Pose aut3Finish = row3Right;
        final Pose aut4Start = row2Left;
        final Pose aut4Finish = row2Right;
        final Pose aut5Start = row3Left;
        final Pose aut5Finish = row1Right;

        final int maxVelocity = 4;

        AutonomousVehicle hum0 = new HumanDrivenVehicle(0, Color.GREEN, Color.BLUE, maxVelocity, 2, 1.5, 1.5);
        AutonomousVehicle aut1 = new AutonomousVehicle(0, Color.YELLOW, Color.YELLOW, maxVelocity, 2, 1.5, 1.5);
        AutonomousVehicle aut2 = new AutonomousVehicle(0, Color.YELLOW, Color.YELLOW, maxVelocity, 2, 1.5, 1.5);
        AutonomousVehicle aut3 = new AutonomousVehicle(0, Color.YELLOW, Color.YELLOW, maxVelocity, 2, 1.5, 1.5);
        AutonomousVehicle aut4 = new AutonomousVehicle(0, Color.YELLOW, Color.YELLOW, maxVelocity, 2, 1.5, 1.5);
        AutonomousVehicle aut5 = new AutonomousVehicle(0, Color.YELLOW, Color.YELLOW, maxVelocity, 2, 1.5, 1.5);

        // TODO: maxVelocity(2)=7, maxVelocity(tec)=15 -> v(2)=15
        System.out.println(VehiclesHashMap.getInstance().getList());

        TrajectoryEnvelopeTrackerRK4.emergencyBreaker = new EmergencyBreaker(false, false);

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
        tec.placeRobot(aut3.getID(), aut3Start);
        tec.placeRobot(aut4.getID(), aut4Start);
        tec.placeRobot(aut5.getID(), aut5Start);

        Heuristics heuristics = new Heuristics();
        tec.addComparator(heuristics.closest());

        tec.setUseInternalCriticalPoints(false);
        tec.setYieldIfParking(true);
        tec.setBreakDeadlocks(true, false, false);
        //tec.setMotionPlanner(1, hum0.makePlanner()); // needed for re-planning

        // Set up a simple GUI (null means empty map, otherwise provide yaml file)
        var viz = new BrowserVisualization();
        viz.setMap(YAML_FILE);
        viz.setInitialTransform(7.0, 45.0, 5.0);
        tec.setVisualization(viz);

        Missions.setMap(YAML_FILE);
        Missions.startMissionDispatchers(tec, loopTime);
        Missions.loopMissions.put(hum0.getID(), ishumLoop);

        if (humFinish != null) {
            MissionUtils.targetVelocityHuman = 10;
            Missions.enqueueMission(new Mission(hum0.getID(), hum0.getPlan(humStart, new Pose[] { humFinish }, YAML_FILE, ishumReturn)));
        }

        Missions.enqueueMission(new Mission(aut1.getID(), aut1.getPlan(aut1Start, new Pose[] { aut1Finish }, YAML_FILE,  true)));
        Missions.enqueueMission(new Mission(aut2.getID(), aut2.getPlan(aut2Start, new Pose[] { aut2Finish }, YAML_FILE, true)));
        Missions.enqueueMission(new Mission(aut3.getID(), aut3.getPlan(aut3Start, new Pose[] { aut3Finish }, YAML_FILE,  true)));
        Missions.enqueueMission(new Mission(aut4.getID(), aut4.getPlan(aut4Start, new Pose[] { aut4Finish }, YAML_FILE, true)));
        Missions.enqueueMission(new Mission(aut5.getID(), aut5.getPlan(aut5Start, new Pose[] { aut5Finish }, YAML_FILE,  true)));

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
                    MissionUtils.changeTargetVelocityHuman(1);
                }
            }.start();
        }

        final boolean isNewMission = true;
        if (isNewMission) {
            new GatedThread("new mission") {
                @Override
                public void runCore() {
                    GatedThread.skipCycles(100);
                    MissionUtils.moveRobot(hum0.getID(), column2Bottom);
                    GatedThread.skipCycles(10);
                    MissionUtils.changeTargetVelocityHuman(1); // requires emergency break
                }
            }.start();
        }
    }
}
