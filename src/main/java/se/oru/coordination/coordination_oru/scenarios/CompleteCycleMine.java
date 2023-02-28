package se.oru.coordination.coordination_oru.scenarios;

import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.PoseSteering;
import se.oru.coordination.coordination_oru.ConstantAccelerationForwardModel;
import se.oru.coordination.coordination_oru.Mission;
import se.oru.coordination.coordination_oru.code.AutonomousVehicle;
import se.oru.coordination.coordination_oru.code.Heuristics;
import se.oru.coordination.coordination_oru.code.LimitedPredictabilityVehicle;
import se.oru.coordination.coordination_oru.code.VehiclesHashMap;
import se.oru.coordination.coordination_oru.simulation2D.TrajectoryEnvelopeCoordinatorSimulation;
import se.oru.coordination.coordination_oru.util.BrowserVisualization;
import se.oru.coordination.coordination_oru.util.Missions;

import java.awt.*;

public class CompleteCycleMine {
    public static void main(String[] args) throws InterruptedException {

        final int loopMinutes = 5;
        final long loopTime = System.currentTimeMillis() + (loopMinutes * 60 * 1000);
        final double predictableDistance = 20.0;
        final Pose mainTunnelLeft = new Pose(4.25, 15.35, -Math.PI);
        final Pose mainTunnelRight = new Pose(80.05, 24.75, Math.PI);
        final Pose drawPoint16 = new Pose(16.75, 87.15, -Math.PI / 2);
        final Pose drawPoint38 = new Pose(19.55, 26.25, -Math.PI / 2);
        final Pose drawPoint18 = new Pose(24.15, 85.55, -Math.PI / 2);
        final Pose drawPoint23 = new Pose(67.75, 86.95, -Math.PI / 2);
        final Pose drawPoint24 = new Pose(75.05, 84.65, -Math.PI / 2);
        final Pose orePass = new Pose(54.35, 11.25, -Math.PI / 2);
        final String YAML_FILE = "maps/mine-map-test.yaml";

        final Pose[] autonomousVehicleGoal = {orePass};

        var autonomousVehicle1 = new AutonomousVehicle(1, 1, Color.GREEN, 12, 6, YAML_FILE, 0.5, 0.5);
        var autonomousVehicle2 = new AutonomousVehicle(2, 1, Color.GREEN, 12, 6, YAML_FILE, 0.5, 0.5);
        var drillRig = new LimitedPredictabilityVehicle(3, 1, predictableDistance, Color.YELLOW, 5, 2, YAML_FILE, 0.5, 0.5);
        PoseSteering[] autonomousVehicle1Path = autonomousVehicle1.getPlan(drawPoint16, autonomousVehicleGoal, YAML_FILE, true);
        PoseSteering[] autonomousVehicle2Path = autonomousVehicle2.getPlan(drawPoint23, autonomousVehicleGoal, YAML_FILE, true);

        // Instantiate a trajectory envelope coordinator.
        final var tec = new TrajectoryEnvelopeCoordinatorSimulation(2000, 1000, 5, 2);
        // Need to set up infrastructure that maintains the representation
        tec.setupSolver(0, 100000000);
        // Start the thread that checks and enforces dependencies at every clock tick
        tec.startInference();

        tec.setDefaultFootprint(autonomousVehicle1.getFootPrint());
        tec.placeRobot(autonomousVehicle1.getID(), drawPoint16);
        tec.placeRobot(autonomousVehicle2.getID(), drawPoint23);
        tec.setForwardModel(autonomousVehicle1.getID(), new ConstantAccelerationForwardModel(autonomousVehicle1.getMaxAcceleration(), autonomousVehicle1.getMaxVelocity(), tec.getTemporalResolution(),
                tec.getControlPeriod(), tec.getRobotTrackingPeriodInMillis(autonomousVehicle1.getID())));
        tec.setForwardModel(autonomousVehicle2.getID(), new ConstantAccelerationForwardModel(autonomousVehicle2.getMaxAcceleration(), autonomousVehicle2.getMaxVelocity(), tec.getTemporalResolution(),
                tec.getControlPeriod(), tec.getRobotTrackingPeriodInMillis(autonomousVehicle2.getID())));

        tec.addComparator(new Heuristics().closest());
        tec.setUseInternalCriticalPoints(false);
        tec.setYieldIfParking(true);
        tec.setBreakDeadlocks(true, false, false);

        // Set up a simple GUI (null means empty map, otherwise provide yaml file)
        var viz = new BrowserVisualization();
        viz.setMap(YAML_FILE);
        viz.setFontScale(4);
        viz.setInitialTransform(9.6, 60.8, -2.4);
        tec.setVisualization(viz);

        var m1 = new Mission(autonomousVehicle1.getID(), autonomousVehicle1Path);
        var m2 = new Mission(autonomousVehicle2.getID(), autonomousVehicle2Path);

        Missions.enqueueMission(m1);
        Missions.enqueueMission(m2);
        Missions.setMap(YAML_FILE);
        Missions.startMissionDispatchers(tec, loopTime);

        tec.setForwardModel(drillRig.getID(), new ConstantAccelerationForwardModel(drillRig.getMaxAcceleration(), drillRig.getMaxVelocity(), tec.getTemporalResolution(),
                tec.getControlPeriod(), tec.getRobotTrackingPeriodInMillis(drillRig.getID())));
        final Pose[] drillRigGoal = {drawPoint38, drawPoint18, drawPoint24, mainTunnelRight};
        PoseSteering[] drillRigPath = drillRig.getPlan(mainTunnelLeft, drillRigGoal, YAML_FILE, false);
        Thread.sleep(3000);
        tec.placeRobot(drillRig.getID(), mainTunnelLeft);
        var m3 = new Mission(drillRig.getID(), drillRigPath);
        Missions.enqueueMission(m3);
        tec.addMissions(m3);
        tec.replacePath(drillRig.getID(), drillRigPath, 0, false, null);

            // TODO How does a robot move? Which gives the command for robot to move?
        }
    }
