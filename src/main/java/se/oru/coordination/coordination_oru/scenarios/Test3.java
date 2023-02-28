package se.oru.coordination.coordination_oru.scenarios;

import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.PoseSteering;
import se.oru.coordination.coordination_oru.ConstantAccelerationForwardModel;
import se.oru.coordination.coordination_oru.Mission;
import se.oru.coordination.coordination_oru.code.AutonomousVehicle;
import se.oru.coordination.coordination_oru.code.Heuristics;
import se.oru.coordination.coordination_oru.code.LimitedPredictabilityVehicle;
import se.oru.coordination.coordination_oru.simulation2D.TrajectoryEnvelopeCoordinatorSimulation;
import se.oru.coordination.coordination_oru.util.BrowserVisualization;
import se.oru.coordination.coordination_oru.util.Missions;

import java.awt.*;

public class Test3 {
    public static void main(String[] args) {

        final int loopMinutes = 5;
        final long loopTime = System.currentTimeMillis() + (loopMinutes * 60 * 1000);
        final double predictableDistance = 15.0;
        final Pose mainTunnelLeft = new Pose(4.25,15.35, -Math.PI);
        final Pose mainTunnelRight = new Pose(80.05,24.75, Math.PI);
        final Pose drawPoint18 = new Pose(24.15,85.55,-Math.PI/2);
        final Pose drawPoint21 = new Pose(52.95,87.75,-Math.PI/2);
        final Pose drawPoint22 = new Pose(60.35,87.85,-Math.PI/2);
        final Pose drawPoint23 = new Pose(67.75,86.95,-Math.PI/2);
        final Pose orePass = new Pose(54.35,11.25,-Math.PI/2);
        final String YAML_FILE = "maps/mine-map-test.yaml";

        final Pose[] autonomousVehicleGoal = {orePass};
        final Pose[] limitedVehicleGoal = {mainTunnelRight};

        var autonomousVehicle1 = new AutonomousVehicle(1, 1, Color.GREEN, 5, 2, YAML_FILE, 0.5, 0.5);
        var autonomousVehicle2 = new AutonomousVehicle(2, 1, Color.GREEN, 5, 2, YAML_FILE, 0.5, 0.5);
        var autonomousVehicle3 = new AutonomousVehicle(3, 1, Color.GREEN, 5, 2, YAML_FILE, 0.5, 0.5);
        var limitedPredictabilityVehicle1 = new LimitedPredictabilityVehicle(4, 1, predictableDistance, Color.RED, 5, 2, YAML_FILE, 0.5, 0.5);
        var AVPlan1 = autonomousVehicle1.getPlan(drawPoint18, autonomousVehicleGoal, YAML_FILE, true);
        var AVPlan2 = autonomousVehicle1.getPlan(drawPoint21, autonomousVehicleGoal, YAML_FILE, true);
        var AVPlan3 = autonomousVehicle1.getPlan(drawPoint23, autonomousVehicleGoal, YAML_FILE, true);
        var LPVPlan1 = limitedPredictabilityVehicle1.getPlan(mainTunnelLeft, limitedVehicleGoal, YAML_FILE, true);

        // Instantiate a trajectory envelope coordinator.
        final var tec = new TrajectoryEnvelopeCoordinatorSimulation(2000, 1000, 5, 2);
        // Need to set up infrastructure that maintains the representation
        tec.setupSolver(0, 100000000);
        // Start the thread that checks and enforces dependencies at every clock tick
        tec.startInference();

        tec.setDefaultFootprint(limitedPredictabilityVehicle1.getFootPrint());
        tec.placeRobot(autonomousVehicle1.getID(), drawPoint18);
        tec.placeRobot(autonomousVehicle2.getID(), drawPoint21);
        tec.placeRobot(autonomousVehicle3.getID(), drawPoint23);
        tec.placeRobot(limitedPredictabilityVehicle1.getID(), mainTunnelLeft);
        tec.setForwardModel(autonomousVehicle1.getID(), new ConstantAccelerationForwardModel(autonomousVehicle1.getMaxAcceleration(), autonomousVehicle1.getMaxVelocity(), tec.getTemporalResolution(),
                tec.getControlPeriod(), tec.getRobotTrackingPeriodInMillis(autonomousVehicle1.getID())));
        tec.setForwardModel(autonomousVehicle2.getID(), new ConstantAccelerationForwardModel(autonomousVehicle2.getMaxAcceleration(), autonomousVehicle2.getMaxVelocity(), tec.getTemporalResolution(),
                tec.getControlPeriod(), tec.getRobotTrackingPeriodInMillis(autonomousVehicle2.getID())));
        tec.setForwardModel(autonomousVehicle3.getID(), new ConstantAccelerationForwardModel(autonomousVehicle3.getMaxAcceleration(), autonomousVehicle3.getMaxVelocity(), tec.getTemporalResolution(),
                tec.getControlPeriod(), tec.getRobotTrackingPeriodInMillis(autonomousVehicle3.getID())));
        tec.setForwardModel(limitedPredictabilityVehicle1.getID(), new ConstantAccelerationForwardModel(limitedPredictabilityVehicle1.getMaxAcceleration(), limitedPredictabilityVehicle1.getMaxVelocity(), tec.getTemporalResolution(),
                tec.getControlPeriod(), tec.getRobotTrackingPeriodInMillis(limitedPredictabilityVehicle1.getID())));
        tec.addComparator(new Heuristics().closest());
        tec.setUseInternalCriticalPoints(false);
        tec.setYieldIfParking(true);
        tec.setBreakDeadlocks(true, false, false);

        // Set up a simple GUI (null means empty map, otherwise provide yaml file)
        var viz = new BrowserVisualization();
        viz.setMap(YAML_FILE);
        viz.setInitialTransform(7.0, 83.0, 15.5);
        tec.setVisualization(viz);

        var m1 = new Mission(autonomousVehicle1.getID(), AVPlan1);
        var m2 = new Mission(autonomousVehicle2.getID(), AVPlan2);
        var m3 = new Mission(autonomousVehicle3.getID(), AVPlan3);
        var m4 = new Mission(limitedPredictabilityVehicle1.getID(), LPVPlan1);

        Missions.enqueueMission(m1);
        Missions.enqueueMission(m2);
        Missions.enqueueMission(m3);
        Missions.enqueueMission(m4);
        Missions.setMap(YAML_FILE);
        Missions.startMissionDispatchers(tec, loopTime);

    }
}
