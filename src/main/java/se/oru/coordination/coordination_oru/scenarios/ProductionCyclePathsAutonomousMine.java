package se.oru.coordination.coordination_oru.scenarios;

import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.PoseSteering;
import se.oru.coordination.coordination_oru.ConstantAccelerationForwardModel;
import se.oru.coordination.coordination_oru.Mission;
import se.oru.coordination.coordination_oru.code.AutonomousVehicle;
import se.oru.coordination.coordination_oru.code.Heuristics;
import se.oru.coordination.coordination_oru.motionplanning.ompl.ReedsSheppCarPlanner;
import se.oru.coordination.coordination_oru.simulation2D.TrajectoryEnvelopeCoordinatorSimulation;
import se.oru.coordination.coordination_oru.util.BrowserVisualization;
import se.oru.coordination.coordination_oru.util.Missions;

import java.awt.*;

public class ProductionCyclePathsAutonomousMine {
    public static void main(String[] args) throws InterruptedException {

        final int loopMinutes = 5;
        final long loopTime = System.currentTimeMillis() + (loopMinutes * 60 * 1000);
        final String YAML_FILE = "maps/mine-map-new.yaml";
        final Pose mainTunnelLeft = new Pose(4.25, 15.35, -Math.PI);
        final Pose mainTunnelRight = new Pose(80.05, 24.75, Math.PI);
        final Pose drawPoint15 = new Pose(9.65, 84.35, -Math.PI / 2);
        final Pose drawPoint16 = new Pose(16.75, 87.15, -Math.PI / 2);
        final Pose drawPoint17 = new Pose(24.15, 84.95, -Math.PI / 2);
        final Pose drawPoint18 = new Pose(31.35, 85.65, -Math.PI / 2);
        final Pose drawPoint19 = new Pose(38.85, 85.35, -Math.PI / 2);
        final Pose drawPoint20 = new Pose(45.85, 84.65, -Math.PI / 2);
        final Pose drawPoint21 = new Pose(53.05, 87.45, -Math.PI / 2);
        final Pose drawPoint22 = new Pose(60.25, 87.55, -Math.PI / 2);
        final Pose drawPoint23 = new Pose(67.75, 86.95, -Math.PI / 2);
        final Pose drawPoint24 = new Pose(75.05, 84.65, -Math.PI / 2);
        final Pose drawPoint36 = new Pose(7.25, 45.75, -Math.PI / 2);
        final Pose drawPoint37 = new Pose(19.55, 34.85, -Math.PI / 2);
        final Pose drawPoint38 = new Pose(19.55, 26.25, -Math.PI / 2);
        final Pose orePass = new Pose(54.35, 11.25, -Math.PI / 2);
        final Pose workStation1 = new Pose(23.75, 8.95, -Math.PI / 2);
        final Pose workStation2 = new Pose(20.15, 9.05, -Math.PI / 2);
        final Pose workStation3 = new Pose(17.35, 9.65, -Math.PI / 2);

        final Pose[] autonomousVehicleGoal = {orePass};

        var drillVehicle = new AutonomousVehicle(1, Color.MAGENTA, 5, 2, 0.5, 0.5);
        var chargingVehicle = new AutonomousVehicle(1, Color.PINK, 5, 2, 0.5, 0.5);
        var waterVehicle = new AutonomousVehicle(1, Color.BLUE, 5, 2, 0.5, 0.5);

        var autonomousVehicle1 = new AutonomousVehicle();
        var autonomousVehicle2 = new AutonomousVehicle();
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
//        viz.setFontScale(4);
        viz.setInitialTransform(11, 45, -3.5);
        tec.setVisualization(viz);

        var m1 = new Mission(autonomousVehicle1.getID(), autonomousVehicle1Path);
        var m2 = new Mission(autonomousVehicle2.getID(), autonomousVehicle2Path);
//        m1.setStoppingPoint(orePass, 5000);
//        m2.setStoppingPoint(orePass, 5000);

        Missions.enqueueMission(m1);
        Missions.enqueueMission(m2);
        Missions.setMap(YAML_FILE);
        Missions.startMissionDispatchers(tec, loopTime);

        tec.setForwardModel(drillVehicle.getID(), new ConstantAccelerationForwardModel(drillVehicle.getMaxAcceleration(), drillVehicle.getMaxVelocity(), tec.getTemporalResolution(),
                tec.getControlPeriod(), tec.getRobotTrackingPeriodInMillis(drillVehicle.getID())));
        tec.setForwardModel(chargingVehicle.getID(), new ConstantAccelerationForwardModel(chargingVehicle.getMaxAcceleration(), chargingVehicle.getMaxVelocity(), tec.getTemporalResolution(),
                tec.getControlPeriod(), tec.getRobotTrackingPeriodInMillis(chargingVehicle.getID())));
        tec.setForwardModel(waterVehicle.getID(), new ConstantAccelerationForwardModel(waterVehicle.getMaxAcceleration(), waterVehicle.getMaxVelocity(), tec.getTemporalResolution(),
                tec.getControlPeriod(), tec.getRobotTrackingPeriodInMillis(waterVehicle.getID())));

        // FIXME Maybe try smaller motion plans
        final Pose[] drillRigGoal = {drawPoint38, drawPoint18, drawPoint24, workStation1};
        final Pose[] chargingVehicleGoal = {drawPoint38, drawPoint18, drawPoint24, workStation2};
        final Pose[] waterVehicleGoal = {drawPoint24, drawPoint23, drawPoint22, drawPoint21,
                drawPoint20, drawPoint19, drawPoint18, drawPoint17, drawPoint16, drawPoint15,
                drawPoint36, drawPoint37, drawPoint38, workStation3};

        PoseSteering[] drillRigPath1 = drillVehicle.getPlan(mainTunnelLeft, new Pose[]{drawPoint38}, YAML_FILE, false);
        PoseSteering[] drillRigPath2 = drillVehicle.getPlan(drawPoint38, new Pose[]{drawPoint18}, YAML_FILE,  false);
        PoseSteering[] drillRigPath3 = drillVehicle.getPlan(drawPoint18, new Pose[]{drawPoint24}, YAML_FILE,  false);
        PoseSteering[] drillRigPath4 = drillVehicle.getPlan(drawPoint24, new Pose[]{workStation1}, YAML_FILE,  false);
//        PoseSteering[] chargingVehiclePath = chargingVehicle.getPlan(mainTunnelRight, chargingVehicleGoal, YAML_FILE, false);
//        PoseSteering[] waterVehiclePath = waterVehicle.getPlan(mainTunnelRight, waterVehicleGoal, YAML_FILE, false,
//                ReedsSheppCarPlanner.PLANNING_ALGORITHM.RRTConnect, 0.01, 120, 0.01, 0.1);

        Thread.sleep(5000);
        tec.placeRobot(drillVehicle.getID(), mainTunnelLeft);
        var m3 = new Mission(drillVehicle.getID(), drillRigPath1);
        tec.addMissions(m3);
//        Thread.sleep(2000);
//        var m5 = new Mission(drillVehicle.getID(), drillRigPath3);
//        tec.addMissions(m5);
//        Thread.sleep(2000);
//        var m6 = new Mission(drillVehicle.getID(), drillRigPath4);
//        tec.addMissions(m6);

//        Thread.sleep(10000);
//        tec.placeRobot(chargingVehicle.getID(), mainTunnelRight);
//        var m4 = new Mission(chargingVehicle.getID(), chargingVehiclePath);
//        tec.addMissions(m4);
//
//        Thread.sleep(20000);
//        tec.placeRobot(waterVehicle.getID(), mainTunnelRight);
//        var m5 = new Mission(waterVehicle.getID(), waterVehiclePath);
//        tec.addMissions(m5);

        }
    }
