package se.oru.coordination.coordination_oru.scenarios;

import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.PoseSteering;
import se.oru.coordination.coordination_oru.ConstantAccelerationForwardModel;
import se.oru.coordination.coordination_oru.Mission;
import se.oru.coordination.coordination_oru.code.AutonomousVehicle;
import se.oru.coordination.coordination_oru.code.Heuristics;
import se.oru.coordination.coordination_oru.code.LookAheadVehicle;
import se.oru.coordination.coordination_oru.simulation2D.TrajectoryEnvelopeCoordinatorSimulation;
import se.oru.coordination.coordination_oru.util.BrowserVisualization;
import se.oru.coordination.coordination_oru.util.Missions;

import java.awt.*;

public class ProductionCycleMine {
    public static void main(String[] args) throws InterruptedException {

        final int loopMinutes = 5;
        final long loopTime = System.currentTimeMillis() + (loopMinutes * 60 * 1000);
        final double predictableDistance = 25.0;
        final String YAML_FILE = "maps/mine-map-test.yaml";
        final Pose mainTunnelLeft = new Pose(4.25, 15.35, -Math.PI);
        final Pose mainTunnelRight = new Pose(80.05, 24.75, Math.PI);
        final Pose drawPoint16 = new Pose(16.75, 87.15, -Math.PI / 2);
        final Pose drawPoint38 = new Pose(19.55, 26.25, -Math.PI / 2);
        final Pose drawPoint18 = new Pose(24.15, 85.55, -Math.PI / 2);
        final Pose drawPoint23 = new Pose(67.75, 86.95, -Math.PI / 2);
        final Pose drawPoint24 = new Pose(75.05, 84.65, -Math.PI / 2);
        final Pose orePass = new Pose(54.35, 11.25, -Math.PI / 2);

        final Pose[] autonomousVehicleGoal = {orePass};

        var drillVehicle = new LookAheadVehicle(1, predictableDistance, Color.CYAN, 5, 2, 0.5, 0.5);
        var chargingVehicle = new LookAheadVehicle(1, 6 * predictableDistance, Color.WHITE, 5, 2, 0.5, 0.5);

        var autonomousVehicle1 = new AutonomousVehicle();
        var autonomousVehicle2 = new AutonomousVehicle();
        autonomousVehicle1.getPlan(drawPoint16, autonomousVehicleGoal, YAML_FILE, true);
        autonomousVehicle2.getPlan(drawPoint23, autonomousVehicleGoal, YAML_FILE, true);

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

        tec.addComparator(new Heuristics().lowestIDNumber());
        tec.setUseInternalCriticalPoints(false);
        tec.setYieldIfParking(true);
        tec.setBreakDeadlocks(true, false, false);

        // Set up a simple GUI (null means empty map, otherwise provide yaml file)
        var viz = new BrowserVisualization(YAML_FILE);
        viz.setFontScale(4);
        viz.setInitialTransform(11, 45, -3.5);
        tec.setVisualization(viz);

        var m1 = new Mission(autonomousVehicle1.getID(), autonomousVehicle1.getPath());
        var m2 = new Mission(autonomousVehicle2.getID(), autonomousVehicle2.getPath());
        m2.setStoppingPoint(orePass, 5000);

        Missions.enqueueMission(m1);
        Missions.enqueueMission(m2);
        Missions.setMap(YAML_FILE);
        Missions.startMissionDispatchers(tec, loopTime);

        long missionTime = 5000;
        tec.setForwardModel(drillVehicle.getID(), new ConstantAccelerationForwardModel(drillVehicle.getMaxAcceleration(), drillVehicle.getMaxVelocity(), tec.getTemporalResolution(),
                tec.getControlPeriod(), tec.getRobotTrackingPeriodInMillis(drillVehicle.getID())));
        tec.setForwardModel(chargingVehicle.getID(), new ConstantAccelerationForwardModel(chargingVehicle.getMaxAcceleration(), chargingVehicle.getMaxVelocity(), tec.getTemporalResolution(),
                tec.getControlPeriod(), tec.getRobotTrackingPeriodInMillis(chargingVehicle.getID())));

        final Pose[] drillRigGoal = {drawPoint38, drawPoint18, drawPoint24, mainTunnelRight};
        final Pose[] chargingVehicleGoal = {drawPoint24, drawPoint23, drawPoint18, drawPoint16, drawPoint38, mainTunnelLeft};

        drillVehicle.getPlan(mainTunnelLeft, drillRigGoal, YAML_FILE, false);
        chargingVehicle.getPlan(mainTunnelRight, chargingVehicleGoal, YAML_FILE, false);
        Thread.sleep(5000);
        tec.placeRobot(drillVehicle.getID(), mainTunnelLeft);
        PoseSteering[] drillInitialPath = drillVehicle.getLimitedPath(drillVehicle.getID(), drillVehicle.getPredictableDistance(), tec);
        var m3 = new Mission(drillVehicle.getID(), drillInitialPath);
//        Missions.enqueueMission(m3);
        tec.addMissions(m3);
        m3.setStoppingPoint(drawPoint38, 5000);
        Thread.sleep(10000);
        tec.placeRobot(chargingVehicle.getID(), mainTunnelRight);
        PoseSteering[] chargingInitialPath = chargingVehicle.getLimitedPath(chargingVehicle.getID(), chargingVehicle.getPredictableDistance(), tec);
        var m4 = new Mission(chargingVehicle.getID(), chargingInitialPath);
        tec.addMissions(m4);

//        while(true) {
//            if (!tec.isDriving(drillVehicle.getID())) break;
//        }
//        System.out.println(tec.isDriving(drillVehicle.getID()));
//        System.out.println(tec.isFree(drillVehicle.getID()));
//        System.out.println(tec.isParked(drillVehicle.getID()));

//        for (int i = 0; i < drillRigLocations.length; i++) {
//            PoseSteering[] drillRigPath = drillVehicle.getPlan(drillRigLocations[i], new Pose[]{drillRigLocations[i++]}, false);
//            Thread.sleep(missionTime);
//            tec.placeRobot(drillVehicle.getID(), drillRigLocations[i]);
//            var m = new Mission(drillVehicle.getID(), drillRigPath);
//            Missions.enqueueMission(m);
//            tec.addMissions(m);
//        }

//        PoseSteering[] drillRigPath = drillVehicle.getPlan(mainTunnelLeft, drillRigGoal, false);
//        Thread.sleep(1000);
//        tec.placeRobot(drillVehicle.getID(), mainTunnelLeft);
//        var m3 = new Mission(drillVehicle.getID(), drillRigPath);
//        Missions.enqueueMission(m3);
//        tec.addMissions(m3);

        // TODO How does a robot move? Which gives the command for robot to move?
    }
}
