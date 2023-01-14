//package se.oru.coordination.coordination_oru.scenarios;
//
//import com.vividsolutions.jts.geom.Coordinate;
//import org.apache.commons.lang.ArrayUtils;
//import org.metacsp.multi.spatioTemporal.paths.Pose;
//import org.metacsp.multi.spatioTemporal.paths.PoseSteering;
//import se.oru.coordination.coordination_oru.ConstantAccelerationForwardModel;
//import se.oru.coordination.coordination_oru.Mission;
//import se.oru.coordination.coordination_oru.code.Heuristics;
//import se.oru.coordination.coordination_oru.code.AutonomousVehicle;
//import se.oru.coordination.coordination_oru.demo.DemoDescription;
//import se.oru.coordination.coordination_oru.motionplanning.ompl.ReedsSheppCarPlanner;
//import se.oru.coordination.coordination_oru.simulation2D.TrajectoryEnvelopeCoordinatorSimulation;
//import se.oru.coordination.coordination_oru.util.BrowserVisualization;
//import se.oru.coordination.coordination_oru.util.Missions;
//
//import java.io.IOException;
//import java.util.ArrayList;
//
//@DemoDescription(desc = "Example of a single human driven and seven autonomous vehicles using heuristics for computation time, " +
//        "navigating in a test mine map.")
//public class OneLimitedVisibilitySevenAutonomous {
//
//    public static void main(String[] args) throws InterruptedException, IOException {
//
//        String yamlFile = "maps/mine-map-test.yaml";
//        double maxAccel = 2.0;
//        double maxVel = 6.0;
//        double avgVel = maxVel * 0.75;
//        long dt = 2000;     // In milliseconds
//        double dx = avgVel * dt/1000;
//        double lookAheadDistance = 10.0;
//
//        // Poses
//        Pose mainTunnelLeft = new Pose(4.25,15.35, -Math.PI);
//        Pose mainTunnelRight = new Pose(80.05,24.75, Math.PI);
//        Pose orePass = new Pose(54.35,11.25,-Math.PI/2);
//        Pose drawPoint18 = new Pose(31.65,84.95,-Math.PI/2);
//        Pose drawPoint19 = new Pose(39.05,85.45,-Math.PI/2);
//        Pose drawPoint20 = new Pose(45.85,85.55,-Math.PI/2);
//        Pose drawPoint21 = new Pose(52.95,87.75,-Math.PI/2);
//        Pose drawPoint22 = new Pose(60.35,87.85,-Math.PI/2);
//        Pose drawPoint23 = new Pose(67.75,86.95,-Math.PI/2);
//        Pose drawPoint24 = new Pose(75.05,84.55,-Math.PI/2);
//
//        // Instantiate a trajectory envelope coordinator.
//        final var tec = new TrajectoryEnvelopeCoordinatorSimulation(2000, 1000, maxVel, maxAccel);
//
//        tec.addComparator(new Heuristics().closest());
//        tec.setUseInternalCriticalPoints(false);
//        tec.setYieldIfParking(true);
//        tec.setBreakDeadlocks(true, false, false);
//
//        double xl = 0.5;
//        double yl = 0.5;
//        Coordinate footprint1 = new Coordinate(-xl, yl);
//        Coordinate footprint2 = new Coordinate(xl, yl);
//        Coordinate footprint3 = new Coordinate(xl, -yl);
//        Coordinate footprint4 = new Coordinate(-xl, -yl);
//        tec.setDefaultFootprint(footprint1, footprint2, footprint3, footprint4);
//
//        // Need to set up infrastructure that maintains the representation
//        tec.setupSolver(0, 100000000);
//        // Start the thread that checks and enforces dependencies at every clock tick
//        tec.startInference();
//
//        // Set up a simple GUI (null means empty map, otherwise provide yaml file)
//        var viz = new BrowserVisualization();
//        viz.setMap(yamlFile);
//        viz.setInitialTransform(7.0, 83.0, 15.5);
//        tec.setVisualization(viz);
//
//        // Set up initial and goal poses
//        Pose initial0 = mainTunnelLeft;
//        Pose initial1 = drawPoint18;
//        Pose initial2 = drawPoint19;
//        Pose initial3 = drawPoint20;
//        Pose initial4 = drawPoint21;
//        Pose initial5 = drawPoint22;
//        Pose initial6 = drawPoint23;
//        Pose initial7 = drawPoint24;
//
//        Pose goal0 = mainTunnelRight;
//        Pose goal1 = orePass;
//        Pose goal2 = orePass;
//        Pose goal3 = orePass;
//        Pose goal4 = orePass;
//        Pose goal5 = orePass;
//        Pose goal6 = orePass;
//        Pose goal7 = orePass;
//
//        // Instantiate a simple motion planner
//        var rsp = new ReedsSheppCarPlanner();
//        rsp.setMap(yamlFile);
//        rsp.setRadius(0.01);
//        rsp.setPlanningTimeInSecs(60);
//        rsp.setFootprint(tec.getDefaultFootprint());
//        rsp.setTurningRadius(0.01);
//        rsp.setDistanceBetweenPathPoints(0.1);
//
//        AutonomousVehicle humAbstractVehicle1 = new AutonomousVehicle();
//
//        AutonomousVehicle autAbstractVehicle1 = new AutonomousVehicle();
//        AutonomousVehicle autAbstractVehicle2 = new AutonomousVehicle();
//        AutonomousVehicle autAbstractVehicle3 = new AutonomousVehicle();
//        AutonomousVehicle autAbstractVehicle4 = new AutonomousVehicle();
//        AutonomousVehicle autAbstractVehicle5 = new AutonomousVehicle();
//        AutonomousVehicle autAbstractVehicle6 = new AutonomousVehicle();
//        AutonomousVehicle autAbstractVehicle7 = new AutonomousVehicle();
//
//        tec.setMotionPlanner(humAbstractVehicle1.getID(), rsp);
//
//        tec.setMotionPlanner(autAbstractVehicle1.getID(), rsp);
//        tec.setMotionPlanner(autAbstractVehicle2.getID(), rsp);
//        tec.setMotionPlanner(autAbstractVehicle3.getID(), rsp);
//        tec.setMotionPlanner(autAbstractVehicle4.getID(), rsp);
//        tec.setMotionPlanner(autAbstractVehicle5.getID(), rsp);
//        tec.setMotionPlanner(autAbstractVehicle6.getID(), rsp);
//        tec.setMotionPlanner(autAbstractVehicle7.getID(), rsp);
//
//        tec.setForwardModel(humAbstractVehicle1.getID(), new ConstantAccelerationForwardModel(humAbstractVehicle1.getID(), maxVel, tec.getTemporalResolution(),
//                tec.getControlPeriod(), tec.getRobotTrackingPeriodInMillis(humAbstractVehicle1.getID())));
//
//        tec.setForwardModel(autAbstractVehicle1.getID(), new ConstantAccelerationForwardModel(autAbstractVehicle1.getID(), maxVel, tec.getTemporalResolution(),
//                tec.getControlPeriod(), tec.getRobotTrackingPeriodInMillis(autAbstractVehicle1.getID())));
//        tec.setForwardModel(autAbstractVehicle2.getID(), new ConstantAccelerationForwardModel(autAbstractVehicle2.getID(), maxVel, tec.getTemporalResolution(),
//                tec.getControlPeriod(), tec.getRobotTrackingPeriodInMillis(autAbstractVehicle2.getID())));
//        tec.setForwardModel(autAbstractVehicle3.getID(), new ConstantAccelerationForwardModel(autAbstractVehicle3.getID(), maxVel, tec.getTemporalResolution(),
//                tec.getControlPeriod(), tec.getRobotTrackingPeriodInMillis(autAbstractVehicle3.getID())));
//        tec.setForwardModel(autAbstractVehicle4.getID(), new ConstantAccelerationForwardModel(autAbstractVehicle3.getID(), maxVel, tec.getTemporalResolution(),
//                tec.getControlPeriod(), tec.getRobotTrackingPeriodInMillis(autAbstractVehicle3.getID())));
//        tec.setForwardModel(autAbstractVehicle5.getID(), new ConstantAccelerationForwardModel(autAbstractVehicle3.getID(), maxVel, tec.getTemporalResolution(),
//                tec.getControlPeriod(), tec.getRobotTrackingPeriodInMillis(autAbstractVehicle3.getID())));
//        tec.setForwardModel(autAbstractVehicle6.getID(), new ConstantAccelerationForwardModel(autAbstractVehicle3.getID(), maxVel, tec.getTemporalResolution(),
//                tec.getControlPeriod(), tec.getRobotTrackingPeriodInMillis(autAbstractVehicle3.getID())));
//        tec.setForwardModel(autAbstractVehicle7.getID(), new ConstantAccelerationForwardModel(autAbstractVehicle3.getID(), maxVel, tec.getTemporalResolution(),
//                tec.getControlPeriod(), tec.getRobotTrackingPeriodInMillis(autAbstractVehicle3.getID())));
//
//        tec.placeRobot(humAbstractVehicle1.getID(), initial0);
//
//        tec.placeRobot(autAbstractVehicle1.getID(), initial1);
//        tec.placeRobot(autAbstractVehicle2.getID(), initial2);
//        tec.placeRobot(autAbstractVehicle3.getID(), initial3);
//        tec.placeRobot(autAbstractVehicle4.getID(), initial4);
//        tec.placeRobot(autAbstractVehicle5.getID(), initial5);
//        tec.placeRobot(autAbstractVehicle6.getID(), initial6);
//        tec.placeRobot(autAbstractVehicle7.getID(), initial7);
//
//        // Get complete path plan
////        PoseSteering[] path1 = null;
////        rsp.setStart(initial1);
////        rsp.setGoals(goal1);
////        rsp.plan();
////        if (rsp.getPath() == null) throw new Error("No path found.");
////        path1 = rsp.getPath();
//
//        PoseSteering[] path0 = getPath(rsp, initial0, goal0, true);
//
//        PoseSteering[] path1 = getPath(rsp, initial1, goal1, true);
//        PoseSteering[] path2 = getPath(rsp, initial2, goal2, true);
//        PoseSteering[] path3 = getPath(rsp, initial3, goal3, true);
//        PoseSteering[] path4 = getPath(rsp, initial4, goal4, true);
//        PoseSteering[] path5 = getPath(rsp, initial5, goal5, true);
//        PoseSteering[] path6 = getPath(rsp, initial6, goal6, true);
//        PoseSteering[] path7 = getPath(rsp, initial7, goal7, true);
//
//        // Get first look ahead path plan FIXME Update the human vehicle to not pass through other vehicles
//        var initialPath0 = getLookAheadPath(lookAheadDistance, path0);
//        var newPath0 = initialPath0;
//        int index0 = initialPath0.length;
//        var pathArray0 = new ArrayList<PoseSteering[]>();
//        pathArray0.add(initialPath0);
//
//        // Calculate updated path plans
//        for (int i = 0; i < 100; i++) {
//            try {
//                newPath0 = getUpdatePath(dx, path0, index0);
//            }
//            catch (ArrayIndexOutOfBoundsException e) {
//                newPath0 = path0;
//                pathArray0.add(newPath0);
//                break;
//            };
//            index0 = newPath0.length;
//            pathArray0.add(newPath0);
//        }
//
////        System.out.println("Press a key");
////        System.in.read();
//        var m0 = new Mission(humAbstractVehicle1.getID(), path0);
//
//        var m1 = new Mission(autAbstractVehicle1.getID(), path1);
//        var m2 = new Mission(autAbstractVehicle2.getID(), path2);
//        var m3 = new Mission(autAbstractVehicle3.getID(), path3);
//        var m4 = new Mission(autAbstractVehicle4.getID(), path4);
//        var m5 = new Mission(autAbstractVehicle5.getID(), path5);
//        var m6 = new Mission(autAbstractVehicle6.getID(), path6);
//        var m7 = new Mission(autAbstractVehicle7.getID(), path7);
//
//        Missions.enqueueMission(m0);
//
//        Missions.enqueueMission(m1);
//        Missions.enqueueMission(m2);
//        Missions.enqueueMission(m3);
//        Missions.enqueueMission(m4);
//        Missions.enqueueMission(m5);
//        Missions.enqueueMission(m6);
//        Missions.enqueueMission(m7);
//
//        Missions.setMap(yamlFile);
//        Missions.startMissionDispatchers(tec, true, 0, 1, 2, 3, 4, 5, 6, 7);
//
//        for (int i = 0; i < pathArray0.size(); i++) {
//            Thread.sleep(dt);
//            tec.replacePath(humAbstractVehicle1.getID(), pathArray0.get(i+1), pathArray0.get(i).length, false, null);
//        }
//
//    }
//    //FIXME Handle Null Point Exception
//    private static PoseSteering[] getPath(ReedsSheppCarPlanner rsp, Pose initial, Pose goal, Boolean inverse) {
//        PoseSteering[] pathFwd = null;
//        PoseSteering[] pathInv = null;
//        PoseSteering[] path = null;
//        rsp.setStart(initial);
//        rsp.setGoals(goal);
//        rsp.plan();
//        if (rsp.getPath() == null) throw new Error("No path found.");
//        pathFwd = rsp.getPath();
//        if (inverse == true) {
//            pathInv = rsp.getPathInv();
//            path = (PoseSteering[]) ArrayUtils.addAll(pathFwd, pathInv);
//        }
//        else {
//            path = pathFwd;
//        }
//        return path;
//    }
//
//    // Get updated path for a human driven vehicle
//    private static PoseSteering[] getUpdatePath(double dX, PoseSteering[] path, int index) {
//        double distance;
//        for (int i = 1; i < path.length; i++) {
//            distance = path[index].getPose().distanceTo(path[i+index].getPose());
//            if (distance >= dX) {
//                index += i;
//                break;
//            }
//        }
//        var newPath = new PoseSteering[index];
//        System.arraycopy(path, 0, newPath, 0, index);
//        return newPath;
//    }
//
//    // Get initial look ahead path index for a human driven vehicle
//    private static PoseSteering[] getLookAheadPath(double lookAheadDistance, PoseSteering[] path) {
//        double distance;
//        int replaceIndex = path.length;
//        for (int i = 1; i < path.length; i++) {
//            distance = path[0].getPose().distanceTo(path[i].getPose());
//            if (distance >= lookAheadDistance) {
//                replaceIndex = i;
//                break;
//            }
//        }
//        var newPath = new PoseSteering[replaceIndex];
//        System.arraycopy(path, 0, newPath, 0, replaceIndex);
//        return newPath;
//    }
//
//}
