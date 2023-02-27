//package se.oru.coordination.coordination_oru.scenarios;
//
//import com.vividsolutions.jts.geom.Coordinate;
//import org.apache.commons.lang.ArrayUtils;
//import org.metacsp.multi.spatioTemporal.paths.Pose;
//import org.metacsp.multi.spatioTemporal.paths.PoseSteering;
//import se.oru.coordination.coordination_oru.Mission;
//import se.oru.coordination.coordination_oru.code.AutonomousVehicle;
//import se.oru.coordination.coordination_oru.code.Heuristics;
//import se.oru.coordination.coordination_oru.code.AbstractVehicle;
//import se.oru.coordination.coordination_oru.demo.DemoDescription;
//import se.oru.coordination.coordination_oru.motionplanning.ompl.ReedsSheppCarPlanner;
//import se.oru.coordination.coordination_oru.simulation2D.TrajectoryEnvelopeCoordinatorSimulation;
//import se.oru.coordination.coordination_oru.util.BrowserVisualization;
//import se.oru.coordination.coordination_oru.util.Missions;
//
//import java.io.IOException;
//
//@DemoDescription(desc = "Example of a single human driven and seven autonomous vehicles using heuristics for computation time, " +
//        "navigating in a test mine map.")
//public class test {
//
//    public static void main(String[] args) throws InterruptedException, IOException {
//
//        String YAML_FILE = "maps/mine-map-test.yaml";
//        double maxAccel = 2.0;
//        double maxVel = 6.0;
////        double avgVel = maxVel * 0.75;
////        long dt = 2000;     // In milliseconds
////        double dx = avgVel * dt/1000;
////        double lookAheadDistance = 10.0;
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
//        viz.setMap(YAML_FILE);
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
//        rsp.setMap(YAML_FILE);
//        rsp.setRadius(0.01);
//        rsp.setPlanningTimeInSecs(60);
//        rsp.setFootprint(tec.getDefaultFootprint());
//        rsp.setTurningRadius(0.01);
//        rsp.setDistanceBetweenPathPoints(0.1);
//
//        AbstractVehicle autAbstractVehicle1 = new AutonomousVehicle();
//
////        tec.setMotionPlanner(autVehicle1.getID(), rsp);
//
////        tec.setForwardModel(autVehicle1.getID(), new ConstantAccelerationForwardModel(autVehicle1.getID(), maxVel, tec.getTemporalResolution(),
////                tec.getControlPeriod(), tec.getRobotTrackingPeriodInMillis(autVehicle1.getID())));
//
//        tec.placeRobot(autAbstractVehicle1.getID(), initial1);
//
//        // Get complete path plan
////        PoseSteering[] path1 = null;
////        rsp.setStart(initial1);
////        rsp.setGoals(goal1);
////        rsp.plan();
////        if (rsp.getPath() == null) throw new Error("No path found.");
////        path1 = rsp.getPath();
//
//        PoseSteering[] path1 = getPath(rsp, initial1, goal1, true);
//
//        // Get first look ahead path plan FIXME Update the human vehicle to not pass through other vehicles
////        var initialPath0 = getLookAheadPath(lookAheadDistance, path0);
////        var newPath0 = initialPath0;
////        int index0 = initialPath0.length;
////        var pathArray0 = new ArrayList<PoseSteering[]>();
////        pathArray0.add(initialPath0);
////
////        // Calculate updated path plans
////        for (int i = 0; i < 100; i++) {
////            try {
////                newPath0 = getUpdatePath(dx, path0, index0);
////            }
////            catch (ArrayIndexOutOfBoundsException e) {
////                newPath0 = path0;
////                pathArray0.add(newPath0);
////                break;
////            };
////            index0 = newPath0.length;
////            pathArray0.add(newPath0);
////        }
//
////        System.out.println("Press a key");
////        System.in.read();
//
//        var m1 = new Mission(autAbstractVehicle1.getID(), path1);
//
//        Missions.enqueueMission(m1);
//
//        Missions.setMap(YAML_FILE);
//        Missions.startMissionDispatchers(tec, true, 1);
//
////        for (int i = 0; i < pathArray0.size(); i++) {
////            GatedThread.sleep(dt);
////            tec.replacePath(humAutonomousVehicle1.getID(), pathArray0.get(i+1), pathArray0.get(i).length, false, null);
////        }
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
//            pathFwd = rsp.getPath();
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
