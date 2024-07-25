package se.oru.coordination.coordination_oru.code;

import com.vividsolutions.jts.geom.Coordinate;
import org.apache.commons.io.FilenameUtils;
import org.apache.commons.lang.ArrayUtils;
import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.PoseSteering;
import se.oru.coordination.coordination_oru.motionplanning.ompl.ReedsSheppCarPlanner;
import se.oru.coordination.coordination_oru.simulation2D.TrajectoryEnvelopeCoordinatorSimulation;
import se.oru.coordination.coordination_oru.util.Missions;
import se.oru.coordination.coordination_oru.util.NoPathFoundError;

import java.awt.*;
import java.io.File;
import java.io.IOException;

public class AutonomousVehicle extends AbstractVehicle {
    public static boolean isPathCachingEnabled = true;
    public static ReedsSheppCarPlanner.PLANNING_ALGORITHM planningAlgorithm = ReedsSheppCarPlanner.PLANNING_ALGORITHM.RRTConnect;

    public AutonomousVehicle(int id, int priorityID, Color color, Color colorInMotion, double maxVelocity, double maxAcceleration, double xLength, double yLength) {
        super(id, priorityID, color, colorInMotion, maxVelocity, maxAcceleration, xLength, yLength);
    }

    public AutonomousVehicle(int id, int priorityID, Color color, Color colorInMotion, double maxVelocity, double maxAcceleration) {
        this(id, priorityID, color, colorInMotion, maxVelocity, maxAcceleration, 0, 0);
    }

    public AutonomousVehicle(int priorityID, Color color, Color colorInMotion, double maxVelocity, double maxAcceleration, double xLength, double yLength) {
        super(vehicleNumber, priorityID, color, colorInMotion, maxVelocity, maxAcceleration, xLength, yLength);
    }

    public AutonomousVehicle(int priorityID, Color color, double maxVelocity, double maxAcceleration, double xLength, double yLength) {
        super(vehicleNumber, priorityID, color, null, maxVelocity, maxAcceleration, xLength, yLength);
    }

    public AutonomousVehicle() {
        super(vehicleNumber, 1, Color.YELLOW, null, 5, 2, 0.5, 0.5);
    }

    public static ReedsSheppCarPlanner makePlanner(String map, Coordinate[] footprint) {
        var rsp = new ReedsSheppCarPlanner(planningAlgorithm);
        rsp.setMap(map);
        rsp.setRadius(0.01);
        rsp.setPlanningTimeInSecs(60);
        rsp.setFootprint(footprint);
        rsp.setTurningRadius(0.01);
        rsp.setDistanceBetweenPathPoints(0.1);
        return rsp;
    }

    @Override
    public void getPlan(Pose initial, Pose[] goals, String map, Boolean inversePath) {
        String filenameCache = null;
        PoseSteering[] path = null;

        String base = poseToString(initial);
        for (Pose goal : goals) {
            base += "_" + poseToString(goal);
        }
        base += "_" + planningAlgorithm + (inversePath ? "_inv" : "");

        if (isPathCachingEnabled) {
            filenameCache = "paths/" + FilenameUtils.getBaseName(map) + "/" + base + ".path";
            if (new File(filenameCache).isFile()) {
                path = Missions.loadPathFromFile(filenameCache);
            }
        }

        if (path == null) {
            var rsp = makePlanner(map, getFootprint());
            TrajectoryEnvelopeCoordinatorSimulation.tec.setMotionPlanner(this.getID(), rsp);
//
//            HashSet<Integer> otherRobotIDs = new HashSet<Integer>();
//            for (int otherRobotID : VehiclesHashMap.getList().keySet()) {
//                if (otherRobotID != getID()) {
//                    otherRobotIDs.add(otherRobotID);
//                }
//            }
//            if (!otherRobotIDs.isEmpty()) {
//                TrajectoryEnvelopeCoordinatorSimulation tec = TrajectoryEnvelopeCoordinatorSimulation.tec;
//                rsp.addObstacles(tec.getObstaclesInCriticalPoints(ArrayUtils.toPrimitive(otherRobotIDs.toArray(new Integer[otherRobotIDs.size()]))));
//            }


            PoseSteering[] pathFwd;
            PoseSteering[] pathInv;
            rsp.setStart(initial);
            rsp.setGoals(goals);
            rsp.plan();
            if (rsp.getPath() == null) throw new NoPathFoundError();
            pathFwd = rsp.getPath();
            if (inversePath) {
                pathInv = rsp.getPathInverseWithoutFirstAndLastPose();
                path = (PoseSteering[]) ArrayUtils.addAll(pathFwd, pathInv);
            } else {
                path = pathFwd;
            }
        }

        VehiclesHashMap.getVehicle(this.getID()).setPath(path);

        if (filenameCache != null && ! new File(filenameCache).isFile()) {
            try {
                Missions.savePathToFile(path, filenameCache);
            } catch (IOException e) {
                throw new RuntimeException(e);
            }
        }
    }

    private static String poseToString(Pose pose) {
        return round3(pose.getX()) + "," + round3(pose.getY()) + "," + round3(pose.getTheta());
    }

    private static double round3(double x) {
        return Math.round(x * 1000) / 1000.0;
    }

    public PoseSteering[] getPlan(Pose initial, Pose[] goals, String map, Boolean inversePath, ReedsSheppCarPlanner.PLANNING_ALGORITHM planningAlgorithm,
                                  double radius, double planningTime, double turningRadius, double distanceBetweenPathPoints) {

        var rsp = new ReedsSheppCarPlanner(planningAlgorithm);
        rsp.setMap(map);
        rsp.setRadius(radius);
        rsp.setPlanningTimeInSecs(planningTime);
        rsp.setFootprint(getFootprint());
        rsp.setTurningRadius(turningRadius);
        rsp.setDistanceBetweenPathPoints(distanceBetweenPathPoints);

        PoseSteering[] pathFwd;
        PoseSteering[] pathInv;
        PoseSteering[] path;
        rsp.setStart(initial);
        rsp.setGoals(goals);
        rsp.plan();
        if (rsp.getPath() == null) throw new Error("No path found.");
        pathFwd = rsp.getPath();
        if (inversePath) {
            pathInv = rsp.getPathInverseWithoutFirstAndLastPose();
            path = (PoseSteering[]) ArrayUtils.addAll(pathFwd, pathInv);
        }
        else {
            path = pathFwd;
        }
        VehiclesHashMap.getVehicle(this.getID()).setPath(path);
        return path;
    }

}
