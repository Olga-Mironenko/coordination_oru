package se.oru.coordination.coordination_oru.code;

import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.Geometry;
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
import java.util.List;

public class AutonomousVehicle extends AbstractVehicle {
    public static boolean isPathCachingEnabled = false;
    public static ReedsSheppCarPlanner.PLANNING_ALGORITHM planningAlgorithm = ReedsSheppCarPlanner.PLANNING_ALGORITHM.RRTConnect;
    private ReedsSheppCarPlanner planner;

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

    public ReedsSheppCarPlanner makePlanner(String map, Coordinate[] footprint) {
//        if (true) {
        if (planner == null) {
            var rsp = new ReedsSheppCarPlanner(planningAlgorithm);
            rsp.setMap(map);
            rsp.setRadius(0.01);
            rsp.setPlanningTimeInSecs(60);
            rsp.setFootprint(footprint);
            rsp.setTurningRadius(0.01);
            rsp.setDistanceBetweenPathPoints(0.1);
            planner = rsp;
        }
        return planner;
    }

    @Override
    public void getPlan(Pose initial, Pose[] goals, String map, Boolean inversePath) {
        getPlan(initial, goals, map, inversePath, new int[0]);
    }

    public void getPlan(Pose initial, Pose[] goals, String map, Boolean inversePath, int[] robotIDsObstacles) {
        String filenameCache = null;
        PoseSteering[] path = null;

        String base = poseToString(initial);
        for (Pose goal : goals) {
            base += "_" + poseToString(goal);
        }
        base += "_" + planningAlgorithm + (inversePath ? "_inv" : "");

        if (isPathCachingEnabled && robotIDsObstacles.length == 0) {
            filenameCache = "paths/" + FilenameUtils.getBaseName(map) + "/" + base + ".path";
            if (new File(filenameCache).isFile()) {
                path = Missions.loadPathFromFile(filenameCache);
            }
        }

        if (path == null) {
            var rsp = makePlanner(map, getFootprint());
            TrajectoryEnvelopeCoordinatorSimulation.tec.setMotionPlanner(this.getID(), rsp);

            if (robotIDsObstacles.length > 0) {
                Geometry[] obstacles = TrajectoryEnvelopeCoordinatorSimulation.tec.getObstaclesInCriticalPoints(robotIDsObstacles);
                rsp.addObstacles(obstacles);
            }

            boolean isFound = false;
            for (int dy : java.util.List.of(0, -1, 1)) {
                for (int dx : List.of(0, -1, 1)) {
                    Pose start = new Pose(initial.getX() + dx, initial.getY() + dy, initial.getTheta());
                    rsp.setStart(start);
                    rsp.setGoals(goals);
                    rsp.plan();
                    if (rsp.getPath() != null) {
                        isFound = true;
                        break;
                    }
                }
                if (isFound) {
                    break;
                }
            }
            if (robotIDsObstacles.length > 0) {
                rsp.clearObstacles();
            }
            if (! isFound) {
                throw new NoPathFoundError();
            }

            PoseSteering[] pathFwd = rsp.getPath();
            PoseSteering[] pathInv;
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
