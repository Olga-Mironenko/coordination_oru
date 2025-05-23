package se.oru.coordination.coordination_oru.code;

import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.Geometry;
import org.apache.commons.lang.ArrayUtils;
import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.PoseSteering;
import se.oru.coordination.coordination_oru.motionplanning.ompl.ReedsSheppCarPlanner;
import se.oru.coordination.coordination_oru.simulation2D.TrajectoryEnvelopeCoordinatorSimulation;
import se.oru.coordination.coordination_oru.util.DynamicMap;
import se.oru.coordination.coordination_oru.util.Missions;
import se.oru.coordination.coordination_oru.util.NoPathFoundError;

import java.awt.*;
import java.io.File;
import java.io.IOException;
import java.security.MessageDigest;
import java.security.NoSuchAlgorithmException;
import java.util.ArrayList;
import java.util.List;

public class AutonomousVehicle extends AbstractVehicle {
    public static boolean isPathCachingEnabled = false;
    public static int numIterationsRoadmapConstruction = 1000;
    public static int numIterationsPathSimplification = 500;
    public static ReedsSheppCarPlanner.PLANNING_ALGORITHM planningAlgorithm = ReedsSheppCarPlanner.PLANNING_ALGORITHM.RRTConnect;
    private ReedsSheppCarPlanner rsp;

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

    public ReedsSheppCarPlanner makePlanner(String mapId, Coordinate[] footprint) {
        if (rsp == null) {
            rsp = new ReedsSheppCarPlanner(planningAlgorithm);
            double resolution = Missions.getDynamicMap().resolution;
            rsp.setRadius(2.5 * resolution); // the greater the value, the less is the number of robot circles
            rsp.setFootprint(footprint);
            rsp.setTurningRadius(0.1 * resolution); // TODO: this should be a vehicle attribute (and "1" is a more natural value)
            rsp.setDistanceBetweenPathPoints(resolution);

            if (planningAlgorithm != ReedsSheppCarPlanner.PLANNING_ALGORITHM.PRMcustom) {
                rsp.setPlanningTimeInSecs(15);
            } else {
                rsp.setNumIterationsRoadmapConstruction(numIterationsRoadmapConstruction);
                rsp.setNumIterationsPathSimplification(numIterationsPathSimplification);
                rsp.setMapId(mapId);

                setMapForPlanner(Missions.getDynamicMap().cloneWithoutObstacles());
                rsp.setStart(new Pose(-1, -1, -1));
                rsp.setGoals(new Pose(-1, -1, -1));
                rsp.plan();
            }

            setMapForPlanner(Missions.getDynamicMap());
        }
        return rsp;
    }

    public void setMapForPlanner(DynamicMap dynamicMap) {
        assert rsp != null;
        rsp.setMap(dynamicMap);
    }

    @Override
    public void getPlan(Pose initial, Pose[] goals, String mapId, Boolean inversePath) {
        getPlan(initial, goals, mapId, inversePath, new int[0]);
    }

    private static String geometriesToHash(Geometry[] geometries) {
        MessageDigest digest;
        try {
            digest = MessageDigest.getInstance("MD5");
        } catch (NoSuchAlgorithmException e) {
            throw new RuntimeException(e);
        }

        for (Object obj : geometries) {
            int hashCode = obj.hashCode();
            byte[] bytes = new byte[] {
                    (byte) (hashCode >> 24),
                    (byte) (hashCode >> 16),
                    (byte) (hashCode >> 8),
                    (byte) hashCode
            };
            digest.update(bytes);
        }

        byte[] hashBytes = digest.digest();
        StringBuilder hexString = new StringBuilder();
        for (int i = 0; i < 4; i++) {
            hexString.append(String.format("%02X", hashBytes[i]));
        }

        return hexString.toString();
    }

    private void savePathIfNeeded(String filenameCache, PoseSteering[] path) {
        if (filenameCache != null) {
            assert ! new File(filenameCache).isFile();
            try {
                Missions.savePathToFile(path, filenameCache);
            } catch (IOException e) {
                throw new RuntimeException(e);
            }
        }
    }

    public void getPlan(Pose initial, Pose[] goals, String mapId, Boolean inversePath, int[] robotIDsObstacles) {
        ArrayList<PoseSteering> finalPath = new ArrayList<PoseSteering>();
        for (int i = 0; i < goals.length; i++) {
            Pose start_ = i == 0 ? initial : goals[i - 1];
            Pose goal_ = goals[i];

            int[] obstacles = i == 0 ? robotIDsObstacles : new int[0]; // TODO: do this only for rerouting at slow?
            PoseSteering[] path = findPlanToGoal(start_, goal_, mapId, inversePath, obstacles);

            for (int j = 0; j < path.length; j++) {
                if (i > 0 && j == 0) {
                    continue;
                }
                finalPath.add(path[j]);
            }
        }

        VehiclesHashMap.getVehicle(this.getID()).setPath(finalPath.toArray(new PoseSteering[finalPath.size()]));
    }

    public PoseSteering[] findPlanToGoal(Pose initial, Pose goal, String mapId, Boolean inversePath, int[] robotIDsObstacles) {
        String filenameCache = null;
        PoseSteering[] path = null;

        Geometry[] obstacles = null;
        if (robotIDsObstacles.length > 0) {
            obstacles = TrajectoryEnvelopeCoordinatorSimulation.tec.getObstacles(false, robotIDsObstacles);
        }

        if (isPathCachingEnabled) {
            StringBuilder base = new StringBuilder(poseToString(initial));
            base.append("_").append(poseToString(goal));
            if (robotIDsObstacles.length > 0) {
                base.append("_obs").append(geometriesToHash(obstacles));
            }
            base.append("_").append(planningAlgorithm).append(inversePath ? "_inv" : "");

            filenameCache = "paths/" + mapId + "/" + base + ".path";
            if (new File(filenameCache).isFile()) {
                path = Missions.loadPathFromFile(filenameCache);
                if (path == null) {
                    throw new NoPathFoundError();
                }
            }
        }

        if (path == null) { // not restored from cache
            var rsp = makePlanner(mapId, getFootprint());
            TrajectoryEnvelopeCoordinatorSimulation.tec.setMotionPlanner(this.getID(), rsp);

            if (robotIDsObstacles.length > 0) {
                rsp.addObstacles(obstacles);
            }

            boolean areSides = rsp.doesIntersectWithObstacle(rsp.getFootprintInPose(initial));
            boolean isFound = false;
            /*
             8 5 9 (dy=1)
             2 1 3 (dy=0)
             5 4 6 (dy=-1)
             ^
             (dx=-1)
             */
            for (int dy : ! areSides ? java.util.List.of(0) : java.util.List.of(0, -1, 1)) {
                for (int dx : ! areSides ? java.util.List.of(0) : List.of(0, -1, 1)) {
                    Pose start = new Pose(initial.getX() + dx, initial.getY() + dy, initial.getTheta());
                    rsp.setStart(start);
                    rsp.setGoals(goal);
                    if (rsp.plan()) {
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
                savePathIfNeeded(filenameCache, null);
                throw new NoPathFoundError();
            }

            PoseSteering[] pathFwd = rsp.getPath();
            PoseSteering[] pathInv;
            if (inversePath) {
                pathInv = rsp.getPathInverseWithoutFirstPose();
                path = (PoseSteering[]) ArrayUtils.addAll(pathFwd, pathInv);
            } else {
                path = pathFwd;
            }
            savePathIfNeeded(filenameCache, path);
        }

        return path;
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
            pathInv = rsp.getPathInverseWithoutFirstPose();
            path = (PoseSteering[]) ArrayUtils.addAll(pathFwd, pathInv);
        }
        else {
            path = pathFwd;
        }
        VehiclesHashMap.getVehicle(this.getID()).setPath(path);
        return path;
    }

}
