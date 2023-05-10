package se.oru.coordination.coordination_oru.code;

import org.apache.commons.lang.ArrayUtils;
import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.PoseSteering;
import se.oru.coordination.coordination_oru.TrajectoryEnvelopeCoordinator;
import se.oru.coordination.coordination_oru.motionplanning.ompl.ReedsSheppCarPlanner;

import java.awt.*;

public class LookAheadVehicle extends AbstractVehicle {

    private final double predictableDistance;

    public LookAheadVehicle(int id, int priorityID, double predictableDistance, Color color, Color colorInMotion, double maxVelocity, double maxAcceleration, double xLength, double yLength) {
        super(id, priorityID, color, colorInMotion, maxVelocity, maxAcceleration, xLength, yLength);
        this.predictableDistance = predictableDistance;
    }

    public LookAheadVehicle(int priorityID, double predictableDistance, Color color, Color colorInMotion, double maxVelocity, double maxAcceleration, double xLength, double yLength) {
        super(vehicleNumber, priorityID, color, colorInMotion, maxVelocity, maxAcceleration, xLength, yLength);
        this.predictableDistance = predictableDistance;
    }

    public LookAheadVehicle(int priorityID, double predictableDistance, Color color, double maxVelocity, double maxAcceleration, double xLength, double yLength) {
        super(vehicleNumber, priorityID, color, null, maxVelocity, maxAcceleration, xLength, yLength);
        this.predictableDistance = predictableDistance;
    }

    public LookAheadVehicle(double predictableDistance) {
        super(vehicleNumber, 1, Color.GREEN, null, 5, 2, 0.5, 0.5);
        this.predictableDistance = predictableDistance;
    }

    @Override
    public PoseSteering[] getPlan(Pose initial, Pose[] goals, String map, Boolean inversePath) {

        var rsp = new ReedsSheppCarPlanner();
        rsp.setMap(map);
        rsp.setRadius(0.01);
        rsp.setPlanningTimeInSecs(60);
        rsp.setFootprint(super.getFootprint());
        rsp.setTurningRadius(0.01);
        rsp.setDistanceBetweenPathPoints(0.1);

        PoseSteering[] pathFwd;
        PoseSteering[] pathInv;
        PoseSteering[] path;
        rsp.setStart(initial);
        rsp.setGoals(goals);
        rsp.plan();
        if (rsp.getPath() == null) throw new Error("No path found.");
        pathFwd = rsp.getPath();
        if (inversePath) {
            pathInv = rsp.getPathInv();
            path = (PoseSteering[]) ArrayUtils.addAll(pathFwd, pathInv);
        }
        else {
            path = pathFwd;
        }
        VehiclesHashMap.getVehicle(this.getID()).setPath(path);
        return path;
    }

    public PoseSteering[] getPlan(Pose initial, Pose[] goals, String map, Boolean inversePath, ReedsSheppCarPlanner.PLANNING_ALGORITHM planningAlgorithm,
                        double radius, double planningTime, double turningRadius, double distanceBetweenPathPoints) {

        var rsp = new ReedsSheppCarPlanner(planningAlgorithm);
        rsp.setMap(map);
        rsp.setRadius(radius);
        rsp.setPlanningTimeInSecs(planningTime);
        rsp.setFootprint(super.getFootprint());
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
            pathInv = rsp.getPathInv();
            path = (PoseSteering[]) ArrayUtils.addAll(pathFwd, pathInv);
        }
        else {
            path = pathFwd;
        }
        VehiclesHashMap.getVehicle(this.getID()).setPath(path);
        return path;
    }

    public synchronized static void updateLookAheadVehiclesPath(TrajectoryEnvelopeCoordinator tec) {
        for (int robotID : tec.getAllRobotIDs()) {
            if(VehiclesHashMap.getVehicle(robotID).getType().equals("LookAheadVehicle")) {
                var lookAheadVehicle = (LookAheadVehicle) VehiclesHashMap.getVehicle(robotID);
                PoseSteering[] newPath = lookAheadVehicle.getLimitedPath(robotID, lookAheadVehicle.getPredictableDistance(), tec);
                tec.replacePath(robotID, newPath, 0, false, null);
            }
        }
    }

    // TODO Delay Time for Human Vehicles not working
    // FIXME Does not avoid collisions
    // TODO Remove previous path of human vehicle
    // TODO Remove  h form the end of simulation
    public synchronized PoseSteering[] getLimitedPath(int robotID, double predictableDistance, TrajectoryEnvelopeCoordinator tec) {
        double distance = 0.0;
        double currentDistance = tec.getRobotReport(robotID).getDistanceTraveled();
        double totalDistance = getCycleDistance();
        int pathIndex = Math.max(tec.getRobotReport(robotID).getPathIndex(), 0); // Avoid null point exception for starting

        while (distance <= predictableDistance) {
            if ((currentDistance + predictableDistance) >= totalDistance)  {
                return getPath(); // For last iteration less than predictable distance
            }
            else {
                distance += getPath()[pathIndex].getPose().distanceTo(getPath()[pathIndex+1].getPose());
                pathIndex++;
            }
        }
        var newPath = new PoseSteering[pathIndex];
        System.arraycopy(getPath(), 0, newPath, 0, pathIndex);
        return newPath;
    }
    public double getPredictableDistance() {
        return predictableDistance;
    }

}

