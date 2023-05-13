package se.oru.coordination.coordination_oru.vehicles;

import org.apache.commons.lang.ArrayUtils;
import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.PoseSteering;
import se.oru.coordination.coordination_oru.utility.RobotReport;
import se.oru.coordination.coordination_oru.coordinator.TrajectoryEnvelopeCoordinator;
import se.oru.coordination.coordination_oru.motionplanner.ompl.ReedsSheppCarPlanner;

import java.awt.*;
import java.util.Arrays;

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

    public static void updateLookAheadVehiclesPath(TrajectoryEnvelopeCoordinator tec) {
        for (int robotID : tec.getAllRobotIDs()) {
            if (VehiclesHashMap.getVehicle(robotID).getType().equals("LookAheadVehicle")) {
                var lookAheadVehicle = (LookAheadVehicle) VehiclesHashMap.getVehicle(robotID);
                PoseSteering[] newPath = lookAheadVehicle.getLimitedPath(robotID, lookAheadVehicle.getPredictableDistance(), tec);
                tec.replacePath(robotID, newPath, 0, false, null);
            }
        }
    }

    @Override
    public void getPlan(Pose initial, Pose[] goals, String map, Boolean inversePath) {

        var rsp = new ReedsSheppCarPlanner();
        rsp.setMap(map);
        rsp.setRadius(0.01);
        rsp.setPlanningTimeInSecs(60);
        rsp.setFootprint(super.getFootPrint());
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
        } else {
            path = pathFwd;
        }
        VehiclesHashMap.getVehicle(this.getID()).setPath(path);
    }

    public void getPlan(Pose initial, Pose[] goals, String map, Boolean inversePath, ReedsSheppCarPlanner.PLANNING_ALGORITHM planningAlgorithm,
                        double radius, double planningTime, double turningRadius, double distanceBetweenPathPoints) {

        var rsp = new ReedsSheppCarPlanner(planningAlgorithm);
        rsp.setMap(map);
        rsp.setRadius(radius);
        rsp.setPlanningTimeInSecs(planningTime);
        rsp.setFootprint(super.getFootPrint());
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
        } else {
            path = pathFwd;
        }
        VehiclesHashMap.getVehicle(this.getID()).setPath(path);
    }

    // TODO Delay Time for Human Vehicles not working
    // FIXME Does not avoid collisions
    // TODO Remove previous path of human vehicle
    // TODO Remove path form the end of simulation
    public PoseSteering[] getLimitedPath(int robotID, double predictableDistance, TrajectoryEnvelopeCoordinator tec) {
        double distance = 0.0;
        RobotReport robotReport = tec.getRobotReport(robotID);
        PoseSteering[] fullPath = getPath();

        if (robotReport == null) {
            // Handle the case when the RobotReport is null, e.g., return an empty path or throw a custom exception
            System.err.println("Error: RobotReport for robotID " + robotID + " not found.");
            return new PoseSteering[0];
        }

        double currentDistance = robotReport.getDistanceTraveled();
        double totalDistance = getCycleDistance();
        int pathIndex = Math.max(robotReport.getPathIndex(), 0); // Avoid null point exception for starting

        for (; pathIndex < fullPath.length - 1 && distance <= predictableDistance; pathIndex++) {
            if ((currentDistance + predictableDistance) >= totalDistance) {
                return fullPath; // For last iteration less than predictable distance
            } else {
                distance += fullPath[pathIndex].getPose().distanceTo(fullPath[pathIndex + 1].getPose());
            }
        }

        // Create a new path with the limited length
        return Arrays.copyOfRange(fullPath, 0, pathIndex);
    }


    public double getPredictableDistance() {
        return predictableDistance;
    }

}

