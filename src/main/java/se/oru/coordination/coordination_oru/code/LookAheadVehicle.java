package se.oru.coordination.coordination_oru.code;

import org.apache.commons.lang.ArrayUtils;
import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.PoseSteering;
import se.oru.coordination.coordination_oru.TrajectoryEnvelopeCoordinator;
import se.oru.coordination.coordination_oru.motionplanning.ompl.ReedsSheppCarPlanner;

import java.awt.*;

public class LookAheadVehicle extends AbstractVehicle{

    private final double predictableDistance;

    public LookAheadVehicle(String map, double predictableDistance) {
        super(1, "LookAheadVehicle", Color.GREEN, 5, 2, map, 0.5, 0.5);
        this.predictableDistance = predictableDistance;
    }

    public LookAheadVehicle(int priorityID, String type, double predictableDistance, Color color, double maxVelocity, double maxAcceleration, String map, double xLength, double yLength) {
        super(priorityID, type, color, maxVelocity, maxAcceleration, map, xLength, yLength);
        this.predictableDistance = predictableDistance;
    }

    @Override
    public PoseSteering[] getPlan(Pose initial, Pose[] goals, String map, Boolean inversePath) {

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
        }
        else {
            path = pathFwd;
        }
        VehiclesHashMap.getVehicle(this.getID()).setPath(path);
        return path;
    }

    //FIXME Probably do the updated robot state. Does not avoid collisions
    // TODO Remove previous path of human vehicle
    public PoseSteering[] getLimitedPath(int robotID, double predictableDistance, TrajectoryEnvelopeCoordinator tec) {
        synchronized (tec) {
        double distance = 0.0;
        double currentDistance = tec.getRobotReport(robotID).getDistanceTraveled();
        double totalDistance = this.getCycleDistance();
        int index = Math.max(tec.getRobotReport(robotID).getPathIndex(), 0); // Avoid null point exception for starting
        while (distance <= predictableDistance) {
            if ((currentDistance + predictableDistance) >= totalDistance)  {
                return this.getPath(); // For last iteration less than predictable distance
            }
            else {
                distance += this.getPath()[index].getPose().distanceTo(this.getPath()[index+1].getPose());
                index++;
            }
        }
        var newPath = new PoseSteering[index];
        System.arraycopy(this.getPath(), 0, newPath, 0, index);
        return newPath;
        }
    }
    public double getPredictableDistance() {
        return predictableDistance;
    }

}

