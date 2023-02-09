package se.oru.coordination.coordination_oru.code;

import java.awt.Color;

import org.apache.commons.lang.ArrayUtils;
import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.PoseSteering;

import se.oru.coordination.coordination_oru.motionplanning.ompl.ReedsSheppCarPlanner;

public class AutonomousVehicle extends AbstractVehicle {

    double cycleDistance;
    int maxPathAttempts = 10;

    public AutonomousVehicle(int ID, int priorityID, Color color, double maxVelocity, double maxAcceleration, String map, double xLength, double yLength) {
        super(ID, priorityID, color, maxVelocity, maxAcceleration, map, xLength, yLength);
    }

    @Override
    public PoseSteering[] getPath(Pose initial, Pose goal, Boolean inversePath) {

        var rsp = new ReedsSheppCarPlanner();
        rsp.setMap(map);
        rsp.setRadius(0.01);
        rsp.setPlanningTimeInSecs(60);
        rsp.setFootprint(super.getFootPrint());
        rsp.setTurningRadius(0.01);
        rsp.setDistanceBetweenPathPoints(0.1);

        rsp.setStart(initial);
        rsp.setGoals(goal);

        PoseSteering[] pathFwd = null;
        for (int i = 1; i <= maxPathAttempts; i++) {
            if (rsp.plan()) {
		pathFwd = rsp.getPath();
                break;
            }
        }
        if (pathFwd == null) throw new Error("No path found.");

        PoseSteering[] path = null;
        if (inversePath) {
            PoseSteering[] pathInv = rsp.getPathInv();
            path = (PoseSteering[]) ArrayUtils.addAll(pathFwd, pathInv);
        }
        else {
            path = pathFwd;
        }

        for (int i = 0; i < path.length-1; i++) {
            double deltaS = path[i].getPose().distanceTo(path[i + 1].getPose());
            cycleDistance += deltaS;
        }
        VehiclesHashMap.getVehicle(super.getID()).setCycleDistance(Math.round(cycleDistance * 100.0)/100.0);
        return path;
    }

    public PoseSteering[] getPath(Pose initial, Pose goal, String map, Boolean inversePath, ReedsSheppCarPlanner.PLANNING_ALGORITHM planningAlgorithm,
                                  double radius, double planningTime, double turningRadius, double distanceBetweenPathPoints) {

        var rsp = new ReedsSheppCarPlanner(planningAlgorithm);
        rsp.setMap(map);
        rsp.setRadius(radius);
        rsp.setPlanningTimeInSecs(planningTime);
        rsp.setFootprint(super.getFootPrint());
        rsp.setTurningRadius(turningRadius);
        rsp.setDistanceBetweenPathPoints(distanceBetweenPathPoints);

        PoseSteering[] pathFwd = null;
        PoseSteering[] pathInv = null;
        PoseSteering[] path = null;
        rsp.setStart(initial);
        rsp.setGoals(goal);
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

        for (int i = 0; i < path.length-1; i++) {
            double deltaS = path[i].getPose().distanceTo(path[i + 1].getPose());
            cycleDistance += deltaS;
        }
        VehiclesHashMap.getVehicle(super.getID()).setCycleDistance(cycleDistance);
        return path;
    }

}
