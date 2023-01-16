package se.oru.coordination.coordination_oru.code;

import org.apache.commons.lang.ArrayUtils;
import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.PoseSteering;
import se.oru.coordination.coordination_oru.motionplanning.ompl.ReedsSheppCarPlanner;

import java.awt.*;
import java.net.IDN;
import java.text.DecimalFormat;

public class AutonomousVehicle extends AbstractVehicle {

    double cycleDistance;

    public AutonomousVehicle(int ID, int priorityID, Color color, double maxVelocity, double maxAcceleration, String map, double xLength, double yLength) {
        super(ID, priorityID, color, maxVelocity, maxAcceleration, map, xLength, yLength);
    }

    @Override
    public PoseSteering[] getPath(Pose initial, Pose goal, String map, Boolean inversePath) {

        var rsp = new ReedsSheppCarPlanner();
        rsp.setMap(map);
        rsp.setRadius(0.01);
        rsp.setPlanningTimeInSecs(60);
        rsp.setFootprint(super.getFootPrint());
        rsp.setTurningRadius(0.01);
        rsp.setDistanceBetweenPathPoints(0.1);

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
