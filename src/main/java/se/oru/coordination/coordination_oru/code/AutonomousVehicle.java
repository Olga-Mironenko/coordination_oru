package se.oru.coordination.coordination_oru.code;

import org.apache.commons.lang.ArrayUtils;
import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.PoseSteering;
import se.oru.coordination.coordination_oru.motionplanning.ompl.ReedsSheppCarPlanner;

import java.awt.*;

public class AutonomousVehicle extends AbstractVehicle {

    public AutonomousVehicle(String map) {
        super(1, "AutonomousVehicle", Color.YELLOW, Color.YELLOW, 5, 2, map, 0.5, 0.5);

    }

    public AutonomousVehicle(int id, int priorityID, String type, Color colorMoving, Color colorStill, double maxVelocity, double maxAcceleration, String map, double xLength, double yLength) {
        super(id, priorityID, type, colorMoving, colorStill, maxVelocity, maxAcceleration, map, xLength, yLength);
    }

    public AutonomousVehicle(int priorityID, String type, Color colorMoving, Color colorStill, double maxVelocity, double maxAcceleration, String map, double xLength, double yLength) {
        super(priorityID, type, colorMoving, colorStill, maxVelocity, maxAcceleration, map, xLength, yLength);
    }

    @Override
    public PoseSteering[] getPlan(Pose initial, Pose[] goals, Boolean inversePath) {
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

}
