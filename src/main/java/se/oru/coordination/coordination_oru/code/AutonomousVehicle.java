package se.oru.coordination.coordination_oru.code;

import org.apache.commons.lang.ArrayUtils;
import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.PoseSteering;
import se.oru.coordination.coordination_oru.motionplanning.ompl.ReedsSheppCarPlanner;

import java.awt.*;

public class AutonomousVehicle extends AbstractVehicle {
    public AutonomousVehicle(int ID, int priorityID, Color color, double maxVelocity, double maxAcceleration, String map, double xLength, double yLength) {
        super(ID, priorityID, color, maxVelocity, maxAcceleration, map, xLength, yLength);
    }

    @Override
    public PoseSteering[] getPath(Pose initial, Pose goal, String map, Boolean inversePath) {

        // Instantiate a simple motion planner
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
        if (rsp.getPath() == null) throw new Error("No path found."); // FIXME No path find exception
        pathFwd = rsp.getPath();
        if (inversePath) {
            pathInv = rsp.getPathInv();
            path = (PoseSteering[]) ArrayUtils.addAll(pathFwd, pathInv);
        }
        else {
            path = pathFwd;
        }
        return path;
    }
}
