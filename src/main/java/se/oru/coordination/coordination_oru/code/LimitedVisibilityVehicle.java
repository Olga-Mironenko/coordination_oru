package se.oru.coordination.coordination_oru.code;

import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.PoseSteering;

import java.awt.*;

public class LimitedVisibilityVehicle extends AbstractVehicle{


    public LimitedVisibilityVehicle(int ID, int priorityID, Color color, double maxVelocity, double maxAcceleration, String map, double xLength, double yLength) {
        super(ID, priorityID, color, maxVelocity, maxAcceleration, map, xLength, yLength);
    }

    @Override
    public PoseSteering[] getPath(Pose initial, Pose goal, String map, Boolean inversePath) {
        return new PoseSteering[0];
    }
}
