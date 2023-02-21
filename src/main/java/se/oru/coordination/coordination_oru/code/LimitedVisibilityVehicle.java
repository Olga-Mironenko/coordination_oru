package se.oru.coordination.coordination_oru.code;

import java.awt.Color;

import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.PoseSteering;

public class LimitedVisibilityVehicle extends AbstractVehicle{
    public LimitedVisibilityVehicle(int ID, int priorityID, Color colorMoving, Color colorStill, double maxVelocity, double maxAcceleration, String map, double xLength, double yLength) {
        super(ID, priorityID, colorMoving, colorStill, maxVelocity, maxAcceleration, map, xLength, yLength);
    }

    @Override
    public PoseSteering[] getPath(Pose initial, Pose goal, Boolean inversePath) {
        return new PoseSteering[0];
    }
}
