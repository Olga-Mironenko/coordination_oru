package se.oru.coordination.coordination_oru.code;

import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.PoseSteering;

import java.awt.*;

public class BarrierPhantomVehicle extends AbstractVehicle {
    public boolean isActive = true;

    public BarrierPhantomVehicle(AbstractVehicle vehicleTemplate) {
        super(vehicleNumber, 0, new Color(0, 0, 0, 0), new Color(0, 0, 0, 0),
                0, 0, vehicleTemplate.getXLength(), vehicleTemplate.getYLength());
    }

    @Override
    public PoseSteering[] getPlan(Pose initial, Pose[] goals, String map, Boolean inversePath) {
        return new PoseSteering[0];
    }
}
