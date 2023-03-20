package se.oru.coordination.coordination_oru.code;

import java.awt.Color;

// TODO: Move related code from MissionUtils here.
import se.oru.coordination.coordination_oru.util.MissionUtils;

public class HumanDrivenVehicle extends AutonomousVehicle {
    public HumanDrivenVehicle(int priorityID, Color colorMoving, Color colorStill, double maxVelocity, double maxAcceleration, double xLength, double yLength) {
        super(MissionUtils.idHuman, priorityID, colorMoving, colorStill, maxVelocity, maxAcceleration, xLength, yLength);
    }
}