package se.oru.coordination.coordination_oru.code;

import java.awt.Color;

// TODO: Move related code from MissionUtils here.
import se.oru.coordination.coordination_oru.util.MissionUtils;

public class HumanDrivenVehicle extends AutonomousVehicle {
    public HumanDrivenVehicle(int priorityID, String type, Color colorMoving, Color colorStill, double maxVelocity, double maxAcceleration, String map, double xLength, double yLength) {
        super(MissionUtils.idHuman, priorityID, type, colorMoving, colorStill, maxVelocity, maxAcceleration, map, xLength, yLength);
    }
}