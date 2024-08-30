package se.oru.coordination.coordination_oru.code;

import java.awt.Color;

// TODO: Move related code from MissionUtils here.
import se.oru.coordination.coordination_oru.util.HumanControl;

public class HumanDrivenVehicle extends AutonomousVehicle {
    public HumanDrivenVehicle(int id, int priorityID, Color colorMoving, Color colorStill, double maxVelocity, double maxAcceleration, double xLength, double yLength) {
        super(id, priorityID, colorMoving, colorStill, maxVelocity, maxAcceleration, xLength, yLength);
    }

    public HumanDrivenVehicle(int id, int priorityID, Color colorMoving, Color colorStill, double maxVelocity, double maxAcceleration) {
        this(id, priorityID, colorMoving, colorStill, maxVelocity, maxAcceleration, 0, 0);
    }
}