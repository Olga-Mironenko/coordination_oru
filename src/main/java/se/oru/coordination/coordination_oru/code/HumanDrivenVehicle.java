package se.oru.coordination.coordination_oru.code;

import java.awt.Color;

public class HumanDrivenVehicle extends AutonomousVehicle {
    public HumanDrivenVehicle(int ID, int priorityID, Color colorMoving, Color colorStill, double maxVelocity, double maxAcceleration,
            String map, double xLength, double yLength) {
        super(ID, priorityID, colorMoving, colorStill, maxVelocity, maxAcceleration, map, xLength, yLength);
    }
}