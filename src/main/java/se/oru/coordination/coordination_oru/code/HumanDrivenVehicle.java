package se.oru.coordination.coordination_oru.code;

import java.awt.Color;

public class HumanDrivenVehicle extends AutonomousVehicle {
    public HumanDrivenVehicle(int priorityID, String type, Color colorMoving, Color colorStill, double maxVelocity, double maxAcceleration, String map, double xLength, double yLength) {
        super(priorityID, type, colorMoving, colorStill, maxVelocity, maxAcceleration, map, xLength, yLength);
    }
}