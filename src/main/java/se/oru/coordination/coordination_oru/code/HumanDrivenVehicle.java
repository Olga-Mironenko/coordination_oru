package se.oru.coordination.coordination_oru.code;

import java.awt.Color;

public class HumanDrivenVehicle extends AutonomousVehicle {
    public HumanDrivenVehicle(int ID, int priorityID, Color color, double maxVelocity, double maxAcceleration,
            String map, double xLength, double yLength) {
        super(ID, priorityID, color, maxVelocity, maxAcceleration, map, xLength, yLength);
    }
}