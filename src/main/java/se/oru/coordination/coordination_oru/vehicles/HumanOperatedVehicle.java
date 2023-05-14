package se.oru.coordination.coordination_oru.vehicles;

import se.oru.coordination.coordination_oru.utility.MissionUtils;

import java.awt.*;

public class HumanDrivenVehicle extends AutonomousVehicle {
    public HumanDrivenVehicle(int priorityID, Color colorMoving, Color colorStill, double maxVelocity, double maxAcceleration, double xLength, double yLength) {
        super(MissionUtils.idHuman, priorityID, colorMoving, colorStill, maxVelocity, maxAcceleration, xLength, yLength);
    }
}