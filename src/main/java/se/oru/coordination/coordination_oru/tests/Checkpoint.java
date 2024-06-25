package se.oru.coordination.coordination_oru.tests;

import se.oru.coordination.coordination_oru.code.AutonomousVehicle;

public class Checkpoint {
    public double coordinate;
    public boolean isX;
    public boolean isIncreasing;

    public Checkpoint(double coordinate, boolean isX, boolean isIncreasing) {
        this.coordinate = coordinate;
        this.isX = isX;
        this.isIncreasing = isIncreasing;
    }

    public boolean isPassed(AutonomousVehicle vehicle) {
        double coordinateLast = isX ? vehicle.lastRobotReport.getX() : vehicle.lastRobotReport.getY();
        double coordinateCurrent = isX ? vehicle.currentRobotReport.getX() : vehicle.currentRobotReport.getY();

        if (isIncreasing) {
            return coordinateLast < coordinate && coordinate <= coordinateCurrent;
        }
        return coordinateLast > coordinate && coordinate >= coordinateCurrent;
    }
}
