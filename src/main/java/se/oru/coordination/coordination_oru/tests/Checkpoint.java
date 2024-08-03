package se.oru.coordination.coordination_oru.tests;

import se.oru.coordination.coordination_oru.RobotReport;
import se.oru.coordination.coordination_oru.code.AutonomousVehicle;

public class Checkpoint {
    public double coordinate;
    public boolean isX;
    public boolean isIncreasing;
    public RobotReport previousRobotReport;

    public Checkpoint(double coordinate, boolean isX, boolean isIncreasing) {
        this.coordinate = coordinate;
        this.isX = isX;
        this.isIncreasing = isIncreasing;
    }

    public boolean isPassed(AutonomousVehicle vehicle) {
        if (previousRobotReport == null) {
            previousRobotReport = vehicle.currentRobotReport;
            return false;
        }

        double coordinateLast = isX ? previousRobotReport.getX() : previousRobotReport.getY();
        double coordinateCurrent = isX ? vehicle.currentRobotReport.getX() : vehicle.currentRobotReport.getY();

        previousRobotReport = vehicle.currentRobotReport;

        if (isIncreasing) {
            return coordinateLast < coordinate && coordinate <= coordinateCurrent;
        }
        return coordinateLast > coordinate && coordinate >= coordinateCurrent;
    }
}
