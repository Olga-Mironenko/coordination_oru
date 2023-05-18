package se.oru.coordination.coordination_oru.robots;

import se.oru.coordination.coordination_oru.utility.MissionUtils;

import java.awt.*;

/**
 * HumanOperatedVehicle is a subclass of AutonomousVehicle, representing a human-operated vehicle in the simulation.
 * This class inherits all attributes and methods of the AutonomousVehicle class.
 * It provides a specific implementation for human-operated vehicles with a unique identifier, priorityID,
 * colorMoving, colorStill, maxVelocity, maxAcceleration, xLength, and yLength parameters.
 *
 * @author omo
 */
public class HumanOperatedRobot extends AutonomousRobot {

    /**
     * Constructs a HumanOperatedVehicle object with the specified parameters.
     * It initializes the vehicle's unique identifier (ID), priorityID, colorMoving, colorStill,
     * maxVelocity, maxAcceleration, xLength, and yLength.
     *
     * @param priorityID      The priority identifier of the vehicle.
     * @param colorMoving     The color of the vehicle when in motion.
     * @param colorStill      The color of the vehicle when stationary.
     * @param maxVelocity     The maximum velocity of the vehicle.
     * @param maxAcceleration The maximum acceleration of the vehicle.
     * @param xLength         The length of the vehicle along the x-axis.
     * @param yLength         The length of the vehicle along the y-axis.
     */
    public HumanOperatedRobot(int priorityID, Color colorMoving, Color colorStill, double maxVelocity, double maxAcceleration, double xLength, double yLength) {
        super(MissionUtils.idHuman, priorityID, colorMoving, colorStill, maxVelocity, maxAcceleration, xLength, yLength);
    }
}
