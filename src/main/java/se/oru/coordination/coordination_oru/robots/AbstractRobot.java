package se.oru.coordination.coordination_oru.robots;

import com.vividsolutions.jts.geom.Coordinate;
import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.PoseSteering;

import java.awt.*;
import java.util.Arrays;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicInteger;

/**
 * AbstractRobot is an abstract class representing a generic robot with common attributes and methods.
 * It should be extended by concrete robot classes with specific implementations.
 * The class keeps track of the robot's ID, priorityID, color,
 * maxVelocity, maxAcceleration, and footprint of the robot.
 *
 * @author anm
 */
public abstract class AbstractRobot {
    public static AtomicInteger robotNumber = new AtomicInteger(1);
    private final int ID;
    private final int priorityID;
    private final String type = getClass().getSimpleName();
    private final double maxVelocity;
    private final double maxAcceleration;
    private final Coordinate[] footPrint;
    private Color color;
    private double planLength;
    private PoseSteering[] path;

    /**
     * Constructs an AbstractRobot object with the specified parameters.
     * It initializes the robot's ID, priorityID, color, maxVelocity, maxAcceleration, and footprint.
     * If a robot with the same ID already exists, an IllegalStateException is thrown.
     *
     * @param ID              The unique identifier of the robot.
     * @param priorityID      The priority identifier of the robot.
     * @param color           The color of the robot when stationary.
     * @param maxVelocity     The maximum velocity of the robot.
     * @param maxAcceleration The maximum acceleration of the robot.
     * @param xLength         The length of the robot along the x-axis.
     * @param yLength         The length of the robot along the y-axis.
     * @throws IllegalStateException if a robot with the same ID already exists.
     */
    public AbstractRobot(int ID, int priorityID, Color color, double maxVelocity, double maxAcceleration, double xLength, double yLength) {
        this.ID = ID;
        this.priorityID = priorityID;
        this.color = color;
        this.maxVelocity = maxVelocity;
        this.maxAcceleration = maxAcceleration;
        this.footPrint = new Coordinate[]{
                new Coordinate(-xLength, yLength),        // back left
                new Coordinate(xLength, yLength),         // back right
                new Coordinate(xLength, -yLength),        // front right
                new Coordinate(-xLength, -yLength)        // front left
        };

        AbstractRobot existingRobot = RobotHashMap.getRobot(ID);
        if (existingRobot != null) {
            throw new IllegalStateException("ID " + ID + " already exists.");
        }

        RobotHashMap.getList().put(this.ID, this);
        robotNumber.incrementAndGet();
    }

    /**
     * Generates the plan for the robot given the initial pose, goal poses, map, and whether the path should be reversed.
     * The subclasses should provide the concrete implementation for this method.
     *
     * @param initial     The initial pose of the robot.
     * @param goals       An array of goal poses.
     * @param map         The map used for planning.
     * @param inversePath A flag indicating if the path should be reversed.
     */
    public abstract void getPlan(Pose initial, Pose[] goals, String map, Boolean inversePath);

    /**
     * Blinks the robot's color between the original color and a specified toggle color for a given duration.
     *
     * @param colorOriginal    The original color of the robot.
     * @param colorToggle      The color to toggle to during blinking.
     * @param blinkTimeSeconds The duration of the blink in seconds.
     * @throws InterruptedException if the sleep operation is interrupted.
     * TODO: Implement color blinking and robot stoppage.
     */
    public void blinkRobot(Color colorOriginal, Color colorToggle, long blinkTimeSeconds) throws InterruptedException {
        RobotHashMap.getRobot(ID).setRobotColor(colorToggle);
        TimeUnit.SECONDS.sleep(blinkTimeSeconds);
        RobotHashMap.getRobot(ID).setRobotColor(colorOriginal);
    }

    public int getID() {
        return ID;
    }

    public void setRobotColor(Color color) {
        this.color = color;
    }

    public String getColorCode() {
        return "#" + String.format("%06x", 0xFFFFFF & getColor().getRGB());
    }

    public Coordinate[] getFootPrint() {
        return footPrint;
    }

    public double getPlanLength() {
        return planLength;
    }

    public void setPlanLength(PoseSteering[] path) {
        for (int i = 0; i < path.length - 1; i++) {
            double deltaS = path[i].getPose().distanceTo(path[i + 1].getPose());
            planLength += deltaS;
        }
        planLength = Math.round(planLength * 10.0) / 10.0;
        RobotHashMap.getRobot(this.getID()).planLength = planLength;
    }

    public PoseSteering[] getPath() {
        return path;
    }

    protected void setPath(PoseSteering[] path) {
        this.path = path;
        setPlanLength(path);
    }

    public double getMaxVelocity() {
        return maxVelocity;
    }

    public double getMaxAcceleration() {
        return maxAcceleration;
    }

    public String getType() {
        return type;
    }

    public Color getColor() {
        return color;
    }

    @Override
    public String toString() {
        return "AbstractRobot{" +
                "ID=" + ID +
                ", priorityID=" + priorityID +
                ", type='" + type + '\'' +
                ", maxVelocity=" + maxVelocity +
                ", maxAcceleration=" + maxAcceleration +
                ", footPrint=" + Arrays.toString(footPrint) +
                ", color=" + color +
                '}';
    }
}
