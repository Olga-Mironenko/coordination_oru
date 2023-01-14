package se.oru.coordination.coordination_oru.code;

import java.awt.*;
import java.util.Arrays;

import com.vividsolutions.jts.geom.Coordinate;
import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.PoseSteering;
import se.oru.coordination.coordination_oru.RobotReport;

public abstract class AbstractVehicle {

    private final int ID;
    private final int priorityID;
    private final Color color;
    private final double maxVelocity;
    private final double maxAcceleration;
    private final String map;
    private final double xLength;
    private final double yLength;
    private final Coordinate[] footPrint;
    private RobotReport currentRobotReport = new RobotReport(-1, null, -1, 0.0, 0.0, -1);
    private RobotReport lastRobotReport = new RobotReport(-1, null, -1, 0.0, 0.0, -1);

    public AbstractVehicle(int ID, int priorityID, Color color, double maxVelocity, double maxAcceleration, String map, double xLength, double yLength) {
        this.ID = ID;
        this.priorityID = priorityID;
        this.color = color;
        this.maxVelocity = maxVelocity;
        this.maxAcceleration = maxAcceleration;
        this.map = map;
        this.xLength = xLength;
        this.yLength = yLength;
        this.footPrint = new Coordinate[]{                // FIXME Currently allows four sided vehicles only
                new Coordinate(-xLength, yLength),        //back left
                new Coordinate(xLength, yLength),         //back right
                new Coordinate(xLength, -yLength),        //front right
                new Coordinate(-xLength, -yLength)        //front left
        };
        VehiclesHashMap.getInstance().getList().put(this.getID(), this);
    }

    @Override
    public String toString() {
        return "AbstractVehicle{" +
                "ID=" + ID +
                ", priorityID=" + priorityID +
                ", color='" + color + '\'' +
                ", maxVelocity=" + maxVelocity +
                ", maxAcceleration=" + maxAcceleration +
                ", map='" + map + '\'' +
                ", xLength=" + xLength +
                ", yLength=" + yLength +
                ", footPrint=" + Arrays.toString(footPrint) +
                '}';
    }

    public abstract PoseSteering[] getPath(Pose initial, Pose goal, String map, Boolean inversePath);

    // TODO calculate all statistics
    public void updateStatistics() {
        System.out.println("Robot" + this.getID() + " "+ this.currentRobotReport);
        System.out.println("Robot" + this.getID() + " "+ this.lastRobotReport);
    }

    // TODO implement writing statistics to a file with scenario name
    public void printStatistics() {

    }
    public int getID() {
        return ID;
    }

    public String getColor() {
        return "#" + String.format("%06x", 0xFFFFFF & color.getRGB());
    }

    public Coordinate[] getFootPrint() {
        return footPrint;
    }

    public void setCurrentRobotReport(RobotReport currentRobotReport) {
        this.setLastRobotReport();
        this.currentRobotReport = currentRobotReport;
        updateStatistics();
    }

    public void setLastRobotReport() {
        this.lastRobotReport = this.currentRobotReport;
    }

}
