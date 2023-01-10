package se.oru.coordination.coordination_oru.code;

import java.awt.*;
import java.util.Arrays;

import com.vividsolutions.jts.geom.Coordinate;
import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.PoseSteering;

public abstract class AbstractVehicle {

    private final int ID;
    private final int priorityID;
    private final Color color; // FIXME Need to check in BrowserVisualization. Create Enum and check if it works.
    private final double maxVelocity;
    private final double maxAcceleration;
    private final String map;
    private final double xLength;
    private final double yLength;
    private final Coordinate[] footPrint;

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
    }
    public AbstractVehicle() {
        this.ID = 1;
        this.priorityID = 0;
        this.color = Color.GREEN;
        this.maxVelocity = 5;
        this.maxAcceleration = 2;
        this.map = null;
        this.xLength = 0.5;
        this.yLength = 0.5;
        this.footPrint = new Coordinate[]{
                new Coordinate(-xLength, yLength),        //back left
                new Coordinate(xLength, yLength),         //back right
                new Coordinate(xLength, -yLength),        //front right
                new Coordinate(-xLength, -yLength)        //front left
        };
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

    public String colorToHEX() {
        return "#" + String.format("%06X", 0xFFFFFF & color.getRGB());
    }
    public abstract PoseSteering[] getPath(Pose initial, Pose goal, String map, Boolean inversePath);

    public int getID() {
        return ID;
    }
    public Coordinate[] getFootPrint() {
        return footPrint;
    }

}
