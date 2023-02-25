package se.oru.coordination.coordination_oru.code;

import java.awt.Color;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Arrays;

import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.PoseSteering;

import com.vividsolutions.jts.geom.Coordinate;

import se.oru.coordination.coordination_oru.RobotReport;
import se.oru.coordination.coordination_oru.util.NoPathFound;

public abstract class AbstractVehicle {

    private final int ID;
    private final int priorityID;
    private final Color colorMoving;
    private final Color colorStill;
    private final double maxVelocity;
    private final double maxAcceleration;
    protected final String map;
    private final double xLength;
    private final double yLength;
    private final Coordinate[] footPrint;
    private RobotReport currentRobotReport = new RobotReport(-1, null, -1, 0.0, 0.0, -1);
    private double cycleDistance;
    private double totalDistance;
    private long timeInterval;
    private double averageSpeed;
    private int cycles = -1;
    private int stops = -1;
    private final double startTime = System.nanoTime();
    private long waitingTime = -2;

    public AbstractVehicle(int ID, int priorityID, Color colorMoving, Color colorStill, double maxVelocity, double maxAcceleration, String map, double xLength, double yLength) {
        this.ID = ID;
        this.priorityID = priorityID;
        this.colorMoving = colorMoving;
        this.colorStill = colorStill;
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
                ", color='" + colorMoving + '\'' +
                ", maxVelocity=" + maxVelocity +
                ", maxAcceleration=" + maxAcceleration +
                ", map='" + map + '\'' +
                ", xLength=" + xLength +
                ", yLength=" + yLength +
                ", footPrint=" + Arrays.toString(footPrint) +
                '}';
    }

    public abstract PoseSteering[] getPath(Pose initial, Pose goal, Boolean inversePath) throws NoPathFound;

    public void updateStatistics() {

        // FIXME Need to fix the cycles and stops calculation. Might need to use last Robto Report.
        if (this.currentRobotReport.getPathIndex() == -1) {
            //System.out.println("DONE");
            this.cycles++;
            //System.out.println(cycles);
        }
        if (this.currentRobotReport.getVelocity() == 0.0){
            waitingTime += 2;
            stops++;
        }
        totalDistance = Math.round(((cycleDistance * cycles) +
            currentRobotReport.getDistanceTraveled()) * 10.0) / 10.0;
        timeInterval = Math.round(System.nanoTime() - startTime) / 1000_000_000;
        averageSpeed = Math.round((totalDistance / timeInterval) * 10.0) / 10.0;
    }

    public void writeStatistics() {

        try {
            String line1 = "Vehicle: V" + this.getID() + "  " +  this.getClass().getSimpleName() + "\n";
            String line2 = "---------------------------------" + "\n";
            String line3 = "Cycle distance: " + this.cycleDistance + " m" + "\n";
            String line4 = "No. of completed cycles: " + this.cycles + "\n";
            String line5 = "Total distance travelled: " + totalDistance + " m" + "\n";
            String line6 = "No. of stops: " + this.stops + "\n";
            String line7 = "Total waiting time: " + waitingTime + " s" + "\n";
            String line8 = "Average speed: " + averageSpeed + " m/s" + "\n";
            String line9 = "Total simulation time: " + timeInterval + " s" + "\n";
            String line10 = "\n";

            String fileName = "./src/main/java/se/oru/coordination/coordination_oru/scenarios/results.txt" ;
            File file = new File(fileName);

            // if file doesn't exist, then create it
            if (!file.exists()) {
                file.createNewFile();
            }

            FileWriter fw = new FileWriter(file.getAbsoluteFile(), true);
            BufferedWriter bw = new BufferedWriter(fw);
            bw.write(line1);
            bw.write(line2);
            bw.write(line3);
            bw.write(line4);
            bw.write(line5);
            bw.write(line6);
            bw.write(line7);
            bw.write(line8);
            bw.write(line9);
            bw.write(line10);
            bw.close();

        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public int getID() {
        return ID;
    }

    public String getColorCode() {
        return "#" + String.format("%06x", 0xFFFFFF & getColor().getRGB());
    }

    public Color getColor() {
        return currentRobotReport.getVelocity() > 0.1 ? colorMoving : colorStill;
    }

    public Coordinate[] getFootPrint() {
        return footPrint;
    }

    public void setCurrentRobotReport(RobotReport currentRobotReport) {
        this.currentRobotReport = currentRobotReport;
        updateStatistics();
    }

    public void setCycleDistance(double cycleDistance) {
        this.cycleDistance = cycleDistance;
    }
}
