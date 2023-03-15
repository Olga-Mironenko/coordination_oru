package se.oru.coordination.coordination_oru.code;

import java.awt.Color;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Arrays;
import java.util.Collections;
import java.util.concurrent.TimeUnit;

import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.PoseSteering;

import com.vividsolutions.jts.geom.Coordinate;

import se.oru.coordination.coordination_oru.RobotReport;

public abstract class AbstractVehicle {
    private final int id;
    private final int priorityID;
    private final String type;
    private Color colorMoving;
    private Color colorStill;
    private double maxVelocity;
    private double maxAcceleration;
    protected final String map;
    private final double xLength;
    private final double yLength;
    private final Coordinate[] footPrint;
    private RobotReport currentRobotReport = new RobotReport(-1, null, -1, 0.0, 0.0, -1);
    private RobotReport lastRobotReport = new RobotReport(-1, null, -1, 0.0, 0.0, -1);
    private double cycleDistance;
    private double totalDistance;
    private long timeInterval;
    private double averageSpeed;
    private int cycles;
    private int maxWaitingTime;
    private int currentWaitingTime;
    private long totalWaitingTime;
    private int stops;
    private final double startTime = System.nanoTime();
    private PoseSteering[] path;
    public AbstractVehicle(int id, int priorityID, String type, Color colorMoving, Color colorStill, double maxVelocity, double maxAcceleration, String map, double xLength, double yLength) {
        this.id = id;
        this.priorityID = priorityID;
        this.type = this.getClass().getSimpleName();
        this.colorMoving = colorMoving;
        this.colorStill = colorStill;
        this.maxVelocity = maxVelocity;
        this.maxAcceleration = maxAcceleration;
        this.map = map;
        this.xLength = xLength;
        this.yLength = yLength;
        this.footPrint = new Coordinate[] {               // FIXME Currently allows four sided vehicles only
                new Coordinate(-xLength, yLength),        //back left
                new Coordinate(xLength, yLength),         //back right
                new Coordinate(xLength, -yLength),        //front right
                new Coordinate(-xLength, -yLength)        //front left
        };
        if (VehiclesHashMap.getList().containsKey(id)) {
            throw new Error("ID " + id + " already exists.");
        }
        VehiclesHashMap.getList().put(id, this);
    }

    public AbstractVehicle(int priorityID, String type, Color colorMoving, Color colorStill, double maxVelocity, double maxAcceleration, String map, double xLength, double yLength) {
        this(
                VehiclesHashMap.getList().isEmpty() ? 1 : Collections.max(VehiclesHashMap.getList().keySet()) + 1,
                priorityID, type, colorMoving, colorStill, maxVelocity, maxAcceleration, map, xLength, yLength
        );
    }

        @Override
    public String toString() {
        return "AbstractVehicle{" +
                "ID=" + id +
                ", priorityID=" + priorityID +
                ", type='" + type + '\'' +
                ", color=" + colorMoving +
                ", maxVelocity=" + maxVelocity +
                ", maxAcceleration=" + maxAcceleration +
                ", map='" + map + '\'' +
                ", xLength=" + xLength +
                ", yLength=" + yLength +
                ", footPrint=" + Arrays.toString(footPrint) +
                '}';
    }

    public abstract PoseSteering[] getPlan(Pose initial, Pose[] goals, Boolean inversePath);
    public synchronized void updateStatistics() {

        // Loading and unloading times and stoppages are not considered
        if ((this.currentRobotReport.getPathIndex() == -1) && (this.lastRobotReport.getPathIndex() != -1)) this.cycles++;

        if ((this.currentRobotReport.getVelocity() == 0.0) && (this.lastRobotReport.getVelocity() != 0.0)) this.stops++;

        if (this.currentRobotReport.getVelocity() == 0.0) {
            this.currentWaitingTime += 2;
            this.totalWaitingTime += 2;
        }

        if (this.currentRobotReport.getVelocity() != 0.0) {
            this.maxWaitingTime = Math.max(currentWaitingTime, maxWaitingTime);
        }

        this.totalDistance = Math.round(((cycleDistance * cycles) +
            currentRobotReport.getDistanceTraveled()) * 10.0) / 10.0;
        this.timeInterval = Math.round(System.nanoTime() - startTime) / 1000_000_000;
        this.averageSpeed = Math.round((totalDistance / timeInterval) * 10.0) / 10.0;
    }

    // TODO Write the data in a csv file with each cycle results
    // TODO Separate Scenario general settings and robot settings
    // TODO Date, Day, Timing settings. Fix file name
    // FIXME Check statistics

    public void writeStatistics() {

        try {
            String line0 = "=================================" + "\n";
            String line1 = "Map: " + map + "\n";
            String line2 = "Vehicle: V" + this.getID() + "  " +  this.getClass().getSimpleName() + "\n";
            String line3 = "---------------------------------" + "\n";
            String line4 = "Cycle distance: " + this.cycleDistance + " m" + "\n";
            String line5 = "No. of completed cycles: " + this.cycles + "\n";
            String line6 = "Total distance travelled: " + totalDistance + " m" + "\n";
            String line7 = "No. of stops: " + this.stops + "\n";
            String line8 = "Total waiting time: " + totalWaitingTime + " s" + "\n";
            String line9 = "Maximum waiting time: " + maxWaitingTime + " s" + "\n";
            String line10 = "Maximum acceleration: " + maxAcceleration + " m/s^2" + "\n";
            String line11 = "Maximum speed: " + maxVelocity + " m/s" + "\n";
            String line12 = "Average speed: " + averageSpeed + " m/s" + "\n";
            String line13 = "Total simulation time: " + timeInterval + " s" + "\n";
            String line14 = "\n";

            String fileName = "./src/main/java/se/oru/coordination/coordination_oru/scenarios/results.txt" ;
            File file = new File(fileName);

            // if file doesn't exist, then create it
            if (!file.exists()) {
                file.createNewFile();
            }

            //FIXME Use a loop for ease
            FileWriter fw = new FileWriter(file.getAbsoluteFile(), true);
            BufferedWriter bw = new BufferedWriter(fw);
            bw.write(line0);
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
            bw.write(line11);
            bw.write(line12);
            bw.write(line13);
            bw.write(line14);
            bw.close();

        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    //TODO Do color blinking and vehicle stoppage
    public void blinkVehicle(Color colorOriginal, Color colorToggle, long blinkTimeSeconds) throws InterruptedException {
        VehiclesHashMap.getVehicle(this.id).setColor(colorToggle);
        TimeUnit.SECONDS.sleep(blinkTimeSeconds);
        VehiclesHashMap.getVehicle(this.id).setColor(colorOriginal);
    }

    public int getID() {
        return id;
    }

    public Color getColor() {
        return currentRobotReport.getVelocity() > 0.1 ? colorMoving : colorStill;
    }

    public String getColorCode() {
        return "#" + String.format("%06x", 0xFFFFFF & getColor().getRGB());
    }

    public Coordinate[] getFootPrint() {
        return footPrint;
    }

    public synchronized void setCurrentRobotReport(RobotReport currentRobotReport)  {
        this.lastRobotReport = this.currentRobotReport;
        this.currentRobotReport = currentRobotReport;
        updateStatistics();
    }

    public RobotReport getCurrentRobotReport() {
        return currentRobotReport;
    }

    public double getCycleDistance() {
        return cycleDistance;
    }

    public void setCycleDistance(PoseSteering[] path) {
        for (int i = 0; i < path.length-1; i++) {
            double deltaS = path[i].getPose().distanceTo(path[i + 1].getPose());
            cycleDistance += deltaS;
        }
        cycleDistance = Math.round(cycleDistance * 10.0) / 10.0;
        VehiclesHashMap.getVehicle(this.getID()).cycleDistance = cycleDistance;
    }

    public PoseSteering[] getPath() {
        return path;
    }

    protected void setPath(PoseSteering[] path) {
        this.path = path;
        setCycleDistance(path);
    }

    public RobotReport getLastRobotReport() {
        return lastRobotReport;
    }

    public double getMaxVelocity() {
        return maxVelocity;
    }

    public double getMaxAcceleration() {
        return maxAcceleration;
    }

    public void setColor(Color color) {
        this.colorMoving = color;
        this.colorStill = color;
    }

    public String getType() {
        return type;
    }
}
