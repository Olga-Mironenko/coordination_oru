package se.oru.coordination.coordination_oru.code;

import java.awt.Color;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Arrays;
import java.util.Collections;
import java.util.concurrent.TimeUnit;

import org.apache.commons.io.FileUtils;
import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.PoseSteering;

import com.vividsolutions.jts.geom.Coordinate;

import se.oru.coordination.coordination_oru.RobotReport;

public abstract class AbstractVehicle {
    public static int vehicleNumber = 1;
    private final int ID;
    private final int priorityID;
    private final String type = this.getClass().getSimpleName();
    private final double maxVelocity;
    private final double maxAcceleration;
    private final double xLength;
    private final double yLength;
    private final Coordinate[] footprint;
    public Coordinate[] innerFootprint = null;
    private final double startTime = System.nanoTime();
    private Color color;
    private Color colorInMotion;
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
    private PoseSteering[] path;

    private final String statisticsDirectory = "results";
    private boolean isStatisticsDirectoryCleaned = false;

    public AbstractVehicle(int id, int priorityID, Color color, Color colorInMotion, double maxVelocity, double maxAcceleration, double xLength, double yLength) {
        this.ID = id;
        this.priorityID = priorityID;
        this.color = color;
        this.colorInMotion = colorInMotion;
        this.maxVelocity = maxVelocity;
        this.maxAcceleration = maxAcceleration;
        this.xLength = xLength;
        this.yLength = yLength;
        this.footprint = makeFootprint(xLength, yLength);

        if (VehiclesHashMap.getList().containsKey(id)) {
            throw new Error("ID " + id + " already exists.");
        }
        VehiclesHashMap.getList().put(id, this);
        vehicleNumber++;
    }

    public static Coordinate[] makeFootprint(double xLength, double yLength) {
        return new Coordinate[]{               // FIXME Currently allows four sided vehicles only
                new Coordinate(-xLength, yLength),        //back left
                new Coordinate(xLength, yLength),         //back right
                new Coordinate(xLength, -yLength),        //front right
                new Coordinate(-xLength, -yLength)        //front left
        };
    }

    public AbstractVehicle(int priorityID, Color color, Color colorInMotion, double maxVelocity, double maxAcceleration, double xLength, double yLength) {
        this(
                VehiclesHashMap.getList().isEmpty() ? 1 : Collections.max(VehiclesHashMap.getList().keySet()) + 1,
                priorityID, color, colorInMotion, maxVelocity, maxAcceleration, xLength, yLength
        );
    }


    public AbstractVehicle(int priorityID, Color color, double maxVelocity, double maxAcceleration, double xLength, double yLength) {
        this(vehicleNumber, priorityID, color, null, maxVelocity, maxAcceleration, xLength, yLength);
    }

    @Override
    public String toString() {
        return "AbstractVehicle{" +
                "ID=" + ID +
                ", priorityID=" + priorityID +
                ", type='" + type + '\'' +
                ", color=" + color +
                ", maxVelocity=" + maxVelocity +
                ", maxAcceleration=" + maxAcceleration +
                ", xLength=" + xLength +
                ", yLength=" + yLength +
                ", footprint=" + Arrays.toString(footprint) +
                (innerFootprint == null ? "" : ", innerFootprint=" + Arrays.toString(innerFootprint)) +
                '}';
    }

    public abstract PoseSteering[] getPlan(Pose initial, Pose[] goals, String map, Boolean inversePath);

    private static boolean isStopped(RobotReport rr) {
        return rr.getVelocity() < 1e-3;
    }

    public synchronized void updateStatistics() {
        // Loading and unloading times and stoppages are not considered
        if ((this.currentRobotReport.getPathIndex() == -1) && (this.lastRobotReport.getPathIndex() != -1))
            this.cycles++;

        if (isStopped(this.currentRobotReport)) {
            if (! isStopped(this.lastRobotReport))
                this.stops++;

            this.currentWaitingTime += 2;
            this.totalWaitingTime += 2;
            this.maxWaitingTime = Math.max(currentWaitingTime, maxWaitingTime);
        }

        this.totalDistance = Math.round(
                (cycleDistance * cycles + currentRobotReport.getDistanceTraveled()) * 10.0
        ) / 10.0;
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
//            String line1 = "Map: " + map + "\n";
            String line2 = "Vehicle: V" + this.getID() + "  " + this.type + "\n";
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

            if (! isStatisticsDirectoryCleaned) {
                File dir = new File(statisticsDirectory);
                dir.mkdirs();
                FileUtils.cleanDirectory(dir);
                isStatisticsDirectoryCleaned = true;
            }

            String fileName = statisticsDirectory + "/" + this.ID + ".txt";
            File file = new File(fileName);

            // if file doesn't exist, then create it
            if (!file.exists()) {
                file.createNewFile();
            }

            //FIXME Use a loop for ease
            FileWriter fw = new FileWriter(file.getAbsoluteFile(), false);
            BufferedWriter bw = new BufferedWriter(fw);
            bw.write(line0);
//            bw.write(line1);
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
        VehiclesHashMap.getVehicle(this.ID).setVehicleColor(colorToggle);
        TimeUnit.SECONDS.sleep(blinkTimeSeconds);
        VehiclesHashMap.getVehicle(this.ID).setVehicleColor(colorOriginal);
    }
    public int getID() {
        return ID;
    }

    public Color getVehicleColor() {
        return currentRobotReport.getVelocity() > 0.1 ? color : colorInMotion;
    }

    public void setVehicleColor(Color color) {
        this.color = color;
        this.colorInMotion = color;
    }

    public String getColorCode() {
        return "#" + String.format("%06x", 0xFFFFFF & getColor().getRGB());
    }

    public Coordinate[] getFootprint() {
        return footprint;
    }

    public RobotReport getCurrentRobotReport() {
        return currentRobotReport;
    }

    public synchronized void setCurrentRobotReport(RobotReport currentRobotReport) {
        this.lastRobotReport = this.currentRobotReport;
        this.currentRobotReport = currentRobotReport;
        updateStatistics();
    }

    public double getCycleDistance() {
        return cycleDistance;
    }

    public void setCycleDistance(PoseSteering[] path) {
        for (int i = 0; i < path.length - 1; i++) {
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

    public String getType() {
        return type;
    }

    public Color getColor() {
        return color;
    }
}
