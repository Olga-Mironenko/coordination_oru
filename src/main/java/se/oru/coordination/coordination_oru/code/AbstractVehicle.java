package se.oru.coordination.coordination_oru.code;

import java.awt.Color;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.text.SimpleDateFormat;
import java.util.Arrays;
import java.util.Collections;
import java.util.Date;
import java.util.concurrent.TimeUnit;

import org.apache.commons.io.FileUtils;
import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.PoseSteering;

import com.vividsolutions.jts.geom.Coordinate;

import se.oru.coordination.coordination_oru.RobotReport;
import se.oru.coordination.coordination_oru.simulation2D.TrajectoryEnvelopeCoordinatorSimulation;
import se.oru.coordination.coordination_oru.util.MissionUtils;

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
    public RobotReport currentRobotReport = new RobotReport(-1, null, -1, 0.0, 0.0, -1);
    public RobotReport lastRobotReport = new RobotReport(-1, null, -1, 0.0, 0.0, -1);
    private double cycleDistance;
    private double totalDistance;
    private double totalTime;
    private int cycles;
    private double maxWaitingTime;
    private double currentWaitingTime;
    private double totalWaitingTime;
    private int stops;
    private PoseSteering[] path;

    private boolean isRundirPrepared = false;
    private static final String rundirsRoot = "logs/rundirs";
    private static final String dateString = new SimpleDateFormat("yyyyMMdd_HHmmss").format(new Date());
    private static final String rundirCurrent = rundirsRoot + "/current";
    public static String scenarioId;

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

    public void registerInTec(TrajectoryEnvelopeCoordinatorSimulation tec, double xLengthInner, double yLengthInner) {
        Coordinate[] innerFootprint = AbstractVehicle.makeFootprint(xLengthInner, yLengthInner);
        this.innerFootprint = innerFootprint;
        tec.setFootprint(getID(), getFootprint());
        tec.setInnerFootprint(getID(), innerFootprint);
    }

    public abstract PoseSteering[] getPlan(Pose initial, Pose[] goals, String map, Boolean inversePath);

    private static boolean isStopped(RobotReport rr) {
        return rr.getVelocity() < 1e-3;
    }

    public synchronized void updateStatistics() {
        // Loading and unloading times and stoppages are not considered
        if ((this.currentRobotReport.getPathIndex() == -1) && (this.lastRobotReport.getPathIndex() != -1))
            this.cycles++;

        double totalTimeNew = Math.round(System.nanoTime() - startTime) / 1_000_000_000;
        double delta = totalTimeNew - totalTime;
        this.totalTime = totalTimeNew;

        this.totalDistance = (cycleDistance * cycles + currentRobotReport.getDistanceTraveled());

        if (! isStopped(this.currentRobotReport)) {
            this.currentWaitingTime = 0;
        } else {
            if (! isStopped(this.lastRobotReport)) {
                this.stops++;
            }

            this.currentWaitingTime += delta;
            this.totalWaitingTime += delta;
        }
        this.maxWaitingTime = Math.max(currentWaitingTime, maxWaitingTime);
    }


    // TODO Write the data in a csv file with each cycle results
    // TODO Separate Scenario general settings and robot settings
    // TODO Date, Day, Timing settings. Fix file name
    // FIXME Check statistics
    public void writeStatistics() {

        try {
            String subdir = dateString + (scenarioId == null ? "" : "_" + scenarioId);
            File dir = new File(rundirsRoot + "/" + subdir);
            if (!isRundirPrepared) {
                dir.mkdirs();
                FileUtils.cleanDirectory(dir);

                Path current = Path.of(rundirCurrent);
                Files.deleteIfExists(current);
                Files.createSymbolicLink(current, Path.of(subdir));

                isRundirPrepared = true;
            }

            File file = new File(dir.toString() + "/" + this.ID + ".csv");
            FileWriter fw = new FileWriter(file.getAbsoluteFile(), false);
            BufferedWriter bw = new BufferedWriter(fw);

            bw.write("Date," + dateString + "\n");
            bw.write("Scenario ID," + scenarioId + "\n");
            bw.write("Vehicle ID," + this.getID() + "\n");
            bw.write("Vehicle type," + this.type + "\n");

            bw.write("Cycle distance (m)," + this.cycleDistance + "\n");
            bw.write("No. of completed cycles," + this.cycles + "\n");
            bw.write("Total distance travelled (m)," + round(totalDistance) + "\n");

            bw.write("No. of stops," + this.stops + "\n");
            bw.write("No. of forcing events," + MissionUtils.robotIDToNumForcingEvents.getOrDefault(ID, 0) + "\n");
            bw.write("No. of potential interactions," + TrajectoryEnvelopeCoordinatorSimulation.tec.robotIDToNumPotentialInteractions.get(ID) + "\n");
            bw.write("Total waiting time (s)," + round(totalWaitingTime) + "\n");
            bw.write("Maximum waiting time (s)," + round(maxWaitingTime) + "\n");
            bw.write("Total simulation time (s)," + round(totalTime) + "\n");

            bw.write("Maximum acceleration (m/s^2)," + round(maxAcceleration) + "\n");
            bw.write("Maximum speed (m/s)," + round(maxVelocity) + "\n");
            bw.write("Average speed (m/s)," + round(totalDistance / totalTime) + "\n");

            bw.close();

        } catch (IOException e) {
            throw new RuntimeException(e.getMessage());
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
        cycleDistance = round(cycleDistance);
        VehiclesHashMap.getVehicle(this.getID()).cycleDistance = cycleDistance;
    }

    protected static double round(double value) {
        return Math.round(value * 10.0) / 10.0;
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

    public double getXLength() {
        return xLength;
    }

    public double getYLength() {
        return yLength;
    }
}
