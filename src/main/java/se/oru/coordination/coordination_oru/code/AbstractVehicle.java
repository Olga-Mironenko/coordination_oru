package se.oru.coordination.coordination_oru.code;

import com.vividsolutions.jts.geom.Coordinate;
import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.PoseSteering;
import se.oru.coordination.coordination_oru.AbstractTrajectoryEnvelopeTracker;
import se.oru.coordination.coordination_oru.ConstantAccelerationForwardModel;
import se.oru.coordination.coordination_oru.RobotReport;
import se.oru.coordination.coordination_oru.TrajectoryEnvelopeTrackerDummy;
import se.oru.coordination.coordination_oru.simulation2D.AdaptiveTrajectoryEnvelopeTrackerRK4;
import se.oru.coordination.coordination_oru.simulation2D.TrajectoryEnvelopeCoordinatorSimulation;
import se.oru.coordination.coordination_oru.util.BrowserVisualization;
import se.oru.coordination.coordination_oru.util.Containerization;
import se.oru.coordination.coordination_oru.util.Forcing;
import se.oru.coordination.coordination_oru.util.Missions;
import se.oru.coordination.coordination_oru.util.gates.GatedCalendar;
import se.oru.coordination.coordination_oru.util.gates.GatedThread;

import java.awt.*;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.text.SimpleDateFormat;
import java.util.*;
import java.util.concurrent.TimeUnit;
import java.util.stream.Collectors;

/**
 * AbstractRobot is an abstract class representing a generic robot with common attributes and methods.
 * It should be extended by concrete robot classes with specific implementations.
 * The class keeps track of the robot's ID, priorityID, color,
 * maxVelocity, maxAcceleration, and footprint of the robot.
 *
 * @author anm
 */
public abstract class AbstractVehicle {
    public static int vehicleNumber = 1;
    private final int ID;
    private final int priorityID;
    private final String type = this.getClass().getSimpleName();
    private double maxVelocity;
    private final double maxVelocityOriginal;
    private final double maxAcceleration;
    private final double xLength;
    private final double yLength;
    private Coordinate[] footprint;
    public Coordinate[] innerFootprint = null;
    private final double startTime = GatedCalendar.getInstance().getTimeInMillis();
    private Color color;
    private Color colorInMotion;
    public RobotReport currentRobotReport = new RobotReport(-1, null, -1, 0.0, 0.0, 0.0, -1);
    public RobotReport lastRobotReport = new RobotReport(-1, null, -1, 0.0, 0.0, 0.0, -1);
    public double totalDistance;
    private double totalTime;
    private int numMissions;
    private double maxWaitingTime;
    private double currentWaitingTime;
    private double totalWaitingTime;
    private int stops;
    private PoseSteering[] path;
    private double pathLength;
    private static boolean isRundirPrepared = false;
    public boolean isAdaptiveTracker = false;
    private static final String rundirsRoot = Containerization.RUNDIRS;
    private static final String dateString = new SimpleDateFormat("yyyyMMdd_HHmmss").format(new Date());
    private static final String rundirCurrent = rundirsRoot + "/current";
    public static String scenarioId;
    public static String scenarioFilename;
    public VehicleSize vehicleSize;

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
    public AbstractVehicle(int ID, int priorityID, Color color, Color colorInMotion, double maxVelocity, double maxAcceleration, double xLength, double yLength) {
        this.ID = ID;
        this.priorityID = priorityID;
        this.color = color;
        this.colorInMotion = colorInMotion;
        this.maxVelocity = maxVelocity;
        this.maxVelocityOriginal = maxVelocity;
        this.maxAcceleration = maxAcceleration;
        this.xLength = xLength;
        this.yLength = yLength;
        this.footprint = xLength == 0 && yLength == 0 ? null : makeFootprint(xLength, yLength);
        this.totalDistance = 0;

        AbstractVehicle existingVehicle = VehiclesHashMap.getVehicle(ID);
        if (existingVehicle != null) {
            throw new IllegalStateException("ID " + ID + " already exists.");
        }

        VehiclesHashMap.getList().put(this.ID, this);
        vehicleNumber++;
    }

    public AbstractVehicle(int priorityID, Color color, Color colorInMotion, double maxVelocity, double maxAcceleration, double xLength, double yLength) {
        this(vehicleNumber + 1, priorityID, color, colorInMotion, maxVelocity, maxAcceleration, xLength, yLength);
    }

    public AbstractVehicle(int priorityID, Color color, double maxVelocity, double maxAcceleration, double xLength, double yLength) {
        this(vehicleNumber, priorityID, color, null, maxVelocity, maxAcceleration, xLength, yLength);
    }

    public static Coordinate[] makeFootprint(double xLength, double yLength) {
        return makeFootprint(xLength, xLength, yLength, yLength);
    }

    public static Coordinate[] makeFootprint(double xLengthFront, double xLengthBack,
                                             double yLengthLeft, double yLengthRight) {
        return new Coordinate[]{               // FIXME Currently allows four sided vehicles only
                new Coordinate(-xLengthBack, yLengthLeft),         //back left
                new Coordinate(xLengthFront, yLengthLeft),         //front left
                new Coordinate(xLengthFront, -yLengthRight),       //front right
                new Coordinate(-xLengthBack, -yLengthRight)        //back right
        };
        // length=3, safeDistance=1 -> fullLength=3+1*2=5, x=-2.5..2.5
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
        registerInTec(tec, new VehicleSize(xLengthInner * 2, yLengthInner * 2, 0, 0, 0, 0));
    }

    public void registerInTec(TrajectoryEnvelopeCoordinatorSimulation tec,
                              VehicleSize vehicleSize) {
        this.vehicleSize = vehicleSize;

        // E.g.: vehicleSize = {length: 3, frontSafeDistance: 1, backSafeDistance: 1}:
        double xLengthInner = vehicleSize.length / 2; // -> inner footprint: x = -1.5 .. 1.5
        // total length: 3 + 1*2 = 5 -> (outer) footprint (computed below): x = -2.5 .. 2.5
        double yLengthInner = vehicleSize.width / 2;

        this.innerFootprint = AbstractVehicle.makeFootprint(xLengthInner, yLengthInner);
        this.footprint = AbstractVehicle.makeFootprint(
            xLengthInner + vehicleSize.frontSafeDistance, xLengthInner + vehicleSize.backSafeDistance,
            yLengthInner + vehicleSize.leftSafeDistance, yLengthInner + vehicleSize.rightSafeDistance
        );

        tec.setInnerFootprint(getID(), this.innerFootprint);
        tec.setFootprint(getID(), this.footprint);

        tec.setForwardModel(
                this.getID(),
                new ConstantAccelerationForwardModel(
                        this.getMaxAcceleration(),
                        this.getMaxVelocity(),
                        tec.getTemporalResolution(),
                        tec.getControlPeriod(),
                        tec.getRobotTrackingPeriodInMillis(this.getID())
                )
        );
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

    private static boolean isStopped(RobotReport rr) {
        assert rr.getVelocity() >= 0.0;
        return rr.getVelocity() == 0.0;
    }

    public synchronized void updateStatistics() {
        // Loading and unloading times and stoppages are not considered
        if ((this.currentRobotReport.getPathIndex() == -1) && (this.lastRobotReport.getPathIndex() != -1))
            this.numMissions++;

        double totalTimeNew = (GatedCalendar.getInstance().getTimeInMillis() - startTime) / 1000;
        double delta = totalTimeNew - totalTime;
        this.totalTime = totalTimeNew;

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

    public static String getScenarioIdAsBasename() {
        if (scenarioId == null) {
            return "null";
        }
        return scenarioId.replace('/', '_').replace(' ', '_');
    }

    public LinkedHashMap<String, String> collectStatistics() {
        LinkedHashMap<String, String> mapStats = new LinkedHashMap<>();
        TrajectoryEnvelopeCoordinatorSimulation tec = TrajectoryEnvelopeCoordinatorSimulation.tec;

        mapStats.put("Date", dateString);
        mapStats.put("Scenario ID", scenarioId);
        mapStats.put("Vehicle ID", "" + this.getID());
        mapStats.put("Vehicle type", this.type);

        mapStats.put("Cycle distance (m)", "" + this.pathLength);
        mapStats.put("No. of completed missions", "" + this.numMissions);
        mapStats.put("Total distance traveled (m)", "" + round(totalDistance));

        int numForcings = Forcing.robotIDToNumForcingEvents.getOrDefault(ID, 0);
        int numUselessForcings = Forcing.robotIDToNumUselessForcingEvents.getOrDefault(ID, 0);
        int numViolations = numForcings - numUselessForcings;

        mapStats.put("No. of stops", "" + this.stops);
        mapStats.put("No. of forcing events", "" + numForcings);
        mapStats.put("No. of violations", "" + numViolations);
        mapStats.put("No. of critical sections", "" + tec.robotIDToNumPotentialInteractions.get(ID));

        mapStats.put("No. of near-misses", "" + tec.robotIDToMinorCollisions.getOrDefault(ID, new ArrayList<>()).size());
        mapStats.put("No. of collisions", "" + tec.robotIDToMajorCollisions.getOrDefault(ID, new ArrayList<>()).size());
        mapStats.put("Is blocked", "" + (VehiclesHashMap.getVehicle(ID).isBlocked() ? 1 : 0));

        mapStats.put("Total waiting time (s)", "" + round(totalWaitingTime));
        mapStats.put("Maximum waiting time (s)", "" + round(maxWaitingTime));
        mapStats.put("Total time (s)", "" + round(totalTime));

        mapStats.put("Maximum acceleration (m/s^2)", "" + round(maxAcceleration));
        mapStats.put("Maximum speed (m/s)", "" + round(maxVelocity));
        mapStats.put("Average speed (m/s)", "" + round(totalDistance / totalTime));

        if (BrowserVisualization.mapPretable != null) {
            mapStats.putAll(BrowserVisualization.mapPretable);
        }

        String[] columns = BrowserVisualization.statsColumns;
        if (columns != null) {
            String[] rows = BrowserVisualization.statsIdToRow.get(ID);
            assert columns.length == rows.length;
            for (int i = 1; i < columns.length; i++) {
                mapStats.put(columns[i], rows[i]);
            }
        }

        return mapStats;
    }

    public LinkedHashMap<String, String> collectLinearizations() {
        LinkedHashMap<String, String> mapStats = new LinkedHashMap<>();

        addLinearization(mapStats, "A", Missions.robotIDToMissionLinearizationA.get(ID));
        addLinearization(mapStats, "B", Missions.robotIDToMissionLinearizationB.get(ID));
        addLinearization(mapStats, "C", Missions.robotIDToMissionLinearizationC.get(ID));
        for (AbstractVehicle other : VehiclesHashMap.getVehicles()) {
            if (other.ID == ID) {
                continue;
            }
            addLinearization(mapStats, "D" + other.ID,
                    Missions.robotIDToOtherIDToMissionLinearizationD.get(ID).get(other.ID));
        }

        return mapStats;
    }

    public void writeStatistics() {
        LinkedHashMap<String, String> mapStats = collectStatistics();
        if (GatedThread.gatekeeper.isOver) {
            mapStats.putAll(collectLinearizations());
        }

        try {
            String subdir = dateString + "_" + Containerization.WORKER + (
                    scenarioId == null
                            ? ""
                            : "_" + getScenarioIdAsBasename()
            );
            File dir = new File(rundirsRoot + "/" + subdir);
            if (!isRundirPrepared) {
                boolean isCreated = dir.mkdirs();
                assert isCreated;

                if (! Containerization.IS_CONTAINER) {
                    Path current = Path.of(rundirCurrent);
                    Files.deleteIfExists(current);
                    Files.createSymbolicLink(current, Path.of(subdir));
                }

                isRundirPrepared = true;
            }

            File file = new File(dir + "/" + this.ID + ".csv");
            FileWriter fw = new FileWriter(file.getAbsoluteFile(), false);
            BufferedWriter bw = new BufferedWriter(fw);
            for (Map.Entry<String, String> entry : mapStats.entrySet()) {
                bw.write(entry.getKey() + "\t" + entry.getValue() + "\n");
            }
            bw.close();
        } catch (IOException e) {
            throw new RuntimeException(e.getMessage());
        }
    }

    private static void addLinearization(LinkedHashMap<String, String> mapStats, String name, double[] linearization) {
        String result = Arrays.stream(linearization)
                .mapToObj(d -> String.format("%.6f", d)) // Convert each number to String
                .collect(Collectors.joining(" "));
        mapStats.put("Linearization " + name, result);
    }

    /**
     * Blinks the robot's color between the original color and a specified toggle color for a given duration.
     *
     * @param colorOriginal    The original color of the robot.
     * @param colorToggle      The color to toggle to during blinking.
     * @param blinkTimeSeconds The duration of the blink in seconds.
     * @throws InterruptedException if the sleep operation is interrupted.
     */
    public void blinkRobot(Color colorOriginal, Color colorToggle, long blinkTimeSeconds) throws InterruptedException {
        VehiclesHashMap.getVehicle(ID).setVehicleColor(colorToggle);
        TimeUnit.SECONDS.sleep(blinkTimeSeconds);
        VehiclesHashMap.getVehicle(ID).setVehicleColor(colorOriginal);
    }

    public Color getVehicleColor() {
        return currentRobotReport.getVelocity() > 0.1 ? color : colorInMotion;
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

    protected static double round(double value) {
        return Math.round(value * 10.0) / 10.0;
    }

    public RobotReport getLastRobotReport() {
        return lastRobotReport;
    }

    public double getXLength() {
        return xLength;
    }

    public double getYLength() {
        return yLength;
    }

    public boolean isYPassedDownwards(double y) {
        return lastRobotReport.getY() > y && currentRobotReport.getY() <= y;
    }

    public boolean isYPassedUpwards(double y) {
        return lastRobotReport.getY() < y && currentRobotReport.getY() >= y;
    }

    public int getID() {
        return ID;
    }

    public void setVehicleColor(Color color) {
        this.color = color;
    }

    public String getColorCode() {
        return "#" + String.format("%06x", 0xFFFFFF & getColor().getRGB());
    }

    public Coordinate[] getFootPrint() {
        return footprint;
    }

    public double getPlanLength() {
        return pathLength;
    }

    public void setPlanLength(PoseSteering[] path) {
        for (int i = 0; i < path.length - 1; i++) {
            double deltaS = path[i].getPose().distanceTo(path[i + 1].getPose());
            pathLength += deltaS;
        }
        pathLength = round(pathLength * 10.0) / 10.0;
    }

    public PoseSteering[] getPath() {
        return path;
    }

    public void setPath(PoseSteering[] path) {
        this.path = path;
        setPlanLength(path);
    }

    public double getMaxVelocity() {
        return maxVelocity;
    }

    public double getMaxVelocityOriginal() {
        return maxVelocityOriginal;
    }

    public AbstractTrajectoryEnvelopeTracker getTracker() {
        return TrajectoryEnvelopeCoordinatorSimulation.tec.trackers.get(getID());
    }

    public AdaptiveTrajectoryEnvelopeTrackerRK4 getAdaptiveTracker() {
        AbstractTrajectoryEnvelopeTracker tracker = getTracker();
        if (tracker instanceof TrajectoryEnvelopeTrackerDummy) {
            return null;
        }
        assert tracker instanceof AdaptiveTrajectoryEnvelopeTrackerRK4;
        return ((AdaptiveTrajectoryEnvelopeTrackerRK4) tracker);
    }

    public void updateTracker() {
        AdaptiveTrajectoryEnvelopeTrackerRK4 tracker = getAdaptiveTracker();
        if (tracker != null) {
            tracker.onTrajectoryEnvelopeUpdate();
        }
    }

    public void setMaxVelocity(double maxVelocity) {
        // Note: Comment this to force `setCriticalPoint` from time to time
        // (to reveal bugs):
        if (maxVelocity == this.maxVelocity) {
            return;
        }
        this.maxVelocity = maxVelocity;
        updateTracker();
    }

    public void resetMaxVelocity() {
        setMaxVelocity(this.maxVelocityOriginal);
    }

    public boolean isMaxVelocityLowered() {
        return getMaxVelocity() < getMaxVelocityOriginal();
    }

    public double getMaxAcceleration() {
        return maxAcceleration;
    }

    public String getType() {
        return type;
    }

    public String getTypeForVisualization() {
        switch (type) {
            case "HumanDrivenVehicle":
                return "human-controlled";
            case "AutonomousVehicle":
                return "automated";
            default:
                return "unknown";
        }
    }

    public Color getColor() {
        return color;
    }

    public int getNumMissions() {
        return numMissions;
    }

    public boolean isBlocked() {
        AdaptiveTrajectoryEnvelopeTrackerRK4 tracker = getAdaptiveTracker();
        if (tracker == null) {
            return false;
        }
        return tracker.isBlocked();
    }
}
