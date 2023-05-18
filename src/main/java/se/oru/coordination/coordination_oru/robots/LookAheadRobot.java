package se.oru.coordination.coordination_oru.robots;

import org.apache.commons.lang.ArrayUtils;
import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.PoseSteering;
import se.oru.coordination.coordination_oru.coordinator.TrajectoryEnvelopeCoordinator;
import se.oru.coordination.coordination_oru.motionplanner.ompl.ReedsSheppCarPlanner;

import java.awt.*;
import java.util.Arrays;
import java.util.concurrent.atomic.AtomicInteger;

/**
 * LookAheadRobot is a subclass of AbstractRobot that represents robots capable of predicting
 * their path up to a certain distance. The class provides methods for updating the paths of LookAheadRobots
 * in the TrajectoryEnvelopeCoordinator and generating limited paths based on the predictable distance.
 *
 * @author anm
 */
public class LookAheadRobot extends AbstractRobot {

    private final double predictableDistance;

    /**
     * Constructs a new LookAheadRobot with the specified parameters.
     *
     * @param ID                  The robot ID.
     * @param priorityID          The priority ID of the robot.
     * @param predictableDistance The maximum distance ahead in the path to consider.
     * @param color               The color of the robot when stationary.
     * @param maxVelocity         The maximum velocity of the robot.
     * @param maxAcceleration     The maximum acceleration of the robot.
     * @param xLength             The length of the robot along the X-axis.
     * @param yLength             The length of the robot along the Y-axis.
     */
    public LookAheadRobot(int ID, int priorityID, double predictableDistance, Color color, double maxVelocity, double maxAcceleration, double xLength, double yLength) {
        super(ID, priorityID, color, maxVelocity, maxAcceleration, xLength, yLength);
        this.predictableDistance = predictableDistance;
    }

    public LookAheadRobot(int priorityID, double predictableDistance, Color color, double maxVelocity, double maxAcceleration, double xLength, double yLength) {
        super(robotNumber.intValue(), priorityID, color, maxVelocity, maxAcceleration, xLength, yLength);
        this.predictableDistance = predictableDistance;
    }

    public LookAheadRobot(double predictableDistance) {
        super(robotNumber.intValue(), 1, Color.RED, 5, 2, 0.5, 0.5);
        this.predictableDistance = predictableDistance;
    }

    /**
     * Updates the path of all LookAheadRobots in the TrajectoryEnvelopeCoordinator.
     *
     * @param tec The TrajectoryEnvelopeCoordinator containing the robots.
     */
    public static void updateLookAheadRobotPath(TrajectoryEnvelopeCoordinator tec) {
        for (int robotID : tec.getAllRobotIDs()) {
            var robot = RobotHashMap.getRobot(robotID);
            if (robot.getType().equals("LookAheadRobot")) {
                var lookAheadRobot = (LookAheadRobot) robot;
                var newPath = lookAheadRobot.getLimitedPath(robotID, lookAheadRobot.getPredictableDistance(), tec);
                tec.replacePath(robotID, newPath, 0, false, null);
            }
        }
    }

    /**
     * Generates a path for the robot using the default planning algorithm and parameters.
     *
     * @param initial     The initial pose of the robot.
     * @param goals       An array of goal poses.
     * @param map         The map used for planning.
     * @param inversePath A flag indicating whether the inverse path should also be computed.
     */
    @Override
    public void getPlan(Pose initial, Pose[] goals, String map, Boolean inversePath) {
        getPlan(initial, goals, map, inversePath, ReedsSheppCarPlanner.PLANNING_ALGORITHM.RRTConnect, 0.01, 60, 0.01, 0.1);
    }

    /**
     * Generates a path for the robot using the specified planning algorithm and parameters.
     *
     * @param initial                   The initial pose of the robot.
     * @param goals                     An array of goal poses.
     * @param map                       The map used for planning.
     * @param inversePath               A flag indicating whether the inverse path should also be computed.
     * @param planningAlgorithm         The planning algorithm to be used.
     * @param radius                    The radius used for planning.
     * @param planningTime              The maximum planning time in seconds.
     * @param turningRadius             The turning radius of the robot.
     * @param distanceBetweenPathPoints The distance between path points in the generated path.
     */
    public void getPlan(Pose initial, Pose[] goals, String map, Boolean inversePath, ReedsSheppCarPlanner.PLANNING_ALGORITHM planningAlgorithm,
                        double radius, double planningTime, double turningRadius, double distanceBetweenPathPoints) {

        var rsp = configureReedsSheppCarPlanner(planningAlgorithm, map, radius, planningTime, turningRadius, distanceBetweenPathPoints);
        generatePath(rsp, initial, goals, inversePath);
    }

    /**
     * Configures a ReedsSheppCarPlanner instance with the specified parameters.
     *
     * @param planningAlgorithm         The planning algorithm to be used.
     * @param map                       The map used for planning.
     * @param radius                    The radius used for planning.
     * @param planningTime              The maximum planning time in seconds.
     * @param turningRadius             The turning radius of the robot.
     * @param distanceBetweenPathPoints The distance between path points in the generated path.
     * @return A configured ReedsSheppCarPlanner instance.
     */
    private ReedsSheppCarPlanner configureReedsSheppCarPlanner(ReedsSheppCarPlanner.PLANNING_ALGORITHM planningAlgorithm, String map, double radius,
                                                               double planningTime, double turningRadius, double distanceBetweenPathPoints) {
        var rsp = new ReedsSheppCarPlanner(planningAlgorithm);
        rsp.setMap(map);
        rsp.setRadius(radius);
        rsp.setPlanningTimeInSecs(planningTime);
        rsp.setFootprint(getFootPrint());
        rsp.setTurningRadius(turningRadius);
        rsp.setDistanceBetweenPathPoints(distanceBetweenPathPoints);
        return rsp;
    }

    /**
     * Generates a path for the robot using the provided ReedsSheppCarPlanner instance.
     *
     * @param rsp         The ReedsSheppCarPlanner instance used for planning.
     * @param initial     The initial pose of the robot.
     * @param goals       An array of goal poses.
     * @param inversePath A flag indicating whether the inverse path should also be computed.
     */
    private void generatePath(ReedsSheppCarPlanner rsp, Pose initial, Pose[] goals, Boolean inversePath) {
        rsp.setStart(initial);
        rsp.setGoals(goals);
        rsp.plan();

        if (rsp.getPath() == null) {
            throw new Error("No path found.");
        }

        var pathFwd = rsp.getPath();
        var path = inversePath ? (PoseSteering[]) ArrayUtils.addAll(pathFwd, rsp.getPathInv()) : pathFwd;
        RobotHashMap.getRobot(getID()).setPath(path);
    }

    /**
     * Retrieves a limited path for the LookAheadRobot up to a specified predictable distance.
     *
     * @param robotID             The ID of the robot for which the path is being generated.
     * @param predictableDistance The maximum distance ahead in the path to consider.
     * @param tec                 The TrajectoryEnvelopeCoordinator containing the robots.
     * @return An array of PoseSteering objects representing the limited path.
     */
    public PoseSteering[] getLimitedPath(int robotID, double predictableDistance, TrajectoryEnvelopeCoordinator tec) {
        double distance = 0.0;
        var robotReport = tec.getRobotReport(robotID);
        var fullPath = getPath();

        if (robotReport == null) {
            // Handle the case when the RobotReport is null, e.g., return an empty path or throw a custom exception
            System.err.println("Error: RobotReport for robotID " + robotID + " not found.");
            return new PoseSteering[0];
        }

        double currentDistance = robotReport.getDistanceTraveled();
        double totalDistance = getPlanLength();
        int pathIndex = Math.max(robotReport.getPathIndex(), 0); // Avoid null point exception for starting

        for (; pathIndex < fullPath.length - 1 && distance <= predictableDistance; pathIndex++) {
            if ((currentDistance + predictableDistance) >= totalDistance) {
                return fullPath; // For last iteration less than predictable distance
            } else {
                distance += fullPath[pathIndex].getPose().distanceTo(fullPath[pathIndex + 1].getPose());
            }
        }

        // Create a new path with a limited length
        return Arrays.copyOfRange(fullPath, 0, pathIndex);
    }

    public double getPredictableDistance() {
        return predictableDistance;
    }

}

