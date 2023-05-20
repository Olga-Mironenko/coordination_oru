package se.oru.coordination.coordination_oru.robots;

import java.util.HashMap;

/**
 * RobotsHashMap is a singleton class that manages a list of AbstractRobot objects using their IDs as keys.
 * It provides methods to access a single instance of the RobotsHashMap,
 * get a specific robot, and retrieve the list of robots.
 *
 * @author anm
 */
public class RobotHashMap {
    private static final Object lock = new Object();
    private static final HashMap<Integer, AbstractRobot> list = new HashMap<>();
    private static volatile RobotHashMap instance;

    /**
     * Private constructor to prevent instantiation of the class.
     */
    private RobotHashMap() {
    }

    /**
     * Returns the singleton instance of the RobotsHashMap class.
     * If the instance does not exist, it creates a new instance using double-checked locking.
     *
     * @return The singleton instance of the RobotsHashMap class.
     */
    public static RobotHashMap getInstance() {
        if (instance == null) {
            synchronized (lock) {
                if (instance == null) {
                    instance = new RobotHashMap();
                }
            }
        }
        return instance;
    }

    /**
     * Retrieves a robot from the list by its ID.
     *
     * @param key The ID of the robot to be retrieved.
     * @return The AbstractRobot object with the specified ID, or null if the robot is not found.
     */
    public static AbstractRobot getRobot(int key) {
        if (instance == null) {
            synchronized (lock) {
                if (instance == null) {
                    instance = new RobotHashMap();
                }
            }
        }
        return getList().get(key);
    }

    /**
     * Retrieves the list of robots.
     *
     * @return A HashMap containing the list of robots with their IDs as keys.
     */
    public synchronized static HashMap<Integer, AbstractRobot> getList() {
        return list;
    }
}
