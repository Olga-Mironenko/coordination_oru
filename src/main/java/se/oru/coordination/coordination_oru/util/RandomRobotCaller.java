package se.oru.coordination.coordination_oru.util;

import se.oru.coordination.coordination_oru.Mission;

import java.util.Random;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

/**
 * Class responsible for scheduling missions for the robot at random intervals.
 *
 * @author anm
 */
public class RandomRobotCaller {
    private final ScheduledExecutorService executorService;
    private final Random random;
    private final int numCalls;
    private final int simulationTime;

    /**
     * Constructor for the RandomRobotCaller.
     *
     * @param numCalls The number of calls to schedule.
     * @param simulationTime The total simulation time in minutes.
     */
    public RandomRobotCaller(int numCalls, int simulationTime) {
        this.executorService = Executors.newScheduledThreadPool(numCalls);
        this.random = new Random();
        this.numCalls = numCalls;
        this.simulationTime = simulationTime;
    }

    /**
     * Schedules random calls for a given mission.
     *
     * @param mission The mission to be scheduled.
     */
    public void scheduleRandomCalls(Mission mission) {
        int totalSimulationSeconds = simulationTime * 60; // Convert minutes to seconds
        int buffer = 60; // Buffer for the end of the simulation (60 seconds = 1 minute)
        int averageInterval = (totalSimulationSeconds - buffer) / numCalls;

        int lastCallTime = 0;

        for (int i = 0; i < numCalls; i++) {
            // Get a random delay around the average interval
            int minDelay = (int) (0.8 * averageInterval);
            int maxDelay = (int) (1.2 * averageInterval);
            int delay = random.nextInt(maxDelay - minDelay + 1) + minDelay;

            // Ensure the last call fits within the remaining simulation time minus the buffer
            if (i == numCalls - 1 && (lastCallTime + delay) > (totalSimulationSeconds - buffer)) {
                delay = totalSimulationSeconds - buffer - lastCallTime;
            }

            lastCallTime += delay;
            this.executorService.schedule(() -> callLookAheadRobot(mission), lastCallTime, TimeUnit.SECONDS);
            System.out.printf("Method will be called at: %d minutes %d seconds.%n", lastCallTime / 60, lastCallTime % 60);
        }
        this.executorService.shutdown();
    }

    /**
     * Enqueues the given mission to the queue of missions to be performed.
     *
     * @param mission The mission to be enqueued.
     */
    private static void callLookAheadRobot(Mission mission) {
        Missions.enqueueMission(mission);
    }
}
