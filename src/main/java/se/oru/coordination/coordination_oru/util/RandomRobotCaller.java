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
        int lastCallTime = 0;
        for (int i = 0; i < numCalls; i++) {
            int maxDelay = (((simulationTime - 2) * 60 - lastCallTime - 2 * 60 * (numCalls - i - 1)) / (numCalls - i));
            int delay = (i == 0 ? random.nextInt(60) + 1 // Random time up to one minute for the first call
                    : random.nextInt(Math.max(1, maxDelay)) + 1); // Random time up to the max delay for subsequent calls
            lastCallTime += delay + (i == 0 ? 0 : 2 * 60); // Add the delay and two minutes to the last call time for subsequent calls

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
