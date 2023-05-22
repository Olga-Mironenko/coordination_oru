package se.oru.coordination.coordination_oru.utility;

import se.oru.coordination.coordination_oru.utility.Mission;
import se.oru.coordination.coordination_oru.utility.Missions;

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
        for (int i = 0; i < numCalls; i++) {
            int delay = random.nextInt((simulationTime * 60) + 1);
            this.executorService.schedule(() -> callLookAheadRobot(mission), delay, TimeUnit.SECONDS);
            System.out.printf("Method will be called at: %d minutes %d seconds.%n", delay / 60, delay % 60);
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
