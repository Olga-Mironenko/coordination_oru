package se.oru.coordination.coordination_oru.robots;

import se.oru.coordination.coordination_oru.coordinator.TrajectoryEnvelopeCoordinator;
import se.oru.coordination.coordination_oru.tracker.AbstractTrajectoryEnvelopeTracker;

import java.io.BufferedWriter;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.StandardOpenOption;
import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;
import java.time.temporal.ChronoUnit;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicLong;

/**
 * RobotReportCollector is a class that periodically collects robot reports
 * and writes the data into a .csv file.
 *
 * @author anm
 */
public class RobotReportCollector {

    /**
     * Handles the periodic collection of robot reports and writes the data into a .csv file.
     * The method will terminate automatically after the specified termination time.
     * The timestamps in the output file will be rounded to seconds, and the 'T' character will be replaced with '-'.
     *
     * @param tec                  The TrajectoryEnvelopeCoordinator instance that contains the trackers.
     * @param baseFileName         The base name of the .csv file to store the robot data.
     * @param intervalInSeconds    The time interval (in seconds) between collecting and writing robot data.
     * @param terminationInMinutes The termination time (in minutes) for stopping the data collection.
     */
    public void handleRobotReports(TrajectoryEnvelopeCoordinator tec, String baseFileName, long intervalInSeconds, long terminationInMinutes) {
        ScheduledExecutorService executor = Executors.newSingleThreadScheduledExecutor();
        AtomicLong startTime = new AtomicLong(System.currentTimeMillis());

        // Convert interval and termination times to milliseconds
        long intervalInMillis = TimeUnit.SECONDS.toMillis(intervalInSeconds);
        long terminationInMillis = TimeUnit.MINUTES.toMillis(terminationInMinutes);

        // Generate the filename with date and timestamp
        String timestamp = LocalDateTime.now().format(DateTimeFormatter.ofPattern("yyyyMMdd-HHmmss"));
        String fileName = baseFileName + timestamp + ".csv";
        Path filePath = Path.of(fileName);

        Runnable task = () -> {
            try {
                // Get the current trackers list from the 'tec' object
                List<AbstractTrajectoryEnvelopeTracker> trackers = new ArrayList<>(tec.trackers.values());

                // Check if the elapsed time exceeds the specified termination time, and if so, shut down the executor
                long elapsedTime = System.currentTimeMillis() - startTime.get();
                if (elapsedTime >= terminationInMillis) {
                    executor.shutdown();
                    return;
                }

                // Create and/or open the file in appended mode
                try (BufferedWriter writer = Files.newBufferedWriter(filePath, StandardOpenOption.CREATE, StandardOpenOption.APPEND)) {
                    // Write the header if it's a new file
                    if (Files.size(filePath) == 0) {
                        writer.write("Timestamp,RobotID,Pose,PathIndex,Velocity,DistanceTraveled,CriticalPoint\n");
                    }

                    // Get the current timestamp rounded to seconds and replace 'T' with '/'
                    String currentTimestamp = LocalDateTime.now()
                            .truncatedTo(ChronoUnit.SECONDS)
                            .format(DateTimeFormatter.ISO_LOCAL_DATE_TIME)
                            .replace('T', '/');

                    // Iterate through the trackers and get the robot reports
                    for (AbstractTrajectoryEnvelopeTracker tracker : trackers) {
                        var report = tracker.getRobotReport();
                        var pose = report.getPose();
                        String poseString = String.format("\"(%.2f, %.2f, %.2f)\"", pose.getX(), pose.getY(), pose.getTheta());
                        String rowData = String.format("%s,%d,%s,%d,%.2f,%.2f,%d%n",
                                currentTimestamp,
                                report.getRobotID(),
                                poseString,
                                report.getPathIndex(),
                                report.getVelocity(),
                                report.getDistanceTraveled(),
                                report.getCriticalPoint());
                        writer.write(rowData);
                    }
                }
            } catch (IOException e) {
                e.printStackTrace();
            }
        };
        executor.scheduleWithFixedDelay(task, 0, intervalInMillis, TimeUnit.MILLISECONDS);
    }
}
