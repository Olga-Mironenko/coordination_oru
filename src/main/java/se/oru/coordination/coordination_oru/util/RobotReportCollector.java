package se.oru.coordination.coordination_oru.util;

import se.oru.coordination.coordination_oru.AbstractTrajectoryEnvelopeTracker;
import se.oru.coordination.coordination_oru.TrajectoryEnvelopeCoordinator;

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
 * for all robots and writes the data into separate .csv files.
 *
 * @author anm
 */
public class RobotReportCollector {

    /**
     * Handles the periodic collection of robot reports and writes the data into individual .csv files per robot.
     * The method will terminate automatically after the specified termination time.
     * The timestamps in the output file will be rounded to seconds, and the 'T' character will be replaced with '-'.
     * Files will be stored in a directory specified by 'folderName', appended with the current date and time, and each file will be named by the RobotID.
     *
     * @param tec                  The TrajectoryEnvelopeCoordinator instance that contains the trackers.
     * @param folderName           The base name of the directory where to store the robot data files.
     * @param intervalInSeconds    The time interval (in seconds) between collecting and writing robot data.
     * @param terminationInMinutes The termination time (in minutes) for stopping the data collection.
     */
    public void handleRobotReports(TrajectoryEnvelopeCoordinator tec, String folderName, long intervalInSeconds, long terminationInMinutes) {
        ScheduledExecutorService executor = Executors.newSingleThreadScheduledExecutor();
        AtomicLong startTime = new AtomicLong(System.currentTimeMillis());

        // Convert interval and termination times to milliseconds
        long intervalInMillis = TimeUnit.SECONDS.toMillis(intervalInSeconds);
        long terminationInMillis = TimeUnit.MINUTES.toMillis(terminationInMinutes);

        // Generate the folder name with date and timestamp
        String timestamp = LocalDateTime.now().format(DateTimeFormatter.ofPattern("yyyyMMdd-HHmmss"));
        String directoryName = folderName + timestamp;

        // Create the directory if it doesn't exist
        Path directoryPath = Path.of(directoryName);
        try {
            if (!Files.exists(directoryPath)) {
                Files.createDirectories(directoryPath);
            }
        } catch (IOException e) {
            e.printStackTrace();
        }

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

                // Get the current timestamp rounded to seconds and replace 'T' with '/'
                String currentTimestamp = LocalDateTime.now()
                        .truncatedTo(ChronoUnit.SECONDS)
                        .format(DateTimeFormatter.ISO_LOCAL_DATE_TIME)
                        .replace('T', '/');

                // Iterate through the trackers and get the robot reports
                for (AbstractTrajectoryEnvelopeTracker tracker : trackers) {
                    var report = tracker.getRobotReport();
                    var pose = report.getPose();

                    // Generate the filename with RobotID
                    String fileName = "Robot_" + report.getRobotID() + ".csv";
                    Path filePath = directoryPath.resolve(fileName);

                    // Create and/or open the file in appended mode
                    try (BufferedWriter writer = Files.newBufferedWriter(filePath, StandardOpenOption.CREATE, StandardOpenOption.APPEND)) {
                        // Write the header if it's a new file
                        if (Files.size(filePath) == 0) {
                            writer.write("Timestamp;RobotID;Pose_X;Pose_Y;Pose_Theta;PathIndex;Velocity;DistanceTraveled;CriticalPoint\n");
                        }

                        String rowData = String.format("%s;%d;%.2f;%.2f;%.2f;%d;%.2f;%.2f;%d%n",
                                currentTimestamp,
                                report.getRobotID(),
                                pose.getX(),
                                pose.getY(),
                                pose.getTheta(),
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