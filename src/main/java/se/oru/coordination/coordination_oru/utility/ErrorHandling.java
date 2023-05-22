package se.oru.coordination.coordination_oru.utility;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;

/**
 * Utility class for handling errors and exceptions.
 *
 * @author anm
 */
public class ErrorHandling {

    /**
     * Handles a throwable by writing the error message to a file.
     *
     * @param resultsDirectory The directory to store the error file.
     * @param throwable        The throwable object containing the error message.
     */
    public static void handleThrowable(String resultsDirectory, Throwable throwable) {
        String errorMessage = throwable.getMessage();
        Path directoryPath = Paths.get(resultsDirectory);
        String fileName = directoryPath + "/" + "Error.txt";
        Path filePath = Paths.get(fileName);

        try {
            Files.writeString(filePath, errorMessage);
            System.out.println("The error message was written to " + fileName);
        } catch (IOException ioException) {
            System.out.println("An error occurred while writing the error message to " + fileName);
            ioException.printStackTrace();
        }
    }
}
