package se.oru.coordination.coordination_oru.utility;

import java.io.*;

/**
 * A utility class that writes output to both the console and a text file.
 * The output file is specified during the creation of the utility instance.
 *
 * @author amn
 */
public class DualOutput {

    private final PrintStream originalSystemOut;
    private final PrintStream dualPrintStream;

    /**
     * Constructs a new DualOutputUtility instance with the specified output file name.
     *
     * @param fileName The name of the output file.
     * @throws FileNotFoundException if the specified file cannot be opened or created.
     */
    public DualOutput(String fileName) throws FileNotFoundException {
        // Store the original System.out
        originalSystemOut = System.out;

        // Create a file output stream for the text file
        FileOutputStream fileOutputStream = new FileOutputStream(fileName);

        // Create a print stream for the file
        PrintStream filePrintStream = new PrintStream(fileOutputStream);

        // Create a custom print stream that writes to both the console and the file
        dualPrintStream = new PrintStream(new DualOutputStream(System.out, filePrintStream));

        // Set the custom print stream as the new System.out
        System.setOut(dualPrintStream);
    }

    /**
     * Prints the specified message to both the console and the output file.
     *
     * @param message The message to print.
     */
    public void println(String message) {
        System.out.println(message);
    }

    /**
     * Closes the utility, restoring the original System.out and closing the output file.
     */
    public void close() {
        dualPrintStream.close();
        System.setOut(originalSystemOut);
    }

    /**
     * A custom OutputStream that writes to two OutputStreams simultaneously.
     */
    static class DualOutputStream extends OutputStream {
        OutputStream outputStream1;
        OutputStream outputStream2;

        /**
         * Constructs a new DualOutputStream with the specified OutputStreams.
         *
         * @param outputStream1 The first OutputStream.
         * @param outputStream2 The second OutputStream.
         */
        public DualOutputStream(OutputStream outputStream1, OutputStream outputStream2) {
            this.outputStream1 = outputStream1;
            this.outputStream2 = outputStream2;
        }

        @Override
        public void write(int b) throws IOException {
            outputStream1.write(b);
            outputStream2.write(b);
        }

        @Override
        public void write(byte[] b) throws IOException {
            outputStream1.write(b);
            outputStream2.write(b);
        }

        @Override
        public void write(byte[] b, int off, int len) throws IOException {
            outputStream1.write(b, off, len);
            outputStream2.write(b, off, len);
        }

        @Override
        public void flush() throws IOException {
            outputStream1.flush();
            outputStream2.flush();
        }

        @Override
        public void close() throws IOException {
            outputStream1.close();
            outputStream2.close();
        }
    }
}
