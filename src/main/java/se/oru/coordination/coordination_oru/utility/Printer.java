package se.oru.coordination.coordination_oru.utility;

public class Printer {
    protected static long initialMoment = System.currentTimeMillis();

    public static void resetTime() {
        initialMoment = System.currentTimeMillis();
        System.err.println();
    }

    public static void print(String message) {
        long delta = System.currentTimeMillis() - initialMoment;
        String line = String.format("%5d ms | %-5s | %s", delta, Thread.currentThread().getName(), message);
        System.err.println(line);
    }
}