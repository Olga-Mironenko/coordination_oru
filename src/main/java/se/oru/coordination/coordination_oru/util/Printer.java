package se.oru.coordination.coordination_oru.util;

public class Printer {
    protected static long initialMoment = System.currentTimeMillis();

    public static void resetTime() {
        initialMoment = System.currentTimeMillis();
        System.err.println();
    }

    public static void print(String message) {
        long time = getMillis();
        String line = String.format("%5d ms | %-5s | %s", time, Thread.currentThread().getName(), message);
        System.err.println(line);
    }

    public static long getMillis() {
        return System.currentTimeMillis() - initialMoment;
    }
}