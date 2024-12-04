package se.oru.coordination.coordination_oru.util;

public class Containerization {
    public static final String SCENARIO = getenv("SCENARIO", null);
    public static final String RUNDIRS = getenv("RUNDIRS", "logs/rundirs");

    public static final String WORKER = getenv("WORKER", "host");
    public static final boolean IS_CONTAINER = ! WORKER.equals("host");
    public static final boolean IS_VISUALIZATION =
            ! getenv("IS_VISUALIZATION", IS_CONTAINER ? "" : "1").isEmpty();

    protected static String getenv(String varName, String defaultValue) {
        if (System.getenv(varName) != null) {
            return System.getenv(varName);
        }
        return defaultValue;
    }

}
