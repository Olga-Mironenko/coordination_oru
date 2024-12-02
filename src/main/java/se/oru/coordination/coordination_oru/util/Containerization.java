package se.oru.coordination.coordination_oru.util;

public class Containerization {
    public static final boolean IS_CONTAINER = System.getenv().containsKey("WORKER");
    public static final String WORKER = ! IS_CONTAINER ? "host" : System.getenv("WORKER");
}
