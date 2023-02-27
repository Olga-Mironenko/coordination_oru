package se.oru.coordination.coordination_oru.util.gates;

import se.oru.coordination.coordination_oru.util.Printer;

abstract public class GatedThread extends Thread {
    protected static boolean isGated = false;
    protected static Gatekeeper gatekeeper;

    /**
     * This function must be called before any other use of `GatedThread`.
     */
    public static void enable() {
        isGated = true;
        gatekeeper = new Gatekeeper();
    }

    public static void runGatekeeper() {
        if (isGated) {
            gatekeeper.run();
        }
    }

    public GatedThread(String name) {
        super(name);
    }

    abstract public void runCore();

    @Override
    public void run() {
        if (isGated) {
            gatekeeper.pauseCurrentThread("initial", true); // at the beginning of each thread
            Printer.print("ready");
        }

        runCore();

        if (isGated) {
            Printer.print("finished");
            gatekeeper.processNextGate(); // at the end of each thread
        }
    }

    public static void sleep(long millis) throws InterruptedException {
        if (isGated) {
            gatekeeper.pauseCurrentThread("sleep(" + millis + ")", false);
        } else {
            Thread.sleep(millis);
        }
    }
}
