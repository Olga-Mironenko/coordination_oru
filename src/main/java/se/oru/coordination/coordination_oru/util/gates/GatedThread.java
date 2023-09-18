package se.oru.coordination.coordination_oru.util.gates;

import se.oru.coordination.coordination_oru.util.Printer;

abstract public class GatedThread extends Thread {
    protected static boolean isGated = false;
    protected static Gatekeeper gatekeeper;
    protected Gate gateStart;

    /**
     * This function must be called before any other use of `GatedThread`.
     */
    public static void enable() {
        isGated = true;
        gatekeeper = new Gatekeeper();
    }

    public static boolean isEnabled() {
        return isGated;
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
    public void start() {
        if (isGated) {
            gateStart = new Gate(getName() + "'s gateStart");
        }

        super.start();

        if (isGated) {
            gateStart.await();
        }
    }

    @Override
    public void run() {
        if (isGated) {
            gatekeeper.pauseCurrentThread("initial", true, false, gateStart);
            Printer.print("ready");
        }

        runCore();

        moveToNextGate();
    }

    public static void sleep(long millis) throws InterruptedException {
        if (isGated) {
            gatekeeper.pauseCurrentThread("sleep(" + millis + ")", false, false,null);
            Thread.sleep(millis / 20); // TODO: This is for the browser to catch up.
        } else {
            Thread.sleep(millis);
        }
    }

    public static void sleepWithoutException(long millis) {
        try {
            sleep(millis);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }

    public static void skipCycles(int numCycles) {
        for (int i = 1; i <= numCycles; i++) {
            sleepWithoutException(i);
        }
    }

    public static void awaitCurrentGate() {
        if (isGated) {
            gatekeeper.pauseCurrentThread("initial", true, true, null);
        }
    }

    public static void moveToNextGate() {
        if (isGated) {
            Printer.print("finished");
            gatekeeper.processNextGate();
        }
    }
}
