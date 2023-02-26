package se.oru.coordination.coordination_oru.util.gates;

import se.oru.coordination.coordination_oru.util.Printer;

abstract public class GatedThread extends Thread {
    protected static Gatekeeper gatekeeper = new Gatekeeper();

    public static Gatekeeper getGatekeeper() {
        return gatekeeper;
    }

    public GatedThread(String name) {
        super(name);
    }

    abstract public void runCore();

    @Override
    public void run() {
        gatekeeper.pauseCurrentThread("initial", true); // at the beginning of each thread
        Printer.print("ready");

        runCore();

        Printer.print("finished");
        gatekeeper.processNextGate(); // at the end of each thread
    }

    public static void sleep(long millis) throws InterruptedException {
        gatekeeper.pauseCurrentThread("sleep(" + millis + ")", false);
    }
}
