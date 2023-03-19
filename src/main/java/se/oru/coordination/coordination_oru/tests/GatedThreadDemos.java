package se.oru.coordination.coordination_oru.tests;

import se.oru.coordination.coordination_oru.util.gates.Gate;
import se.oru.coordination.coordination_oru.util.gates.GatedThread;
import se.oru.coordination.coordination_oru.util.Printer;

class Sleeper {
    public static void sleep(long millis) {
        try {
            Thread.sleep(millis);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }
}

class DemoPushAwait {
    public static void run() {
        Printer.resetTime();
        Printer.print("started");

        Gate gate = new Gate("gate");
        gate.push();
        gate.await();
    }
}


class DemoChild {
    public static void run() {
        Printer.resetTime();
        Printer.print("started");

        Gate gateChild = new Gate("child's");
        Gate gateMain = new Gate("main's");

        new Thread("child") {
            @Override
            public void run() {
                Printer.print("started");

                gateChild.await();
                Sleeper.sleep(100);
                gateMain.push();
            }
        }.start();

        Sleeper.sleep(50);
        gateChild.push();
        gateMain.await();
    }
}

class DemoGatekeeper {
    public static void run() {
        Printer.resetTime();
        Printer.print("started");

        GatedThread.enable();

        for (int child = 1; child <= 2; child++) {
            new GatedThread("child" + child) {
                @Override
                public void runCore() {
                    for (int step = 1; step <= 3; step++) {
                        String name = "done-" + step;
                        Sleeper.sleep(50);
                        Printer.print(name);

                        try { sleep(step); }
                        catch (InterruptedException e) { e.printStackTrace(); }
                    }
                }
            }.start();
        }

        GatedThread.runGatekeeper();
    }
}

public class GatedThreadDemos {
    public static void main(String[] args) throws InterruptedException {
        DemoPushAwait.run();
        DemoChild.run();
        DemoGatekeeper.run();
    }
}