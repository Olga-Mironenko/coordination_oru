package se.oru.coordination.coordination_oru.util.gates;

import java.util.Calendar;

public class Timekeeper extends GatedThread {
    static int numCyclesPassed = 0;
    static final int virtualMillisPerCycle = 100;
    static int realMillisPassed = 0;
    static boolean hasCreated = false; // singleton

    public Timekeeper() {
        super("Timekeeper");
        assert ! hasCreated;
        hasCreated = true;
    }

    public static int getNumCyclesPassed() {
        return numCyclesPassed;
    }

    public static int getVirtualMillisPassed() {
        return numCyclesPassed * virtualMillisPerCycle;
    }

    public static int getRealMillisPassed() {
        return realMillisPassed;
    }

    public static boolean isTimekeeperActive() {
        return hasCreated;
    }

    @Override
    public void runCore() {
        long millisStarted = Calendar.getInstance().getTimeInMillis();
        while (true) {
            skipCycles(1);
            numCyclesPassed++;
            realMillisPassed = (int) (Calendar.getInstance().getTimeInMillis() - millisStarted);
            int millisToSleep = getVirtualMillisPassed() - getRealMillisPassed() * 2;
            if (millisToSleep > 0) {
                try {
                    Thread.sleep(millisToSleep);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
            }
        }
    }
}
