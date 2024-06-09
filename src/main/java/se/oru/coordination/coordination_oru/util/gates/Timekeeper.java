package se.oru.coordination.coordination_oru.util.gates;

import java.util.Calendar;

public class Timekeeper extends GatedThread {
    static int timestepsPassed = 0;
    static final int virtualMillisPerTimestep = 100;
    static int realMillisPassed = 0;
    static boolean hasCreated = false;
    static boolean isSingleSleep = false;

    public Timekeeper() {
        super("Timekeeper");
        assert ! hasCreated;
        hasCreated = true;
    }

    public static int getTimestepsPassed() {
        return timestepsPassed;
    }

    public static int getVirtualMillisPassed() {
        return timestepsPassed * virtualMillisPerTimestep;
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
            skipTimesteps(1);
            timestepsPassed++;
            realMillisPassed = (int) (Calendar.getInstance().getTimeInMillis() - millisStarted);
            int millisToSleep = isSingleSleep ? getVirtualMillisPassed() - getRealMillisPassed() * 2 : 0;
            if (millisToSleep > 0) {
                try {
                    Thread.sleep(millisToSleep);
                } catch (InterruptedException e) {
                    return;
                }
            }
        }
    }
}
