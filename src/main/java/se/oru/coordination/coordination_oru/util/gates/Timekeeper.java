package se.oru.coordination.coordination_oru.util.gates;

import java.io.IOException;
import java.util.Calendar;

public class Timekeeper extends GatedThread {
    static int timestepsPassed = 0;
    public static final int virtualMillisPerTimestep = 100;
    static int realMillisPassed = 0;

    public static Integer timestepsPassedMax = null;
    public static Integer realMillisPassedMax = null;

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

    public static void setVirtualSecondsPassedMax(int seconds) {
        timestepsPassedMax = seconds * 1000 / virtualMillisPerTimestep;
    }

    public static void setVirtualMinutesPassedMax(int minutes) {
        setVirtualSecondsPassedMax(minutes * 60);
    }

    protected void interruptAllThreads() {
        for (Thread thread : Thread.getAllStackTraces().keySet()) {
            // TODO: skip the current thread for the further `sleep` before `kill` to work
            thread.interrupt();
        }

        /*
        try {
            Thread.sleep(5000); // wait for everyone to finish
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
         */

        long pid = ProcessHandle.current().pid();
        try {
            Runtime.getRuntime().exec("kill " + pid);
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }

    protected static boolean isOver() {
        if (timestepsPassedMax != null && timestepsPassed >= timestepsPassedMax) {
            assert timestepsPassed == timestepsPassedMax;
            return true;
        }
        return realMillisPassedMax != null && realMillisPassed >= realMillisPassedMax;
    }

    @Override
    public void runCore() {
        long millisStarted = Calendar.getInstance().getTimeInMillis();
        while (true) {
            skipTimesteps(1);
            timestepsPassed++;
            realMillisPassed = (int) (Calendar.getInstance().getTimeInMillis() - millisStarted);

            if (isOver()) {
                interruptAllThreads();
            }

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
