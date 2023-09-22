package se.oru.coordination.coordination_oru.util.gates;

import se.oru.coordination.coordination_oru.util.Printer;

/**
 * This describes a thread synchronized with other `GatedThread`s.
 */
abstract public class GatedThread extends Thread {
    /**
     * If `false`, all `GatedThread`s are in the normal mode (they behave exactly as normal `Thread`s).
     * Otherwise, they are in the gated mode.
     */
    protected static boolean isGated = false;

    public static double sleepSpeedRate = 10;

    /**
     * The synchronizer of `GatedThread`s.
     */
    protected static Gatekeeper gatekeeper;

    /**
     * This makes `gatedThread.start()` not to return until the thread has started and added itself to
     * the `gatekeeper`'s queue.
     */
    protected Gate gateStart;

    /**
     * Unless this is called, `GatedThread` behaves exactly as original `Thread`.
     */
    public static void enable() {
        isGated = true;
        gatekeeper = new Gatekeeper();
    }

    /**
     * Whether the gated mode is active.
     */
    public static boolean isEnabled() {
        return isGated;
    }

    /**
     * If the gated mode is used, this should be called once at the end of the code for the main thread.
     */
    public static void runGatekeeper() {
        if (isGated) {
            gatekeeper.run();
        }
    }

    /**
     * Create the gated thread. For simplicity, only one `Thread` constructor is exposed.
     * @param name Thread name (used in debugging output).
     */
    public GatedThread(String name) {
        super(name);
    }

    /**
     * The method should implement what the method `run` of normal `Thread`s would.
     * <p>
     * To run normal `Thread`s, the following code was ordinarily used in the project:
     *
     * <pre>
     * {@code
     *    new Thread("my thread") {
     *        @Override
     *        public void run() {
     *            while (true) {
     *                work();
     *                sleep(...);
     *            }
     *        }
     *    }.start();
     * }
     * </pre>
     *
     * For the gated mode to work, that has been rewritten to the following:
     *
     * <pre>
     * {@code
     *    new GatedThread("my thread") {
     *        @Override
     *        public void runCore() {
     *            while (true) {
     *                work();
     *                sleep(...);
     *            }
     *        }
     *    }.start();
     * }
     * </pre>
     */
    abstract public void runCore();

    /**
     * Start the thread. In the gated mode, this function doesn't return until the created thread has entered its
     * `run()` and added itself to the `gatekeeper`'s queue. So there is no race between children of the current
     * thread: they work in the order of their starting.
     */
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

    /**
     * As with normal `Thread`s, starting a gated thread causes the object's `run` method to be called in that
     * separately executing thread.
     * <p>In the normal mode, just the defined <code>runCore</code> is called.
     * In the gated mode, the following happens:</p>
     * <ul>
     * <li>The thread adds itself to the <code>gatekeeper</code>'s queue and waits in the queue.</li>
     * <li><code>runCore</code> executes, usually with the following loop:<ul>
     * <li>work;</li>
     * <li>sleep: the thread adds itself to the <code>gatekeeper</code>&#39;s queue and waits in the queue.</li>
     * </ul>
     * </li>
     * <li>Before <code>run</code> returns, the thread allows <code>gatekeeper</code> to execute another one (but the current thread doesn't
     * add itself to the queue anymore).</li>
     * </ul>
     */
    @Override
    public void run() {
        if (! isGated) {
            runCore();
            return;
        }

        gatekeeper.pauseCurrentThread("initial", true, false, gateStart);
        Printer.print("ready");

        if (gatekeeper.trackDeath) {
            try {
                runCore();
            } catch (Throwable e) {
                gatekeeper.hasSomeoneDied = true;
                throw e;
            } finally {
                moveToNextGate();
            }
        } else { // for better debugging experience
            runCore();
            moveToNextGate();
        }
    }

    /**
     * In the normal mode, this is just `Thread.sleep`.
     * In the gated mode, this is to pass control to another gated thread.
     *
     * @param millis Milliseconds to sleep. In the gated mode, a speed-up modifier is applied.
     * @throws InterruptedException
     */
    public static void sleep(long millis) throws InterruptedException {
        if (isGated) {
            gatekeeper.pauseCurrentThread("sleep(" + millis + ")", false, false,null);
            if (! Timekeeper.isTimekeeperActive()) {
                Thread.sleep(Math.round(millis / sleepSpeedRate)); // TODO: This is for the browser to catch up.
            }
        } else {
            Thread.sleep(millis);
        }
    }

    /**
     * A utility method to avoid <code>try-catch</code> around <code>sleep()</code>.
     * @param millis
     */
    public static void sleepWithoutTryCatch(long millis) {
        try {
            sleep(millis);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }

    /**
     * Skip a deterministic amount of simulation time.
     * A `GatedThread` cycle repeats when any given gated thread executes again.
     *
     * @param numCycles The number of `GatedThread` cycles.
     */
    public static void skipCycles(int numCycles) {
        for (int i = 1; i <= numCycles; i++) {
            sleepWithoutTryCatch(i);
        }
    }

    /**
     * This method is called in the beginning of some code to synchronize the code with gated threads.
     *
     * @see #moveToNextGate()
     */
    public static void awaitCurrentGate() {
        if (isGated) {
            gatekeeper.pauseCurrentThread("initial", true, true, null);
        }
    }

    /**
     * This method is called at the end of some code to synchronize the code with gated threads.
     *
     * @see #awaitCurrentGate()
     */
    public static void moveToNextGate() {
        if (isGated) {
            Printer.print("finished");
            gatekeeper.processNextGate();
        }
    }
}
