package se.oru.coordination.coordination_oru.util.gates;

import se.oru.coordination.coordination_oru.util.Printer;

import java.util.concurrent.LinkedBlockingDeque;

public class Gatekeeper {
    /**
     * The queue of gates related to waiting gated threads.
     */
    protected LinkedBlockingDeque<Gate> gates = new LinkedBlockingDeque<>();
    /**
     * The gate of the gatekeeper itself.
     */
    protected Gate gateSelf;

    public boolean hasSomeoneDied = false;
    public final boolean trackDeath = false;

    public boolean isOver = false;

    /**
     * This method passes control from a gated thread to the gatekeeper.
     */
    public void processNextGate() {
        assert gateSelf != null;
        gateSelf.push();
    }

    /**
     * This method is called by a gated thread to register itself in the queue and to pass control to the gatekeeper.
     * @param nameStep The name of the current step of the gated thread (useful for debugging).
     * @param isThreadInitialization Whether this method is called at the beginning of gated thread execution.
     * @param isQueueHead Whether to add the gate to the beginning of the queue (to resume the corresponding thread ASAP).
     * @param gateStart An optional gate to push after the queue is updated. It's essentially to guarantee the following:
     *                  if no gated thread executes or has gates in the queue, then there'll be no gated thread execution
     *                  anymore.
     */
    public void pauseCurrentThread(String nameStep, boolean isThreadInitialization, boolean isQueueHead, Gate gateStart) throws InterruptedException {
        Gate gate = new Gate(Thread.currentThread().getName() + "'s " + nameStep);
        if (isQueueHead) {
            gates.addFirst(gate);
        } else {
            gates.add(gate);
        }
        if (gateStart != null) {
            gateStart.push();
        }

        if (! isThreadInitialization) {
            processNextGate();
        }
        gate.await();
    }

    /**
     * The method should be called at the end of the main thread of the program.
     * <p>
     * Here we execute gated threads registered in the queue (one by one).
     */
    public void run() throws InterruptedException {
        assert gateSelf == null;

        while (! gates.isEmpty()) {
            gateSelf = new Gate("gatekeeper's");

            Gate gate = gates.poll(); // get the first gate of the queue
            assert gate != null;

            Printer.print("SWITCHING TO " + gate.name);
            gate.push();

            // Note: By this moment, `gateSelf.push()` may have already happened.
            gateSelf.await(); // Wait for the pushed thread to pause/finish.

            if (hasSomeoneDied) {
                //System.exit(1);

                System.err.println("someone has died, sleeping indefinitely");
                while (true) {
                    try {
                        Thread.sleep(Long.MAX_VALUE);
                    } catch (InterruptedException e) {
                        return;
                    }
                }
            }
        }
    }
}
