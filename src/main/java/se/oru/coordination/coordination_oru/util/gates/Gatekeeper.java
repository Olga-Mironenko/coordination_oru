package se.oru.coordination.coordination_oru.util.gates;

import se.oru.coordination.coordination_oru.util.Printer;

import java.util.concurrent.BlockingDeque;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.LinkedBlockingDeque;
import java.util.concurrent.LinkedBlockingQueue;

public class Gatekeeper {
    protected LinkedBlockingDeque<Gate> gates = new LinkedBlockingDeque<>();
    protected Gate gateSelf;

    public void processNextGate() {
        assert gateSelf != null;
        gateSelf.push();
    }

    public void pauseCurrentThread(String nameStep, boolean isThreadInitialization, boolean isQueueHead, Gate gateStart) {
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

    public void run() {
        assert gateSelf == null;

        while (! gates.isEmpty()) {
            gateSelf = new Gate("gatekeeper's");

            Gate gate = gates.poll();
            assert gate != null;

            Printer.print("SWITCHING TO " + gate.name);
            gate.push();

            // Note: By this moment, `gateSelf.push()` may have already happened.
            gateSelf.await(); // Wait for the pushed thread to pause/finish.
        }
    }
}
