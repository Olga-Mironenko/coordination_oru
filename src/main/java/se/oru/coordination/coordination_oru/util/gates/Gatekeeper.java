package se.oru.coordination.coordination_oru.util.gates;

import se.oru.coordination.coordination_oru.util.Printer;

import java.util.concurrent.BlockingQueue;
import java.util.concurrent.LinkedBlockingQueue;

public class Gatekeeper {
    protected BlockingQueue<Gate> gates = new LinkedBlockingQueue<>();
    protected Gate gateSelf;

    public void processNextGate() {
        assert gateSelf != null;
        gateSelf.push();
    }

    public void pauseCurrentThread(String nameStep, boolean isInitial) {
        Gate gate = new Gate(Thread.currentThread().getName() + "'s " + nameStep);
        gates.add(gate);

        if (!isInitial) {
            processNextGate();
        }
        gate.await();
    }

    public void run() {
        while (true) {
            assert gateSelf == null;
            gateSelf = new Gate("gatekeeper's");

            Gate gate;
            try {
                gate = gates.take(); // blocks if the queue is empty
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
            Printer.print("SWITCHING TO " + gate.name);
            gate.push();

            // Note: By this moment, `gateSelf.push()` may have already happened.
            gateSelf.await(); // Wait for the pushed thread to pause/finish.
        }
    }
}
