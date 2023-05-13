package se.oru.coordination.coordination_oru.utility.gates;

import se.oru.coordination.coordination_oru.utility.Printer;

import java.util.concurrent.CountDownLatch;

public class Gate {
    protected String name;
    protected CountDownLatch latch = new CountDownLatch(1);
    protected boolean isPushed = false;
    protected boolean isAwaited = false;

    public Gate(String name) {
        this.name = name;
    }

    @Override
    public String toString() {
        return "Gate[" + name + "]";
    }

    public void print(String message) {
        Printer.print(String.format("%-8s | %s", name, message));
    }

    public void push() {
        assert !isPushed;

        latch.countDown();
        print("pushed");

        isPushed = true;
    }

    public void await() {
        assert !isAwaited;

        print("awaiting");
        try {
            latch.await();
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        print("awaited");

        isAwaited = true;
    }
}
