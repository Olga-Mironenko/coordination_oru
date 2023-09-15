package se.oru.coordination.coordination_oru.tests.util;

import se.oru.coordination.coordination_oru.util.BrowserVisualization;
import se.oru.coordination.coordination_oru.util.Missions;
import se.oru.coordination.coordination_oru.util.Printer;
import se.oru.coordination.coordination_oru.util.gates.GatedThread;

public abstract class Demo {
    protected abstract void run(String scenarioString);

    public void exec() {
        Printer.resetTime();
        Printer.print("started");

        BrowserVisualization.isStatusText = true;
        GatedThread.enable();
        Missions.isWriteStatistics = true;

        Demo thisDemo = this;

        new GatedThread("demo.run") {
            @Override
            public void runCore() {
                thisDemo.run(System.getenv().get("SCENARIO"));
            }
        }.start();

        GatedThread.runGatekeeper();
    }
}
