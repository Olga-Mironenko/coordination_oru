package se.oru.coordination.coordination_oru.tests.util;

import se.oru.coordination.coordination_oru.CriticalSection;
import se.oru.coordination.coordination_oru.simulation2D.AdaptiveTrajectoryEnvelopeTrackerRK4;
import se.oru.coordination.coordination_oru.util.BrowserVisualization;
import se.oru.coordination.coordination_oru.util.HumanControl;
import se.oru.coordination.coordination_oru.util.Missions;
import se.oru.coordination.coordination_oru.util.Printer;
import se.oru.coordination.coordination_oru.util.gates.GatedThread;
import se.oru.coordination.coordination_oru.util.gates.Timekeeper;

public abstract class Demo {
    protected abstract void run(String scenarioString);

    public void exec() {
        checkForAssertions();

        Printer.resetTime();
        Printer.print("started");

        boolean isOriginalTracker = false;

        if (! isOriginalTracker) {
            AdaptiveTrajectoryEnvelopeTrackerRK4.isEnabledGlobally = true;
            GatedThread.enable();

            //HumanControl.isEnabledForBrowser = true;
            BrowserVisualization.isExtendedText = true;
            AdaptiveTrajectoryEnvelopeTrackerRK4.isRacingThroughCrossroadAllowed = false;
            CriticalSection.isCanPassFirstActive = true;
            AdaptiveTrajectoryEnvelopeTrackerRK4.isReplanningNearParkedVehicle = false;
        }

        BrowserVisualization.isStatusText = true;
        Missions.isStatistics = true;

        Demo thisDemo = this;

        new Timekeeper().start();

        new GatedThread("demo.run") {
            @Override
            public void runCore() {
                thisDemo.run(System.getenv().get("SCENARIO"));
            }
        }.start();

        try {
            GatedThread.runGatekeeper();
        } catch (InterruptedException e) {
            return;
        }
    }

    private void checkForAssertions() {
        try {
            assert false;
        }
        catch (AssertionError e) {
            return;
        }
        throw new RuntimeException("assertions are disabled (add `-ea` to VM options)");
    }
}
