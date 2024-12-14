package se.oru.coordination.coordination_oru.tests.util;

import se.oru.coordination.coordination_oru.AbstractTrajectoryEnvelopeTracker;
import se.oru.coordination.coordination_oru.CriticalSection;
import se.oru.coordination.coordination_oru.code.AutonomousVehicle;
import se.oru.coordination.coordination_oru.motionplanning.ompl.ReedsSheppCarPlanner;
import se.oru.coordination.coordination_oru.simulation2D.AdaptiveTrajectoryEnvelopeTrackerRK4;
import se.oru.coordination.coordination_oru.util.*;
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

            /// Appearance:
            GatedThread.millisArtificialSleep = 0;
            HumanControl.isEnabledForBrowser = false;
            BrowserVisualization.isExtendedText = true;
            AbstractTrajectoryEnvelopeTracker.kToRenderEveryKthFrame = 5;

            /// Path finding:
            AutonomousVehicle.isPathCachingEnabled = true;
            AutonomousVehicle.planningAlgorithm = ReedsSheppCarPlanner.PLANNING_ALGORITHM.PRMcustom;
            if (AutonomousVehicle.planningAlgorithm == ReedsSheppCarPlanner.PLANNING_ALGORITHM.PRMcustom) {
                AutonomousVehicle.numIterationsRoadmapConstruction = 4000;
                AutonomousVehicle.numIterationsPathSimplification = 1000;
            }

            AdaptiveTrajectoryEnvelopeTrackerRK4.durationStoppedMinimumForBlockDefault = 10.0;

            /// Human (mis)behavior actions:
            /// - Violation of priorities:
            AdaptiveTrajectoryEnvelopeTrackerRK4.probabilityForcingForHuman = 0.5;
            AdaptiveTrajectoryEnvelopeTrackerRK4.distanceToCPForForcing = 30.0;
            /// - Moving slowly:
            AdaptiveTrajectoryEnvelopeTrackerRK4.probabilitySlowingDownForHuman = 0.0;
            AdaptiveTrajectoryEnvelopeTrackerRK4.velocitySlowingDownForHuman = 1.5;
            AdaptiveTrajectoryEnvelopeTrackerRK4.lengthIntervalSlowingDownForHuman = 100.0;
            AdaptiveTrajectoryEnvelopeTrackerRK4.millisStopEvents = 3000; // rerouting (slow)
            AdaptiveTrajectoryEnvelopeTrackerRK4.countStopEvents = 20; // rerouting (slow)

            /// Coordination features for AVs:
            /// - Can pass first:
            CriticalSection.isCanPassFirstActiveHum = false;
            CriticalSection.isCanPassFirstActiveAut = false;
            /// - Racing through crossroad:
            AdaptiveTrajectoryEnvelopeTrackerRK4.isRacingThroughCrossroadAllowed = true;
            /// - Cautious mode:
            AdaptiveTrajectoryEnvelopeTrackerRK4.isCautiousModeAllowed = false;
            AdaptiveTrajectoryEnvelopeTrackerRK4.deltaMaxVelocityCautious = -2.0;
            AdaptiveTrajectoryEnvelopeTrackerRK4.minMaxVelocityCautious = 1.0;
            /// - Rerouting:
            AdaptiveTrajectoryEnvelopeTrackerRK4.isReroutingNearParkedVehicleForHuman = false;
            AdaptiveTrajectoryEnvelopeTrackerRK4.isReroutingNearParkedVehicleForNonHuman = false;
            AdaptiveTrajectoryEnvelopeTrackerRK4.isReroutingNearSlowVehicleForHuman = false;
            AdaptiveTrajectoryEnvelopeTrackerRK4.isReroutingNearSlowVehicleForNonHuman = false;
            /// - Change of priorities:
            Forcing.priorityDistance = 50.0;
            Forcing.isDistanceToCPAddedToPriorityDistance = true;
            /// - Stops:
            Forcing.stopDistance = 50.0;
            Forcing.isDistanceToCPAddedToStopDistance = true;
            Forcing.isGlobalTemporaryStop = false; // true for Global, false for Local
            Forcing.isResetAfterCurrentCrossroad = true;
        }

        BrowserVisualization.isStatusText = true;
        Missions.isStatisticsPeriodical = true;
        Missions.isStatisticsFinal = true;

        Demo thisDemo = this;

        new Timekeeper().start();

        new GatedThread("demo.run") {
            @Override
            public void runCore() {
                thisDemo.run(Containerization.SCENARIO);
            }
        }.start();

        try {
            GatedThread.runGatekeeper();
        } catch (InterruptedException ignored) {
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
