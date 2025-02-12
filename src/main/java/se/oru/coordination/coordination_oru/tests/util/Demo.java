package se.oru.coordination.coordination_oru.tests.util;

import se.oru.coordination.coordination_oru.AbstractTrajectoryEnvelopeCoordinator;
import se.oru.coordination.coordination_oru.AbstractTrajectoryEnvelopeTracker;
import se.oru.coordination.coordination_oru.CriticalSection;
import se.oru.coordination.coordination_oru.code.AutonomousVehicle;
import se.oru.coordination.coordination_oru.motionplanning.ompl.ReedsSheppCarPlanner;
import se.oru.coordination.coordination_oru.simulation2D.AdaptiveTrajectoryEnvelopeTrackerRK4;
import se.oru.coordination.coordination_oru.util.*;
import se.oru.coordination.coordination_oru.util.gates.GatedThread;
import se.oru.coordination.coordination_oru.util.gates.Timekeeper;

import java.io.IOException;

public abstract class Demo {
    protected abstract void run(String scenarioString);

    public static int runProcess(String... args) {
        Process process;
        try {
            ProcessBuilder builder = new ProcessBuilder(args);
            builder.redirectError(ProcessBuilder.Redirect.INHERIT);
            builder.redirectOutput(ProcessBuilder.Redirect.INHERIT);
            process = builder.start();
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
        int code;
        try {
            code = process.waitFor();
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        return code;
    }

    public void exec() {
        checkForAssertions();

        if (! Containerization.IS_CONTAINER) {
            long pid = ProcessHandle.current().pid();
//        runProcess("renice --priority=-5 --pid " + pid);
//        runProcess("ionice --class=realtime --classdata=1 --pid " + pid);
        }

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
            AdaptiveTrajectoryEnvelopeTrackerRK4.probabilityForcingForHuman = 0;
            AdaptiveTrajectoryEnvelopeTrackerRK4.distanceToCPForForcing = 30.0;
            /// - Moving slowly:
            AdaptiveTrajectoryEnvelopeTrackerRK4.probabilitySlowingDownForHuman = 0.0;
            AdaptiveTrajectoryEnvelopeTrackerRK4.velocitySlowingDownForHuman = 1.5;
            AdaptiveTrajectoryEnvelopeTrackerRK4.lengthIntervalSlowingDownForHuman = 100.0;
            AdaptiveTrajectoryEnvelopeTrackerRK4.millisStopEventsInitial = 10000; // rerouting (slow)

            /// Coordination features for AVs:
            /// - Can pass first:
            CriticalSection.isCanPassFirstActiveHum = false;
            CriticalSection.isCanPassFirstActiveAut = false;
            /// - Racing through crossroad:
            AdaptiveTrajectoryEnvelopeTrackerRK4.isRacingThroughCrossroadAllowed = true;
            /// - Cautious mode (= responsiveness to cautious situations):
            AdaptiveTrajectoryEnvelopeTrackerRK4.isCautiousMode = false;
            AdaptiveTrajectoryEnvelopeTrackerRK4.deltaMaxVelocityCautious = -2.0;
            AdaptiveTrajectoryEnvelopeTrackerRK4.minMaxVelocityCautious = 1.0;
            /// - Rerouting:
            AdaptiveTrajectoryEnvelopeTrackerRK4.isReroutingAtParkedForHuman = false;
            AdaptiveTrajectoryEnvelopeTrackerRK4.isReroutingAtParkedForNonHuman = false;
            AdaptiveTrajectoryEnvelopeTrackerRK4.millisReroutingAtParkedIfNotInDummyTracker = null;
            AdaptiveTrajectoryEnvelopeTrackerRK4.isReroutingAtSlowForHuman = false;
            AdaptiveTrajectoryEnvelopeTrackerRK4.isReroutingAtSlowForNonHuman = false;
            /// - Change of priorities:
            Forcing.priorityDistance = 50.0;
            Forcing.isDistanceToCPAddedToPriorityDistance = true;
            /// - Stops:
            Forcing.stopDistance = Forcing.priorityDistance;
            Forcing.isDistanceToCPAddedToStopDistance = true;
            Forcing.probabilityStopNotChangeOfPriorities = 0.5;
            Forcing.isGlobalTemporaryStop = false; // true for Global, false for Local
            Forcing.isResetAfterCurrentCrossroad = true;
            /// - Baseline "Ignoring Human":
            AbstractTrajectoryEnvelopeCoordinator.isHumanIgnored = false;
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
