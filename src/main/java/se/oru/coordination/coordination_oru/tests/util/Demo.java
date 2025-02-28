package se.oru.coordination.coordination_oru.tests.util;

import jep.Interpreter;
import jep.SharedInterpreter;

import jep.python.PyObject;
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
import java.util.Map;
import java.util.LinkedHashMap;

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

    private void runInterpreter() {
        Interpreter interp = new SharedInterpreter();

        interp.exec("from java.lang import System");
        interp.exec("s = 'Hello World'");
        interp.exec("System.out.println(s + ' from println')");
        interp.exec("print(s + ' from print')");

        interp.exec("import os");
        interp.exec("System.out.println(os.getcwd())");

        interp.exec("import sys");
        interp.exec("System.out.println(str(sys.version))");
        interp.exec("sys.path = ['', '/home/olga/miniconda3/lib/python312.zip', '/home/olga/miniconda3/lib/python3.12', '/home/olga/miniconda3/lib/python3.12/lib-dynload', '/home/olga/miniconda3/lib/python3.12/site-packages']");
        interp.exec("sys.path.append(os.getcwd() + '/scenario-analysis')");
        interp.exec("System.out.println('Python Executable: ' + sys.executable)");
        interp.exec("System.out.println('Python Path: ' + str(sys.path))");

        interp.exec("import autogluon");
        interp.exec("System.out.println(str(autogluon))");

        interp.exec("import autogluon.tabular");
        interp.exec("System.out.println(str(autogluon.tabular))");

        interp.exec("import pandas as pd");
        interp.exec("System.out.println(str(pd))");

        Map<String, Double> record = new LinkedHashMap<>();
        record.put("a", 10.0);
        record.put("b", 20.0);

        interp.set("record", record);
        interp.exec("System.out.println(f'{record=}, {type(record)=}')");
        interp.exec("record = dict(record)");
        interp.exec("System.out.println(f'{record=}, {type(record)=}')");

        interp.exec("type_ = type");
        PyObject recordType = (PyObject) interp.invoke("type_", record);
        System.out.println("recordType: " + recordType.toString());

        interp.exec("bool_ = bool");
        boolean recordBoolean = (boolean) interp.invoke("bool_", record);
        System.out.println("recordBoolean: " + recordBoolean);

        interp.exec("import recommenderlib");
        interp.exec("System.out.println(recommenderlib.make_greeting())");

        interp.exec("import time");

        interp.exec("recommender = recommenderlib.ForcingReactionRecommender(path_collision='scenario-analysis/AutogluonModels/ag-20250227_141815')");
        interp.exec("record_input_without_is_stop = {'(in) No. of OPs': 2.0, '(in) are_bridges': 0.0, '(in) slowness': 0.0, '(in) V: v_current': 0.1, '(in) V0: v_current': 5.6, '(in) V: POD': 0.0, '(in) V0: POD': 0.0, '(in) event_distanceToCS': 482.258, '(in) event_distanceToCSEnd': 768.382, '(in) event_distanceHumanToCP': 0.0, '(in) indicesHumanToCS': 0.0, '(in) indicesHumanToCSEnd': 59.0, '(in) POD C next 20': 0.0, '(in) POD C next 50': 0.0, '(in) POD C next 100': 0.0, '(in) POD C next 200': 0.0, '(in) POD C CS segment with human': 0.0, '(in) POD Df next 20': 0.0, '(in) POD Df next 50': 0.0, '(in) POD Df next 100': 0.0, '(in) POD Df next 200': 0.0, '(in) POD Df CS segment with human': 0.0}");

        for (int i = 0; i < 5; i++) {
            interp.exec("start = time.time()");
            interp.exec("is_stop_recommended = recommender.is_stop_recommended(record_input_without_is_stop)");
            interp.exec("delta = time.time() - start");
            interp.exec("System.out.println(f'{is_stop_recommended=}, {delta=}')");
        }

        System.exit(0);
    }

    public void exec() {
        checkForAssertions();

        runInterpreter();

        if (! Containerization.IS_CONTAINER) {
            long pid = ProcessHandle.current().pid();
//        runProcess("renice --priority=-5 --pid " + pid);
//        runProcess("ionice --class=realtime --classdata=1 --pid " + pid);
        } else {
            System.setProperty("java.awt.headless", "true"); // https://stackoverflow.com/a/21309150
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
