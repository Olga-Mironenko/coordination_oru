package se.oru.coordination.coordination_oru.tests;

import org.metacsp.multi.spatioTemporal.paths.Pose;
import se.oru.coordination.coordination_oru.code.*;
import se.oru.coordination.coordination_oru.simulation2D.TrajectoryEnvelopeCoordinatorSimulation;
import se.oru.coordination.coordination_oru.tests.util.Demo;
import se.oru.coordination.coordination_oru.tests.util.GridMapConstants;
import se.oru.coordination.coordination_oru.util.BrowserVisualization;
import se.oru.coordination.coordination_oru.util.HumanControl;
import se.oru.coordination.coordination_oru.util.MissionBlueprint;
import se.oru.coordination.coordination_oru.util.Missions;
import se.oru.coordination.coordination_oru.util.gates.GatedThread;
import se.oru.coordination.coordination_oru.util.gates.Timekeeper;

import java.awt.*;

public class GeneratedMapTest {
    public static void main(String[] args) {
        new Demo() {
            @Override
            protected void run(String scenarioString) {
                runDemo(scenarioString);
            }
        }.exec();
    }
    protected static void runDemo(String scenarioString) {
        if (scenarioString == null) {
            scenarioString = "map-generator/generated-maps/current/scenario1.yaml";
        }

        HumanControl.isEnabledForBrowser = true;
        Timekeeper.setVirtualMinutesPassedMax(60);

        final String YAML_FILE = scenarioString;
        Missions.setMap(YAML_FILE);

        final Pose humStart = Missions.getLocationPose("D1");
        final Pose humFinish = GridMapConstants.turnAround(Missions.getLocationPose("D2"));

        final double maxVelocityHum = 3.0;
        final double maxAccelerationHum = 2.0;

        VehicleSize vehicleSizeHum = new VehicleSize(1, 0.5, 0, 0, 0, 0);

        AutonomousVehicle hum0 = new HumanDrivenVehicle(0, 0, Color.ORANGE, Color.ORANGE, maxVelocityHum, maxAccelerationHum);

        TrajectoryEnvelopeCoordinatorSimulation tec = new TrajectoryEnvelopeCoordinatorSimulation(2000, 1000, 0, 0);
        tec.setupSolver(0, 100000000);
        tec.startInference();

        hum0.registerInTec(tec, vehicleSizeHum);

        tec.placeRobot(hum0.getID(), humStart);

        Heuristics heuristics = new Heuristics();
        tec.addComparator(heuristics.automatedFirst());

        tec.setUseInternalCriticalPoints(false);
        tec.setYieldIfParking(false);
        tec.setBreakDeadlocks(true, false, false);

        var viz = new BrowserVisualization();
        viz.setInitialTransform(7.0, 5.0, 5.0);
        tec.setVisualization(viz);

        Missions.startMissionDispatcher(tec);

        new GatedThread("enqueue thread") { // path planning takes a while
            @Override
            public void runCore() {
                Missions.loopMissions.put(hum0.getID(), true);
                Missions.enqueueMissions(
                        new MissionBlueprint(hum0, humStart, humFinish).setDirection(
                                MissionBlueprint.Direction.FORWARD_BACKWARD_SEPARATE_MISSIONS
                        )
                );
            }
        }.start();
    }
}