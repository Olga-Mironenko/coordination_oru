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

public class DynamicMineMap {
    enum Scenario {
        AUTOMATED_FIRST,
        HUMAN_FIRST,
        FIRST_COME,
    }

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
            scenarioString = Scenario.AUTOMATED_FIRST.toString();
        }
        Scenario scenario = Scenario.valueOf(scenarioString);
        AbstractVehicle.scenarioId = String.valueOf(scenario);

        HumanControl.isEnabledForBrowser = true;
//        BrowserVisualization.isExtendedText = false;

//        Timekeeper.setVirtualSecondsPassedMax(15);
        Timekeeper.setVirtualMinutesPassedMax(60);
//        Timekeeper.realMillisPassedMax = 10 * 1000;

        final String YAML_FILE = "maps/mine-map-test.yaml";

        final Pose mainTunnelLeft = new Pose(4.5,14.8, GridMapConstants.thetaRight);
        final Pose mainTunnelRight = new Pose(80.05,24.75, Math.PI);
        final Pose orePass = new Pose(54.11,11.34,-Math.PI/2);
        final Pose drawPoint15 = new Pose(9.9,84.5,-Math.PI/2);
        final Pose drawPoint16 = new Pose(17.1,84.6,-Math.PI/2);
        final Pose drawPoint17 = new Pose(24.3,85.45,-Math.PI/2);
        final Pose drawPoint18 = new Pose(31.6,84.6,GridMapConstants.thetaUp);
        final Pose drawPoint19 = new Pose(39.05,85.45,-Math.PI/2);
        final Pose drawPoint19_bottom = new Pose(38.8,28.6,-Math.PI/2);
        final Pose mainTunnelBetween19And20 = new Pose(43.57,17.85, -Math.PI);
        final Pose drawPoint20 = new Pose(46.0,85.2,-Math.PI/2);
        final Pose drawPoint20_bottom = new Pose(46.0,31.0,-Math.PI/2);
        final Pose drawPoint21 = new Pose(53.3,86.8,-Math.PI/2);
        final Pose drawPoint21_bottom = new Pose(53.3,32.9,-Math.PI/2);
        final Pose drawPoint22 = new Pose(60.3,86.9,-Math.PI/2);
        final Pose drawPoint22_bottom = new Pose(60.3,33.5,-Math.PI/2);
        final Pose drawPoint23 = new Pose(67.8,85.9,-Math.PI/2);
        final Pose drawPoint23_bottom = new Pose(67.8,37.9,-Math.PI/2);
        final Pose drawPoint24 = new Pose(75.1,83.5,-Math.PI/2);
        final Pose drawPoint24_bottom = new Pose(75.1,39.1,-Math.PI/2);
        final Pose drawPoint36 = new Pose(7.5,45.4,-Math.PI/2);
        final Pose drawPoint37 = new Pose(19.4,34.5,-Math.PI/2);
        final Pose drawPoint38 = new Pose(20.1,25.7,-Math.PI/2);
        final Pose orePassOppositePoint = new Pose(53,32.4,-Math.PI/2);

        final Pose humStart = drawPoint20;
        final Pose humFinish = new Pose(46.7,19.2, GridMapConstants.thetaRight);
//        final Pose humFinish = drawPoint21;
        final boolean ishumReturn = false;
        final boolean ishumLoop = false;
        final Pose aut1Start = mainTunnelRight;
        final Pose aut1Finish = drawPoint19_bottom;
        final Pose aut2Start = mainTunnelLeft;
        final Pose aut2Finish = drawPoint18;
        final Pose aut3Start = drawPoint22;
        final Pose aut3Finish = drawPoint23;

        final double maxVelocityHum = 3.0;
        final double maxAccelerationHum = 2.0;
        final double maxAccelerationAut = 0.4;

        VehicleSize vehicleSizeHum = new VehicleSize(2, 1.5, 0, 0, 0, 0);
        VehicleSize vehicleSizeAut1 = vehicleSizeHum;
        VehicleSize vehicleSizeAut2 = vehicleSizeHum;
        VehicleSize vehicleSizeAut3 = vehicleSizeHum;

        // TODO: `maxAcceleration` passed here is not used by `tec`.
        AutonomousVehicle hum0 = new HumanDrivenVehicle(0, 0, Color.ORANGE, Color.ORANGE, maxVelocityHum, maxAccelerationHum);
//        AutonomousVehicle hum0 = new AutonomousVehicle(0, 0, Color.BLUE, Color.BLUE, maxVelocityHum, maxAccelerationHum);
        AutonomousVehicle aut1 = new AutonomousVehicle(1, 0, Color.BLUE, Color.BLUE, 10, maxAccelerationAut);
        AutonomousVehicle aut2 = new AutonomousVehicle(2, 0, Color.BLUE, Color.BLUE, 10, maxAccelerationAut);
        AutonomousVehicle aut3 = new AutonomousVehicle(3, 0, Color.BLUE, Color.BLUE, 10, maxAccelerationAut);

        TrajectoryEnvelopeCoordinatorSimulation tec = new TrajectoryEnvelopeCoordinatorSimulation(2000, 1000, 0, 0);
        tec.setupSolver(0, 100000000);
        tec.startInference();

        hum0.registerInTec(tec, vehicleSizeHum);
        aut1.registerInTec(tec, vehicleSizeAut1);
        aut2.registerInTec(tec, vehicleSizeAut2);
        aut3.registerInTec(tec, vehicleSizeAut3);

        tec.placeRobot(hum0.getID(), humStart);
        tec.placeRobot(aut1.getID(), aut1Start);
        tec.placeRobot(aut2.getID(), aut2Start);
        tec.placeRobot(aut3.getID(), aut3Start);

        Heuristics heuristics = new Heuristics();
        switch (scenario) {
            case AUTOMATED_FIRST:
            default:
                tec.addComparator(heuristics.automatedFirst());
                break;
            case HUMAN_FIRST:
                tec.addComparator(heuristics.humanFirst());
                break;
            case FIRST_COME:
                tec.addComparator(heuristics.closest());
                break;
        }

        tec.setUseInternalCriticalPoints(false);
        tec.setYieldIfParking(false);
        tec.setBreakDeadlocks(true, false, false);

        Missions.setMap(YAML_FILE); // before BrowserVisualization

        var viz = new BrowserVisualization();
        viz.setInitialTransform(7.0, 32.0, -8.5);
        tec.setVisualization(viz);

        Missions.startMissionDispatcher(tec);

        new GatedThread("enqueue thread") { // path planning takes a while
            @Override
            public void runCore() {
                Missions.loopMissions.put(hum0.getID(), false);
                Missions.enqueueMissions(
                        new MissionBlueprint(hum0, humStart, humFinish)
                );

                Missions.loopMissions.put(aut1.getID(), true);
                Missions.enqueueMissions(
                        new MissionBlueprint(aut1, aut1Start, aut1Finish).setDirection(
                                MissionBlueprint.Direction.FORWARD_BACKWARD_SEPARATE_MISSIONS
                        )
                );

                Missions.loopMissions.put(aut2.getID(), true);
                Missions.enqueueMissions(
                        new MissionBlueprint(aut2, aut2Start, aut2Finish).setDirection(
                                MissionBlueprint.Direction.FORWARD_BACKWARD_SEPARATE_MISSIONS
                        )
                );

                Missions.loopMissions.put(aut3.getID(), true);
                Missions.enqueueMissions(
                        new MissionBlueprint(aut3, aut3Start, aut3Finish).setDirection(
                                MissionBlueprint.Direction.FORWARD_BACKWARD_SEPARATE_MISSIONS
                        )
                );
            }
        }.start();
    }
}
