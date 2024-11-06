package se.oru.coordination.coordination_oru.tests;

import com.google.gson.JsonArray;
import org.metacsp.multi.spatioTemporal.paths.Pose;
import se.oru.coordination.coordination_oru.RobotAtCriticalSection;
import se.oru.coordination.coordination_oru.code.*;
import se.oru.coordination.coordination_oru.simulation2D.AdaptiveTrajectoryEnvelopeTrackerRK4;
import se.oru.coordination.coordination_oru.simulation2D.TrajectoryEnvelopeCoordinatorSimulation;
import se.oru.coordination.coordination_oru.tests.util.Demo;
import se.oru.coordination.coordination_oru.tests.util.GridMapConstants;
import se.oru.coordination.coordination_oru.util.*;
import se.oru.coordination.coordination_oru.util.gates.GatedThread;
import se.oru.coordination.coordination_oru.util.gates.Timekeeper;

import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import com.google.gson.JsonParser;

import java.io.File;
import java.io.FileReader;
import java.io.IOException;

import java.awt.*;
import java.util.Comparator;

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
        HumanControl.isEnabledForBrowser = true;
//        Timekeeper.setVirtualMinutesPassedMax(5);

        Heuristics heuristics = new Heuristics();
//        Comparator<RobotAtCriticalSection> comparator = heuristics.humanFirst();
        Comparator<RobotAtCriticalSection> comparator = heuristics.automatedFirst();
//        Comparator<RobotAtCriticalSection> comparator = heuristics.closest()

        if (scenarioString == null) {
            scenarioString = "map-generator/generated-maps/current/scenario1.json, without rerouting";
        }
        AbstractVehicle.scenarioId = String.format(
                "%s; %s",
                scenarioString,
                heuristics.getHeuristicName()
        );

        String[] scenarioTokens = scenarioString.split(", ");
        assert scenarioTokens.length == 2;
        String scenarioFilename = scenarioTokens[0];
        String stringRerouting = scenarioTokens[1];
        switch (stringRerouting) {
            case "with rerouting":
                AdaptiveTrajectoryEnvelopeTrackerRK4.isReroutingNearParkedVehicleForHuman = false;
                AdaptiveTrajectoryEnvelopeTrackerRK4.isReroutingNearParkedVehicleForNonHuman = true;
                AdaptiveTrajectoryEnvelopeTrackerRK4.isReroutingNearSlowVehicleForHuman = false;
                AdaptiveTrajectoryEnvelopeTrackerRK4.isReroutingNearSlowVehicleForNonHuman = true;
                break;
            case "without rerouting":
                AdaptiveTrajectoryEnvelopeTrackerRK4.isReroutingNearParkedVehicleForHuman = false;
                AdaptiveTrajectoryEnvelopeTrackerRK4.isReroutingNearParkedVehicleForNonHuman = false;
                AdaptiveTrajectoryEnvelopeTrackerRK4.isReroutingNearSlowVehicleForHuman = false;
                AdaptiveTrajectoryEnvelopeTrackerRK4.isReroutingNearSlowVehicleForNonHuman = false;
                break;
            default:
                throw new IllegalArgumentException("Unrecognized rerouting string: " + stringRerouting);
        }

        Forcing.priorityDistance = 10.0;

        int numAuts;
        double[] dimensionsVehicle;
        try (FileReader reader = new FileReader(scenarioFilename)) {
            JsonElement jsonElement = JsonParser.parseReader(reader);

            assert jsonElement.isJsonObject();
            JsonObject jsonObject = jsonElement.getAsJsonObject();

            File file = new File(scenarioFilename);
            String basenameLocations = jsonObject.get("locations").getAsString();
            Missions.loadRoadMap(file.getParentFile() + File.separator + basenameLocations);

            String basenameMapconf = jsonObject.get("mapconf").getAsString();
            Missions.setMap(file.getParentFile() + File.separator + basenameMapconf);

            numAuts = jsonObject.get("num_auts").getAsInt();

            JsonArray dimensionsVehicleArray = jsonObject.getAsJsonArray("dimensions_vehicle");
            dimensionsVehicle = new double[dimensionsVehicleArray.size()];
            for (int i = 0; i < dimensionsVehicleArray.size(); i++) {
                dimensionsVehicle[i] = dimensionsVehicleArray.get(i).getAsDouble();
            }
        } catch (IOException e) {
            e.printStackTrace();
            throw new RuntimeException(e);
        }

        final Pose humStart = Missions.getLocationPose("hum1_start");
        final Pose humFinish = GridMapConstants.turnAround(Missions.getLocationPose("hum1_finish"));
        Pose[] autsStart = new Pose[numAuts];
        Pose[] autsFinish = new Pose[numAuts];
        for (int i = 0; i < numAuts; i++) {
            String name = "aut" + (i + 1);
            autsStart[i] = Missions.getLocationPose(name + "_start");
            autsFinish[i] = GridMapConstants.turnAround(Missions.getLocationPose(name + "_finish"));
        }
//        autsStart[0] = Missions.getLocationPose("D1");
//        autsFinish[0] = GridMapConstants.turnAround(Missions.getLocationPose("D2"));
//        autsStart[1] = Missions.getLocationPose("D3");
//        autsFinish[1] = GridMapConstants.turnAround(Missions.getLocationPose("D2"));

        assert dimensionsVehicle.length == 6;
        VehicleSize vehicleSizeHum = new VehicleSize(
                dimensionsVehicle[0], dimensionsVehicle[1],
                dimensionsVehicle[2], dimensionsVehicle[3],
                dimensionsVehicle[4], dimensionsVehicle[5]
        );
        VehicleSize vehicleSizeAut = vehicleSizeHum;

        AutonomousVehicle hum = new HumanDrivenVehicle(
                0, 0,
                Color.ORANGE, Color.ORANGE,
                1.5, 3.0
        );
        AutonomousVehicle[] auts = new AutonomousVehicle[numAuts];
        for (int i = 0; i < numAuts; i++) {
            auts[i] = new AutonomousVehicle(
                    i + 1, 0,
                    Color.BLUE, Color.BLUE,
                    10.0, 3.0
            );
        }

        TrajectoryEnvelopeCoordinatorSimulation tec = new TrajectoryEnvelopeCoordinatorSimulation(2000, 1000, 0, 0);
        tec.setupSolver(0, 100000000);
        tec.startInference();

        hum.registerInTec(tec, vehicleSizeHum);
        for (int i = 0; i < numAuts; i++) {
            auts[i].registerInTec(tec, vehicleSizeAut);
        }

        tec.placeRobot(hum.getID(), humStart);
        for (int i = 0; i < numAuts; i++) {
            tec.placeRobot(auts[i].getID(), autsStart[i]);
        }
;
        tec.addComparator(comparator);

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
                Missions.loopMissions.put(hum.getID(), true);
                Missions.enqueueMissions(
                        new MissionBlueprint(hum, humStart, humFinish).setDirection(
                                MissionBlueprint.Direction.FORWARD_BACKWARD_SEPARATE_MISSIONS
                        )
                );

                for (int i = 0; i < numAuts; i++) {
                    Missions.loopMissions.put(auts[i].getID(), true);
                    Missions.enqueueMissions(
                            new MissionBlueprint(auts[i], autsStart[i], autsFinish[i]).setDirection(
                                    MissionBlueprint.Direction.FORWARD_BACKWARD_SINGLE_MISSION
                            )
                    );
                }
            }
        }.start();
    }
}