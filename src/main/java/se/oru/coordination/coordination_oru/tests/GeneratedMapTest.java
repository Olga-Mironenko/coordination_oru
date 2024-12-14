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

import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import com.google.gson.JsonParser;
import se.oru.coordination.coordination_oru.util.gates.Timekeeper;

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
//        Timekeeper.setVirtualSecondsPassedMax(2);
        Timekeeper.setVirtualMinutesPassedMax(30);
//        Timekeeper.setVirtualMinutesPassedMax(1);

        Heuristics heuristics = new Heuristics();
//        Comparator<RobotAtCriticalSection> comparator = heuristics.humanFirst();
        Comparator<RobotAtCriticalSection> comparator = heuristics.automatedFirst();
//        Comparator<RobotAtCriticalSection> comparator = heuristics.closest()

        if (scenarioString == null) {
            scenarioString = (
                    "map-generator/generated-maps/2024-11-28_13:17:39_with_bridges/scenario3-4.json, " +
                            "baseline, seed 1, probabilityForcingForHuman 0"
            );
        }
        AbstractVehicle.scenarioId = String.format(
                "%s; %s",
                scenarioString,
                heuristics.getHeuristicName()
        );

        String[] scenarioTokens = scenarioString.split(", ");
        assert scenarioTokens.length == 4;

        String scenarioFilename = scenarioTokens[0];
        AbstractVehicle.scenarioFilename = scenarioFilename;

        String stringProb = scenarioTokens[3];
        final String prefixProb = "probabilityForcingForHuman ";
        assert stringProb.startsWith(prefixProb);
        AdaptiveTrajectoryEnvelopeTrackerRK4.probabilityForcingForHuman = Double.parseDouble(
                stringProb.substring(prefixProb.length()));

        String stringVariation = scenarioTokens[1];
        switch (stringVariation) {
            case "baseline":
                assert AdaptiveTrajectoryEnvelopeTrackerRK4.probabilityForcingForHuman == 0.0;
                break;
            case "change of priorities":
                Forcing.stopDistance = Double.NEGATIVE_INFINITY;
                break;
            case "stops":
                Forcing.stopDistance = Forcing.priorityDistance;
                break;
            default:
                throw new IllegalArgumentException("Unrecognized variation string: " + stringVariation);
        }

        String stringSeed = scenarioTokens[2];
        final String prefixSeed = "seed ";
        assert stringSeed.startsWith(prefixSeed);
        AdaptiveTrajectoryEnvelopeTrackerRK4.seedGlobal = Integer.parseInt(stringSeed.substring(prefixSeed.length()));

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
                5.6, 0.3
        );
        AutonomousVehicle[] auts = new AutonomousVehicle[numAuts];
        for (int i = 0; i < numAuts; i++) {
            auts[i] = new AutonomousVehicle(
                    i + 1, 0,
                    Color.BLUE, Color.BLUE,
                    5.6, 0.3
            );
        }

        TrajectoryEnvelopeCoordinatorSimulation tec = new TrajectoryEnvelopeCoordinatorSimulation(2000, 1000, 0, 0);
        tec.setQuiet(true);
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
        tec.addComparator(new Heuristics().closest());

        tec.setUseInternalCriticalPoints(false);
        tec.setYieldIfParking(true);
        tec.setBreakDeadlocks(true, false, false);

        var viz = new BrowserVisualization();
        double resolution = Missions.getDynamicMap().resolution;
        int xTransPixel = 60; // left margin
        int yTransPixel = 30; // bottom margin
        viz.setInitialTransform(0.7 / resolution, xTransPixel * resolution, yTransPixel * resolution);
        tec.setVisualization(viz);

        Missions.startMissionDispatcher(tec);

        new GatedThread("enqueue thread") { // path planning takes a while
            @Override
            public void runCore() {
//                GatedThread.skipTimesteps(100);

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