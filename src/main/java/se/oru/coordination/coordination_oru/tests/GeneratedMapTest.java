package se.oru.coordination.coordination_oru.tests;

import com.google.gson.JsonArray;
import org.metacsp.multi.spatioTemporal.paths.Pose;
import se.oru.coordination.coordination_oru.AbstractTrajectoryEnvelopeCoordinator;
import se.oru.coordination.coordination_oru.CriticalSection;
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

    private static String getSuffix(String string, String prefix) {
        assert string.startsWith(prefix);
        return string.substring(prefix.length());
    }

    public static String replaceUnderscoreAfterFirstComma(String input) {
        // Find the index of the first comma
        int commaIndex = input.indexOf(',');
        if (commaIndex == -1) {
            // If no comma exists, return the original string unchanged
            return input;
        }

        // Get the substring up to and including the first comma
        String beforeComma = input.substring(0, commaIndex + 1);
        beforeComma = beforeComma.replace("map-generator_", "map-generator/");
        beforeComma = beforeComma.replace("generated-maps_", "generated-maps/");
        beforeComma = beforeComma.replace("bridges_", "bridges/");

        // Get the substring after the first comma
        String afterComma = input.substring(commaIndex + 1);

        // Replace all '_' characters with a space in the afterComma part
        afterComma = afterComma.replace('_', ' ');

        // Return the concatenation of the unmodified part and the modified part
        return beforeComma + afterComma;
    }

    protected static void runDemo(String scenarioString) {
        HumanControl.isEnabledForBrowser = true;
        if (Containerization.IS_CONTAINER) {
            Timekeeper.setVirtualMinutesPassedMax(30);
        } else {
            Timekeeper.setVirtualMinutesPassedMax(30);
//            Timekeeper.setVirtualMinutesPassedMax(15);
//            Timekeeper.setVirtualSecondsPassedMax(20);
//            Timekeeper.setVirtualMinutesPassedMax(2);
        }

        Heuristics heuristics = new Heuristics();
//        Comparator<RobotAtCriticalSection> comparator = heuristics.humanFirst();
        Comparator<RobotAtCriticalSection> comparator = heuristics.automatedFirst();
//        Comparator<RobotAtCriticalSection> comparator = heuristics.closest()

        if (scenarioString == null) {
            scenarioString = (
//                    "map-generator/generated-maps/2024-11-28_13:19:18_without_bridges/scenario1-1.json, change of priorities, seed 1, probabilityForcingForHuman 1"
//                    "map-generator/generated-maps/2024-11-28_13:19:18_without_bridges/scenario1-1.json, baseline, seed 1, probabilityForcingForHuman 0"
//                    "map-generator/generated-maps/2024-11-28_13:17:39_with_bridges/scenario1-1.json, passhum 1, slowness with rerouting, forcing change of priorities"
//                    "map-generator/generated-maps/2024-11-28_13:17:39_with_bridges/scenario1-1.json, passhum 1, slowness no, forcing no"
//                    "map-generator/generated-maps/2024-11-28_13:17:39_with_bridges/scenario1-4.json, baseline, seed 1, probabilityForcingForHuman 0"
//                    "map-generator/generated-maps/2024-11-28_13:19:18_without_bridges/scenario1-5.json, passhum 0, slowness no, forcing no"
//                    "map-generator/generated-maps/2024-11-28_13:17:39_with_bridges/scenario2-3.json, passhum 0, slowness without rerouting, forcing ignoring human"
//                    "map-generator/generated-maps/2024-11-28_13:19:18_without_bridges/scenario3-1.json, passhum 0, slowness no, forcing ignoring human"
//                    "map-generator/generated-maps/2024-11-28_13:17:39_with_bridges/scenario2-3.json, passhum 0, slowness without rerouting, forcing change of priorities"
//                    "map-generator/generated-maps/2024-11-28_13:19:18_without_bridges/scenario5-1.json, passhum 0, slowness no, forcing ignoring human"
//                    "map-generator/generated-maps/2024-11-28_13:19:18_without_bridges/scenario5-1.json, passhum 0, slowness no, forcing stops"
//                    "map-generator/generated-maps/2024-11-28_13:19:18_without_bridges/scenario9-6.json, passhum 0, slowness no, forcing change of priorities"
//                    "map-generator/generated-maps/2024-11-28_13:17:39_with_bridges/scenario7-1.json, passhum 0, slowness with rerouting, forcing change of priorities"
//                    "map-generator/generated-maps/2024-11-28_13:17:39_with_bridges/scenario2-2.json, passhum 0, slowness with rerouting, forcing change of priorities"
//                    "map-generator/generated-maps/2024-11-28_13:17:39_with_bridges/scenario2-3.json, passhum 0, slowness with rerouting, forcing change of priorities"
//                    "map-generator/generated-maps/2024-11-28_13:17:39_with_bridges/scenario2-3.json, passhum 0, slowness with rerouting, forcing change of priorities"
//                    "map-generator/generated-maps/2024-11-28_13:17:39_with_bridges/scenario2-3.json, passhum 0, slowness no, forcing change of priorities"
//                    "map-generator_generated-maps_2024-11-28_13:17:39_with_bridges_scenario2-3.json,_passhum_0,_slowness_no,_forcing_change_of_priorities"
//                    "map-generator_generated-maps_2024-11-28_13:17:39_with_bridges_scenario2-3.json,_passhum_0,_slowness_no,_forcing_50%_stops"
//                    "map-generator_generated-maps_2024-11-28_13:17:39_with_bridges_scenario2-3.json,_passhum_0,_slowness_no,_forcing_stops"
//                    "map-generator_generated-maps_2024-11-28_13:17:39_with_bridges_scenario9-2.json,_passhum_0,_slowness_no,_forcing_no"
//                      "map-generator_generated-maps_2024-11-28_13:17:39_with_bridges_scenario2-2.json,_passhum_0,_slowness_with_rerouting,_forcing_change_of_priorities"
                      "map-generator/generated-maps/2024-11-28_13:17:39_with_bridges/scenario9-6.json, passhum 0, slowness no, forcing stops"
            );
        }
        scenarioString = replaceUnderscoreAfterFirstComma(scenarioString);
        AbstractVehicle.scenarioId = scenarioString;
        EventWriter.activate();

        // Parsing `scenarioString`:

        String[] scenarioTokens = scenarioString.split(", ");
        assert scenarioTokens.length == 4;

        AbstractVehicle.scenarioFilename = scenarioTokens[0];
        CriticalSection.isCanPassFirstActiveHum = getSuffix(scenarioTokens[1], "passhum ").equals("1");

        boolean isSlow = false;
        assert ! AdaptiveTrajectoryEnvelopeTrackerRK4.isReroutingAtSlowForNonHuman;
        assert ! AdaptiveTrajectoryEnvelopeTrackerRK4.isReroutingAtParkedForNonHuman;
        switch (getSuffix(scenarioTokens[2], "slowness ")) {
            case "no":
                break;
            case "without rerouting":
                isSlow = true;
                break;
            case "with rerouting":
                isSlow = true;
                AdaptiveTrajectoryEnvelopeTrackerRK4.isReroutingAtSlowForNonHuman = true;
                AdaptiveTrajectoryEnvelopeTrackerRK4.isReroutingAtParkedForNonHuman = true;
                if (AbstractVehicle.scenarioFilename.contains("_without_bridges/")) {
                    return;
                }
                break;
            default:
                throw new IllegalArgumentException();
        }

        assert AdaptiveTrajectoryEnvelopeTrackerRK4.probabilityForcingForHuman == 0;
        Forcing.stopDistance = Forcing.priorityDistance;
        switch (getSuffix(scenarioTokens[3], "forcing ")) {
            case "no":
                break;

            case "ignoring human":
                AbstractTrajectoryEnvelopeCoordinator.isHumanIgnored = true;
                // and "change of priorities"
            case "change of priorities":
                AdaptiveTrajectoryEnvelopeTrackerRK4.probabilityForcingForHuman = 1;
                AdaptiveTrajectoryEnvelopeTrackerRK4.probabilityForcingStops = 0;
                break;

            case "stops":
                AdaptiveTrajectoryEnvelopeTrackerRK4.probabilityForcingForHuman = 1;
                AdaptiveTrajectoryEnvelopeTrackerRK4.probabilityForcingStops = 1;
                break;

            case "50% stops":
                AdaptiveTrajectoryEnvelopeTrackerRK4.probabilityForcingForHuman = 1;
                AdaptiveTrajectoryEnvelopeTrackerRK4.probabilityForcingStops = 0.5;
                break;

            default:
                throw new IllegalArgumentException();
        }

        // Creating vehicles, etc.:

        int numAuts;
        double[] dimensionsVehicle;
        try (FileReader reader = new FileReader(AbstractVehicle.scenarioFilename)) {
            JsonElement jsonElement = JsonParser.parseReader(reader);

            assert jsonElement.isJsonObject();
            JsonObject jsonObject = jsonElement.getAsJsonObject();

            File file = new File(AbstractVehicle.scenarioFilename);
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
                isSlow ? 1 : 5.6, 0.3
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