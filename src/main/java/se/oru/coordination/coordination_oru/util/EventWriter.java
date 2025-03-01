package se.oru.coordination.coordination_oru.util;

import se.oru.coordination.coordination_oru.code.AbstractVehicle;
import se.oru.coordination.coordination_oru.code.Entry;
import se.oru.coordination.coordination_oru.code.VehiclesHashMap;
import se.oru.coordination.coordination_oru.simulation2D.TrajectoryEnvelopeCoordinatorSimulation;
import se.oru.coordination.coordination_oru.util.gates.Timekeeper;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

public class EventWriter {
    private static BufferedWriter bw = null;
    private static boolean isHeaderWritten = false;

    public static void activate() {
        /*synchronized (EventWriter.class) {*/ { // for better debugging
            try {
                File dir = AbstractVehicle.prepareRundir();
                File file = new File(dir + "/events.tsv");
                FileWriter fw = new FileWriter(file.getAbsoluteFile(), false);
                bw = new BufferedWriter(fw);
            } catch (IOException e) {
                throw new RuntimeException(e.getMessage());
            }
        }
    }

    public static LinkedHashMap<String, String> makeMapEvent(Event event) {
        TrajectoryEnvelopeCoordinatorSimulation tec = TrajectoryEnvelopeCoordinatorSimulation.tec;
        LinkedHashMap<Entry<String, Integer>, String> mapStats = new LinkedHashMap<>();
        for (int robotID : tec.getAllRobotIDs()) {
            LinkedHashMap<Entry<String, Integer>, String> mapStatsRobot =
                    VehiclesHashMap.getVehicle(robotID).collectStatistics();
            mapStats.putAll(mapStatsRobot);
        }

        LinkedHashMap<String, String> mapEvent = new LinkedHashMap<>();
        mapEvent.put("secondsVirtual", String.format("%6.1f", Timekeeper.getVirtualMillisPassed() / 1000.0));
        mapEvent.put("event", event.toJson());
        for (boolean isNull : List.of(true, false)) {
            for (Map.Entry<Entry<String, Integer>, String> entry : mapStats.entrySet()) {
                Entry<String, Integer> key = entry.getKey();
                if ((key.getValue() == null) == isNull) {
                    String name = key.getKey();
                    if (key.getValue() != null) {
                        name = "V" + key.getValue() + ": " + name;
                    }
                    mapEvent.put(name, entry.getValue());
                }
            }
        }
        return mapEvent;
    }

    public static void writeEvent(Event event) {
        /*synchronized (EventWriter.class) {*/ { // for better debugging
            if (bw == null) {
                return;
            }

            LinkedHashMap<String, String> mapEvent = makeMapEvent(event);

            try {
                if (! isHeaderWritten) {
                    bw.write(String.join("\t", mapEvent.keySet()) + "\n");
                    isHeaderWritten = true;
                }

                bw.write(String.join("\t", mapEvent.values()) + "\n");

                if (! Containerization.IS_CONTAINER) {
                    bw.flush();
                }
            } catch (IOException e) {
                throw new RuntimeException(e.getMessage());
            }
        }
    }

    public static void close() {
        /*synchronized (EventWriter.class) {*/ { // for better debugging
            if (bw != null) {
                try {
                    bw.close();
                } catch (IOException e) {
                    throw new RuntimeException(e.getMessage());
                }
            }
        }
    }
}
