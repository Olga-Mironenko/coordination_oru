package se.oru.coordination.coordination_oru.util;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import com.google.gson.JsonObject;
import java.util.Arrays;
import java.util.stream.Collectors;

// Base class for all events
public abstract class Event {
    private static final Gson gson = new GsonBuilder()
            .disableHtmlEscaping()
            .setPrettyPrinting()  // Pretty printing ensures spaces after commas
            .create();
    public int robotID;

    public String getType() {
        return this.getClass().getSimpleName();
    }

    // Convert event to JSON string with correct formatting
    public String toJson() {
        JsonObject jsonObject = gson.toJsonTree(this).getAsJsonObject();

        // Create a new JSON object with "type" first
        JsonObject sortedJson = new JsonObject();
        sortedJson.addProperty("type", getType()); // Add "type" first

        for (String key : jsonObject.keySet()) {
            if (!key.equals("type")) {
                sortedJson.add(key, jsonObject.get(key));
            }
        }

        // Get formatted JSON with newlines and indentations
        String formattedJson = gson.toJson(sortedJson);

        // Process: Trim leading spaces per line, then join all lines with a space
        return Arrays.stream(formattedJson.split("\n"))
                .map(String::trim)  // Trim leading spaces
                .collect(Collectors.joining(" ")); // Join lines with a space
    }

    public void write() {
        EventWriter.writeEvent(this);
    }

    public static class MissionStarted extends Event {
        public MissionStarted(int robotID) {
            this.robotID = robotID;
        }
    }

    public static class MissionFinished extends Event {
        public MissionFinished(int robotID) {
            this.robotID = robotID;
        }
    }

    public static class MinorCollision extends Event {
        public int otherID;

        public MinorCollision(int robotID, int otherID) {
            this.robotID = robotID;
            this.otherID = otherID;
        }
    }

    public static class MajorCollisionFromMinor extends Event {
        public int otherID;

        public MajorCollisionFromMinor(int robotID, int otherID) {
            this.robotID = robotID;
            this.otherID = otherID;
        }
    }

    public static class ForcingStarted extends Event {
        public boolean areStopsAllowed;

        public ForcingStarted(int robotID, boolean areStopsAllowed) {
            this.robotID = robotID;
            this.areStopsAllowed = areStopsAllowed;
        }
    }

    public static class ForcingReaction extends Event {
        public boolean isStop;

        public ForcingReaction(int robotID, boolean isStop) {
            this.robotID = robotID;
            this.isStop = isStop;
        }
    }

    public static class ForcingFinished extends Event {
        public ForcingFinished(int robotID) {
            this.robotID = robotID;
        }
    }

    public static class MaxVelocitySet extends Event {
        public double maxVelocityNew;
        public double maxVelocityOld;

        public MaxVelocitySet(int robotID, double maxVelocityNew, double maxVelocityOld) {
            this.robotID = robotID;
            this.maxVelocityNew = maxVelocityNew;
            this.maxVelocityOld = maxVelocityOld;
        }
    }

    public static class Rerouting extends Event {
        public int otherID;
        public boolean isAtParked;

        public Rerouting(int robotID, int otherID, boolean isAtParked) {
            this.robotID = robotID;
            this.otherID = otherID;
            this.isAtParked = isAtParked;
        }
    }

    public static class PassFirst extends Event {
        public int otherID;
        public String weightRobot;
        public String weightOther;

        public PassFirst(int robotID, int otherID, String weightRobot, String weightOther) {
            this.robotID = robotID;
            this.otherID = otherID;
            this.weightRobot = weightRobot;
            this.weightOther = weightOther;
        }
    }

    public static class CautiousStarted extends Event {
        public CautiousStarted(int robotID) {
            this.robotID = robotID;
        }
    }

    public static class CautiousFinished extends Event {
        public CautiousFinished(int robotID) {
            this.robotID = robotID;
        }
    }
}