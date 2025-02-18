package se.oru.coordination.coordination_oru.util;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import com.google.gson.JsonObject;
import com.google.gson.JsonElement;
import com.google.gson.JsonSerializer;
import com.google.gson.JsonSerializationContext;
import com.google.gson.JsonPrimitive;

import java.math.BigDecimal;
import java.math.RoundingMode;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.stream.Collectors;

// Base class for all events
public abstract class Event {
    private static final Gson gson = new GsonBuilder()
            .registerTypeAdapter(Double.class, new JsonSerializer<Double>() {
                @Override
                public JsonElement serialize(Double src, java.lang.reflect.Type typeOfSrc, JsonSerializationContext context) {
                    BigDecimal roundedValue = new BigDecimal(src).setScale(3, RoundingMode.HALF_UP);
                    return new JsonPrimitive(roundedValue);
                }
            })
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
        sortedJson.addProperty("robotID", this.robotID); // Robot ID second
        sortedJson.addProperty("type", getType()); // Add "type" first

        for (String key : jsonObject.keySet()) {
            if (! key.equals("robotID") && ! key.equals("type")) {
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
        public ForcingStarted(int robotID) {
            this.robotID = robotID;
        }
    }

    public static class ForcingFinished extends Event {
        public ForcingFinished(int robotID) {
            this.robotID = robotID;
        }
    }

    public static class ForcingReactionStarted extends Event {
        public boolean isStop;
        public int indicesToCS;
        public double distanceToCS;
        public int indicesToCSEnd;
        public double distanceToCSEnd;
        public ArrayList<Double> linearizationC;
        public ArrayList<Double> linearizationDf;

        public ForcingReactionStarted(
                int robotID,
                boolean isStop,
                int indicesToCS,
                double distanceToCS,
                int indicesToCSEnd,
                double distanceToCSEnd,
                ArrayList<Double> linearizationC,
                ArrayList<Double> linearizationDf
        ) {
            this.robotID = robotID;
            this.isStop = isStop;
            this.indicesToCS = indicesToCS;
            this.distanceToCS = distanceToCS;
            this.indicesToCSEnd = indicesToCSEnd;
            this.distanceToCSEnd = distanceToCSEnd;
            this.linearizationC = linearizationC;
            this.linearizationDf = linearizationDf;
        }
    }

    public static class ForcingReactionFinished extends Event {
        public ForcingReactionFinished(int robotID) {
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