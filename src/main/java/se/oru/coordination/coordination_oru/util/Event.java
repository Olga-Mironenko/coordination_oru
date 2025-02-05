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

    public abstract String getType();

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

    // Inner class for an event with no robot ID
    public static class EventA extends Event {
        public String message;

        public EventA(String message) {
            this.message = message;
        }

        @Override
        public String getType() {
            return "event_a";
        }
    }

    // Inner class for an event with one robot ID
    public static class EventB extends Event {
        public int robotID;
        public String action;

        public EventB(int robotID, String action) {
            this.robotID = robotID;
            this.action = action;
        }

        @Override
        public String getType() {
            return "event_b";
        }
    }

    // Example usage
    public static void main(String[] args) {
        Event event1 = new EventA("System started");
        Event event2 = new EventB(42, "Move forward");

        // Serialize and print JSON
        System.out.println(event1.toJson());
        System.out.println(event2.toJson());
    }

    // Inner class for an event with one robot ID
    public static class MissionStarted extends Event {
        public int robotID;

        public MissionStarted(int robotID) {
            this.robotID = robotID;
        }

        @Override
        public String getType() {
            return "MissionStarted";
        }
    }

}