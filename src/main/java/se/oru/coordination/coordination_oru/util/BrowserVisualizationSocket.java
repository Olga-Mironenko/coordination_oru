package se.oru.coordination.coordination_oru.util;


import java.awt.image.BufferedImage;
import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.nio.ByteBuffer;
import java.util.HashSet;

import javax.imageio.ImageIO;

import org.eclipse.jetty.websocket.api.RemoteEndpoint;
import org.eclipse.jetty.websocket.api.Session;
import org.eclipse.jetty.websocket.api.WebSocketAdapter;
import org.metacsp.multi.spatioTemporal.paths.Pose;

import com.google.gson.Gson;
import com.google.gson.JsonArray;
import com.google.gson.JsonParser;
import com.vividsolutions.jts.geom.Coordinate;
import se.oru.coordination.coordination_oru.util.gates.GatedThread;

public class BrowserVisualizationSocket extends WebSocketAdapter {

    public static HashSet<RemoteEndpoint> ENDPOINTS = new HashSet<RemoteEndpoint>();
    public static BufferedImage map = null;
    public static double resolution = 1;
    public static Coordinate origin = null;
    public static double initialScale = 1;
    public static double fontScale = 0.8;
    public static Coordinate initialTranslation = null;

    @Override
    public void onWebSocketConnect(Session sess) {
        super.onWebSocketConnect(sess);
        System.out.println("Socket Connected: " + sess);
        synchronized (ENDPOINTS) {
            ENDPOINTS.add(super.getRemote());
            //Send map and map parameters if present
            if (BrowserVisualizationSocket.map != null) {
                try {
                    System.out.println("Sending map metadata to newly connected client...");
                    String setMetadataString = "{ \"operation\" : \"setMapMetadata\","
                            + "\"data\" : "
                            + "{ \"resolution\" : " + resolution + ", \"x\" : " + origin.x + ", \"y\" : " + origin.y + "}}";
                    super.getRemote().sendString(setMetadataString);

                    System.out.println("Sending map to newly connected client...");
                    ByteArrayOutputStream baos = new ByteArrayOutputStream();
                    ImageIO.write(map, "png", baos);
                    baos.flush();
                    byte[] imageInBytes = baos.toByteArray();
                    baos.close();
                    ByteBuffer bb = ByteBuffer.wrap(imageInBytes);
                    super.getRemote().sendBytes(bb);
                }
                catch(IOException e) { e.printStackTrace(); }
            }
            if (BrowserVisualizationSocket.initialTranslation != null) {
                try {
                    System.out.println("Sending initial transform to newly connected client...");
                    String setInitialTransformString = "{ \"operation\" : \"setInitialTransform\","
                            + "\"data\" : "
                            + "{ \"scale\" : " + initialScale + ", \"x\" : " + initialTranslation.x + ", \"y\" : " + initialTranslation.y + "}}";
                    super.getRemote().sendString(setInitialTransformString);
                }
                catch(IOException e) { e.printStackTrace(); }
            }
            try {
                System.out.println("Sending initial font scale to newly connected client...");
                String setFontScaleString = "{ \"operation\" : \"setFontScale\","
                        + "\"data\" : "
                        + "{ \"scale\" : " + fontScale + "}}";
                super.getRemote().sendString(setFontScaleString);
            }
            catch(IOException e) { e.printStackTrace(); }
        }
    }

    @Override
    public void onWebSocketText(String message) {
        try {
            GatedThread.awaitCurrentGate();
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }

        try {
            onWebSocketTextCore(message);
        }
        finally {
            GatedThread.moveToNextGate();
        }
    }

    private void onWebSocketTextCore(String message) {
        super.onWebSocketText(message);
        System.out.println("Received TEXT message: " + message);

        int robotID = 0; // the ID should eventually be given through the message

        Gson gson = new Gson();
        JsonArray array = new JsonParser().parse(message).getAsJsonArray();
        String event = gson.fromJson(array.get(0), String.class);
        if (HumanControl.isEnabledForBrowser && event.equals("click")) {
            Pose poseJson = gson.fromJson(array.get(1), Pose.class);
            Pose pose = new Pose(poseJson.getX(), poseJson.getY(), poseJson.getTheta());
            HumanControl.moveRobot(robotID, pose);
        } else if (HumanControl.isEnabledForBrowser && event.equals("keydown")) {
            String code = gson.fromJson(array.get(1), String.class);
            System.out.println("keydown: code=" + code);
            Double delta = null;
            if (code.equals("ArrowRight")) {
                delta = 1.0;
            } else if (code.equals("ArrowLeft")) {
                delta = -1.0;
            } else if (code.equals("Space")) {
                Forcing.startForcing(robotID);
            } else {
                System.out.println("Unknown keydown code: " + code);
            }
            if (delta != null) {
                HumanControl.changeMaxVelocity(robotID, delta);
            }
	    } else {
            System.out.println("Unknown event: " + event);
        }
    }

    @Override
    public void onWebSocketClose(int statusCode, String reason) {
        System.out.println("Removing connection to client");
        ENDPOINTS.remove(super.getRemote());
        super.onWebSocketClose(statusCode,reason);
        System.out.println("Socket Closed: [" + statusCode + "] " + reason);
    }

    @Override
    public void onWebSocketError(Throwable cause) {
        super.onWebSocketError(cause);
        cause.printStackTrace(System.err);
    }
}