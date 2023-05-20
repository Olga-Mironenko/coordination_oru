package se.oru.coordination.coordination_oru.simulator;


import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.Geometry;
import com.vividsolutions.jts.geom.GeometryFactory;
import com.vividsolutions.jts.geom.Polygon;
import com.vividsolutions.jts.geom.util.AffineTransformation;
import org.eclipse.jetty.server.Server;
import org.eclipse.jetty.server.ServerConnector;
import org.eclipse.jetty.servlet.ServletContextHandler;
import org.eclipse.jetty.servlet.ServletHolder;
import org.eclipse.jetty.websocket.api.RemoteEndpoint;
import org.metacsp.multi.spatial.DE9IM.GeometricShapeDomain;
import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.PoseSteering;
import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelope;
import se.oru.coordination.coordination_oru.robots.AbstractRobot;
import se.oru.coordination.coordination_oru.robots.RobotHashMap;
import se.oru.coordination.coordination_oru.tracker.TrajectoryEnvelopeTrackerRK4;
import se.oru.coordination.coordination_oru.utility.*;
import se.oru.coordination.coordination_oru.utility.gates.GatedThread;

import javax.imageio.ImageIO;
import java.awt.*;
import java.awt.image.BufferedImage;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.net.URI;
import java.net.URISyntaxException;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashMap;
import java.util.HashSet;

public class BrowserVisualization implements FleetVisualization {

    public static boolean isStatusText = false;
    private static int UPDATE_PERIOD = 30;
    private final ArrayList<String> msgQueue = new ArrayList<String>();
    private double robotFootprintArea = -1;
    private double robotFootprintXDim = -1;
    private String overlayText = null;

    public BrowserVisualization() {
        this("localhost", 30);
    }

    public BrowserVisualization(String serverHostNameOrIP) {
        this(serverHostNameOrIP, 30);
    }

    public BrowserVisualization(int updatePeriodInMillis) {
        this("localhost", updatePeriodInMillis);
    }

    public BrowserVisualization(String serverHostNameOrIP, int updatePeriodInMillis) {
        UPDATE_PERIOD = updatePeriodInMillis;
        BrowserVisualization.setupVizMessageServer();
        Thread updateThread = new GatedThread("Visualization update thread") {
            @Override
            public void runCore() {
                while (true) {
                    sendMessages();
                    try {
                        GatedThread.sleep(UPDATE_PERIOD);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                }
            }
        };
        updateThread.start();
        BrowserVisualization.setupVizServer(serverHostNameOrIP);
        startOpenInBrowser(serverHostNameOrIP);
    }

    private static int getScreenDPI() {
        //Dimension screen = java.awt.Toolkit.getDefaultToolkit().getScreenSize();
        //System.out.println("Screen width: "+screen.getWidth());
        //System.out.println("Screen height: "+screen.getHeight());
        int pixelPerInch = java.awt.Toolkit.getDefaultToolkit().getScreenResolution();
        //System.out.println("DPI: " + pixelPerInch);
        return pixelPerInch;
    }

    private static double getScreenHeight() {
        Dimension screen = java.awt.Toolkit.getDefaultToolkit().getScreenSize();
        return screen.getHeight();
    }

    private static void setupVizServer(String serverHostNameOrIP) {
        Server server = new Server(8080);
        server.setHandler(new BrowserVisualizationServer(serverHostNameOrIP));
        try {
            server.start();
            //server.join();
        } catch (Throwable t) {
            t.printStackTrace(System.err);
        }
    }

    private static void setupVizMessageServer() {
        Server server = new Server();
        ServerConnector connector = new ServerConnector(server);
        connector.setPort(8081);
        server.addConnector(connector);

        // Setup the basic application "context" for this application at "/"
        // This is also known as the handler tree (in jetty speak)
        ServletContextHandler context = new ServletContextHandler(ServletContextHandler.SESSIONS);
        context.setContextPath("/");
        server.setHandler(context);

        // Add a websocket to a specific path spec
        ServletHolder holderEvents = new ServletHolder("ws-events", BrowserVisualizationServlet.class);
        context.addServlet(holderEvents, "/fleet-events/*");

        try {
            server.start();
            server.dump(System.err);
            //server.join();
        } catch (Throwable t) {
            t.printStackTrace(System.err);
        }
    }

    protected static double round(double value) {
        return (double) Math.round(value * 10) / 10;
    }

    protected static String stringifyMissions(ArrayList<Mission> missions) {
        if (missions == null) {
            missions = new ArrayList<Mission>();
        }
        synchronized (missions) {
            String text = missions.size() + " future missions: [";
            for (int i = 0; i < missions.size(); i++) {
                if (i > 0) {
                    text += ", ";
                }
                Mission mission = missions.get(i);
                text += mission.getPath().length;
            }
            text += "]";
            return text;
        }
    }

    protected static String stringifyCriticalSections(HashSet<CriticalSection> allCriticalSections) {
        ArrayList<CriticalSection> criticalSections = new ArrayList<>(allCriticalSections);
        criticalSections.sort(new Comparator<CriticalSection>() {
            @Override
            public int compare(CriticalSection cs1, CriticalSection cs2) {
                int[] list1 = csToInts(cs1);
                int[] list2 = csToInts(cs2);
                for (int i = 0; i < list1.length; i++) {
                    int value = Integer.compare(list1[i], list2[i]);
                    if (value != 0)
                        return value;
                }
                return 0;
            }

            private int[] csToInts(CriticalSection cs) {
                return new int[]{
                        (cs.getTe1() == null ? -1 : cs.getTe1().getRobotID()),
                        cs.getTe1Start(),
                        cs.getTe1End(),
                        (cs.getTe2() == null ? -1 : cs.getTe2().getRobotID()),
                        cs.getTe2Start(),
                        cs.getTe2End(),
                };
            }
        });

        String text = "Critical sections:";
        if (criticalSections.isEmpty()) {
            text += " none<br>";
        } else {
            text += "<br>";
            for (CriticalSection cs : criticalSections) {
                text += "- " + stringifyCriticalSection(cs) + "<br>";
            }
        }
        return text;
    }

    protected static String stringifyCriticalSection(CriticalSection cs) {
        synchronized (cs) {
            String ret = "";
            String robot1 = cs.getTe1() == null ? "null" : String.valueOf(cs.getTe1().getRobotID());
            String robot2 = cs.getTe2() == null ? "null" : String.valueOf(cs.getTe2().getRobotID());
            ret += robot1 + " [" + cs.getTe1Start() + ";" + cs.getTe1End() + "], " + robot2 + " [" + cs.getTe2Start() + ";" + cs.getTe2End() + "]";
            return ret;
        }
    }

    private void startOpenInBrowser(String serverHostNameOrIP) {
        if (Desktop.isDesktopSupported() && Desktop.getDesktop().isSupported(Desktop.Action.BROWSE)) {
            try {
                Desktop.getDesktop().browse(new URI("http://" + serverHostNameOrIP + ":8080"));
            } catch (IOException e) {
                e.printStackTrace();
            } catch (URISyntaxException e) {
                e.printStackTrace();
            }
        }
    }

    private void updateOverlayText() {
        if (this.overlayText != null) {
            String jsonString = "{ \"operation\" : \"setOverlayText\","
                    + "\"data\" : "
                    + "{ \"text\" : \"" + this.overlayText + "\" }}";
            sendMessage(jsonString);
        }
    }

    public void setOverlayText(String text) {
        this.overlayText = text;
    }

    private void updateRobotFootprintArea(Geometry geom) {
        if (robotFootprintArea == -1) {
            robotFootprintArea = geom.getArea();
            double minX = Double.MAX_VALUE;
            double maxX = Double.MIN_VALUE;
            for (Coordinate coord : geom.getCoordinates()) {
                if (coord.x < minX) minX = coord.x;
                if (coord.x > maxX) maxX = coord.x;
            }
            this.robotFootprintXDim = maxX - minX;
        }
    }

    public void setInitialTransform(double scale, double xTrans, double yTrans) {
        BrowserVisualizationSocket.initialScale = scale;
        BrowserVisualizationSocket.initialTranslation = new Coordinate(xTrans, yTrans);
    }

    public void setFontScale(double scale) {
        BrowserVisualizationSocket.fontScale = scale;
    }

    public void guessInitialTransform(double robotDimension, Pose... robotPoses) {
        BrowserVisualizationSocket.initialScale = getScreenDPI() / robotDimension;
        double avgX = 0;
        double avgY = 0;
        for (int i = 0; i < robotPoses.length; i++) {
            avgX += robotPoses[i].getX();
            avgY += robotPoses[i].getY();
        }
        avgX /= robotPoses.length;
        avgY /= robotPoses.length;
        avgY -= 0.45 * (getScreenHeight() / getScreenDPI());
        BrowserVisualizationSocket.initialTranslation = new Coordinate(avgX, avgY);
    }

    private void enqueueMessage(String message) {
        if (BrowserVisualizationSocket.ENDPOINTS != null && BrowserVisualizationSocket.ENDPOINTS.size() > 0) {
            synchronized (BrowserVisualizationSocket.ENDPOINTS) {
                this.msgQueue.add(message);
            }
        }
    }

    private void sendMessages() {
        if (BrowserVisualizationSocket.ENDPOINTS != null && BrowserVisualizationSocket.ENDPOINTS.size() > 0) {
            synchronized (BrowserVisualizationSocket.ENDPOINTS) {
                for (String message : this.msgQueue) {
                    sendMessage(message);
                }
                msgQueue.clear();
                updateOverlayText();
                sendUpdate();
            }
        }
    }

    private void sendMessage(String text) {
        if (BrowserVisualizationSocket.ENDPOINTS != null) {
            for (RemoteEndpoint rep : BrowserVisualizationSocket.ENDPOINTS) {
                try {
                    rep.sendString(text);
                } catch (IOException e) {
                    e.printStackTrace();
                }
            }
        }
    }

    /**
     * Displays the state of the robot, including its position, path index, and additional status information.
     *
     * @param te              The TrajectoryEnvelope of the robot.
     * @param rr              The RobotReport containing the robot's current report.
     * @param extraStatusInfo Additional status information to be displayed.
     */
    @Override
    public void displayRobotState(TrajectoryEnvelope te, RobotReport rr, String... extraStatusInfo) {
        double x, y, theta;

        if (rr.getPathIndex() != -1) {
            x = rr.getPose().getX();
            y = rr.getPose().getY();
            theta = rr.getPose().getTheta();
        } else {
            x = te.getTrajectory().getPose()[0].getX();
            y = te.getTrajectory().getPose()[0].getY();
            theta = te.getTrajectory().getPose()[0].getTheta();
        }

        String name = "R" + (rr.getPathIndex() != -1 ? te.getRobotID() : rr.getRobotID());

        // TODO Implement percentage complete

        // Show path index
        StringBuilder extraData = new StringBuilder(" : " + rr.getPathIndex());
        if (extraStatusInfo != null) {
            for (String st : extraStatusInfo) {
                extraData.append(" | ").append(st);
            }
        }

        // Default robot color is red
        String color = "#ff0000";
        int robotCount = RobotHashMap.getList().keySet().size();
        if (robotCount != 0) {
            color = RobotHashMap.getRobot(rr.getRobotID()).getColorCode();
        }

        Geometry geom = TrajectoryEnvelope.getFootprint(te.getFootprint(), x, y, theta);
        updateRobotFootprintArea(geom);
        double scale = Math.sqrt(robotFootprintArea) * 0.2;
        Geometry arrowGeom = createArrow(rr.getPose(), robotFootprintXDim / scale, scale);
        String jsonString = "{ \"operation\" : \"addGeometry\", \"data\" : " + geometryToJSONString(name, geom, color, -1, true, extraData.toString()) + "}";
        String jsonStringArrow = "{ \"operation\" : \"addGeometry\", \"data\" : " + geometryToJSONString("_" + name, arrowGeom, "#ffffff", -1, true, null) + "}";
        enqueueMessage(jsonString);
        enqueueMessage(jsonStringArrow);

        if (isStatusText) {
            setStatusText();
        }
    }

    /**
     * Displays the state of the robot, including its position, path index, and additional status information.
     *
     * @param fp              The footprint of the robot.
     * @param rr              The RobotReport containing the robot's current report.
     * @param extraStatusInfo Additional status information to be displayed.
     */
    @Override
    public void displayRobotState(Polygon fp, RobotReport rr, String... extraStatusInfo) {
        double x = rr.getPose().getX();
        double y = rr.getPose().getY();
        double theta = rr.getPose().getTheta();

        String name = "R" + rr.getRobotID();

        // Show path index
        StringBuilder extraData = new StringBuilder(" : " + rr.getPathIndex());
        if (extraStatusInfo != null) {
            for (String st : extraStatusInfo) {
                extraData.append(" | ").append(st);
            }
        }

        // Default robot color is red
        String color = "#ff0000";
        int robotCount = RobotHashMap.getList().keySet().size();
        if (robotCount != 0) {
            color = RobotHashMap.getRobot(rr.getRobotID()).getColorCode();
        }

        Geometry geom = TrajectoryEnvelope.getFootprint(fp, x, y, theta);
        updateRobotFootprintArea(geom);
        double scale = Math.sqrt(robotFootprintArea) * 0.2;
        Geometry arrowGeom = createArrow(rr.getPose(), robotFootprintXDim / scale, scale);
        String jsonString = "{ \"operation\" : \"addGeometry\", \"data\" : " + geometryToJSONString(name, geom, color, -1, true, extraData.toString()) + "}";
        String jsonStringArrow = "{ \"operation\" : \"addGeometry\", \"data\" : " + geometryToJSONString("_" + name, arrowGeom, "#ffffff", -1, true, null) + "}";
        enqueueMessage(jsonString);
        enqueueMessage(jsonStringArrow);
    }

    protected void setStatusText() {
        HashMap<Integer, AbstractRobot> idToRobot = RobotHashMap.getList();
        StringBuilder text = new StringBuilder();
        if (idToRobot.containsKey(MissionUtils.idHuman)) {
            text.append("targetVelocityHuman: ").append(round(MissionUtils.targetVelocityHuman)).append("<br>");
        }
        for (int id : idToRobot.keySet()) {
            text.append("(Robot ").append(id).append(") ");

            RobotReport rr = TrajectoryEnvelopeCoordinatorSimulation.tec.getRobotReport(id);
            if (rr == null) {
                text.append("no robot report");
            } else {
                double velocity = rr.getVelocity();
                text.append("velocity: ").append(round(velocity));

                int numCalls = 0;
                var numIntegrateCalls = TrajectoryEnvelopeCoordinatorSimulation.tec.numIntegrateCalls;
                if (numIntegrateCalls.containsKey(id)) {
                    numCalls = numIntegrateCalls.get(id);
                }
                text.append("; numIntegrateCalls: ").append(numCalls);
            }

            text.append("; ").append(stringifyMissions(Missions.getMissions(id)));

            text.append("<br>");
        }
        text.append(stringifyCriticalSections(TrajectoryEnvelopeCoordinatorSimulation.tec.allCriticalSections));
        text.append(TrajectoryEnvelopeTrackerRK4.emergencyBreaker.toString()).append("<br>");
        setOverlayText(text.toString());
    }

    public void addPath(String pathName, PoseSteering[] ps, double arrowLength, String color) {
        for (int i = 0; i < ps.length; i++) {
            Geometry arrowGeom = createArrow(ps[i].getPose(), arrowLength, 0.2 * arrowLength);
            String jsonStringArrow = "{ \"operation\" : \"addGeometry\", \"data\" : " + this.geometryToJSONString("_" + pathName + "_" + i, arrowGeom, color, -1, true, null) + "}";
            enqueueMessage(jsonStringArrow);
        }
    }

    public void removePath(String pathName, PoseSteering[] ps) {
        for (int i = 0; i < ps.length; i++) {
            String jsonString = "{ \"operation\" : \"removeGeometry\"," + "\"data\" : " + "{ \"name\" : \"" + "_" + pathName + "_" + i + "\" }}";
            enqueueMessage(jsonString);
        }
    }

    @Override
    public void displayDependency(RobotReport rrWaiting, RobotReport rrDriving, String dependencyDescriptor) {
        Geometry arrow = createArrow(rrWaiting.getPose(), rrDriving.getPose());
        String jsonString = "{ \"operation\" : \"addGeometry\", \"data\" : " + this.geometryToJSONString(dependencyDescriptor, arrow, "#adccff", 1000, true, null) + "}";
        enqueueMessage(jsonString);
    }

    private String geometryToJSONString(String name, Geometry geom, String color, long age, boolean filled, String extraData) {
        String ret = "{ \"name\" : \"" + name + "\", \"color\" : \"" + color + "\", ";
        if (age > 0) ret += " \"age\" : " + age + ", ";
        ret += " \"filled\" : " + filled + ", ";
        if (extraData != null && !extraData.trim().equals("")) ret += " \"extraData\" : \"" + extraData + "\", ";
        ret += "\"coordinates\" : [";
        Coordinate[] coords = geom.getCoordinates();
        for (int i = 0; i < coords.length; i++) {
            ret += "{\"x\" : " + coords[i].x + ", \"y\" : " + coords[i].y + "}";
            if (i < coords.length - 1) ret += ", ";
        }
        ret += "]}";
        return ret;
    }

    // TODO Implement safety distances

    @Override
    public void addEnvelope(TrajectoryEnvelope te) {

        // Color the trajectory envelope with the same vehicle color
        String color = "#efe007";
        if (!RobotHashMap.getList().isEmpty()) {
            color = RobotHashMap.getRobot(te.getRobotID()).getColorCode();
        }

        GeometricShapeDomain dom = (GeometricShapeDomain) te.getEnvelopeVariable().getDomain();
        Geometry geom = dom.getGeometry();
        String jsonString = "{ \"operation\" : \"addGeometry\", \"data\" : " + this.geometryToJSONString("_" + te.getID(), geom, color, -1, false, null) + "}";
        enqueueMessage(jsonString);
    }

    @Override
    public void removeEnvelope(TrajectoryEnvelope te) {
        String jsonString = "{ \"operation\" : \"removeGeometry\","
                + "\"data\" : "
                + "{ \"name\" : \"" + "_" + te.getID() + "\" }}";
        enqueueMessage(jsonString);
    }

    @Override
    public void updateVisualization() {
        // This method does nothing - reason:
        // Viz change events are buffered and sent by an internal thread
        // in bursts every UPDATE_PERIOD ms to avoid blocking of RemoteEndpoints
    }

    public void updateFontScale(double scale) {
        String jsonString = "{ \"operation\" : \"updateFontScale\","
                + "\"data\" : "
                + "{ \"value\" : \"" + scale + "\" }}";
//		String jsonString = "{ \"operation\" : \"updateFontScale\" }";

        enqueueMessage(jsonString);
    }

    public void sendUpdate() {
        String callRefresh = "{ \"operation\" : \"refresh\" }";
        sendMessage(callRefresh);
    }

    private Geometry createArrow(Pose pose, double length, double size) {
        GeometryFactory gf = new GeometryFactory();
        double aux = 1.8;
        double aux1 = 0.8;
        double aux2 = 0.3;
        double theta = pose.getTheta();
        Coordinate[] coords = new Coordinate[8];
        coords[0] = new Coordinate(0.0, -aux2);
        coords[1] = new Coordinate(length - aux, -aux2);
        coords[2] = new Coordinate(length - aux, -aux1);
        coords[3] = new Coordinate(length, 0.0);
        coords[4] = new Coordinate(length - aux, aux1);
        coords[5] = new Coordinate(length - aux, aux2);
        coords[6] = new Coordinate(0.0, aux2);
        coords[7] = new Coordinate(0.0, -aux2);
        Polygon arrow = gf.createPolygon(coords);
        AffineTransformation at = new AffineTransformation();
        at.scale(size, size);
        at.rotate(theta);
        at.translate(pose.getX(), pose.getY());
        Geometry ret = at.transform(arrow);
        return ret;
    }

    private Geometry createArrow(Pose pose1, Pose pose2) {
        GeometryFactory gf = new GeometryFactory();
        double aux = 1.8;
        double aux1 = 0.8;
        double aux2 = 0.3;
        double factor = Math.sqrt(robotFootprintArea) * 0.5;
        double distance = Math.sqrt(Math.pow((pose2.getX() - pose1.getX()), 2) + Math.pow((pose2.getY() - pose1.getY()), 2)) / factor;
        double theta = Math.atan2(pose2.getY() - pose1.getY(), pose2.getX() - pose1.getX());
        Coordinate[] coords = new Coordinate[8];
        coords[0] = new Coordinate(0.0, -aux2);
        coords[1] = new Coordinate(distance - aux, -aux2);
        coords[2] = new Coordinate(distance - aux, -aux1);
        coords[3] = new Coordinate(distance, 0.0);
        coords[4] = new Coordinate(distance - aux, aux1);
        coords[5] = new Coordinate(distance - aux, aux2);
        coords[6] = new Coordinate(0.0, aux2);
        coords[7] = new Coordinate(0.0, -aux2);
        Polygon arrow = gf.createPolygon(coords);
        AffineTransformation at = new AffineTransformation();
        at.scale(factor, factor);
        at.rotate(theta);
        at.translate(pose1.getX(), pose1.getY());
        Geometry ret = at.transform(arrow);
        return ret;
    }

    public void setMap(BufferedImage mapImage, double resolution, Coordinate origin) {
        BrowserVisualizationSocket.map = mapImage;
        BrowserVisualizationSocket.resolution = resolution;
        BrowserVisualizationSocket.origin = origin;
    }

    @Override
    public void setMap(String mapYAMLFile) {
        try {
            File file = new File(mapYAMLFile);
            BufferedReader br = new BufferedReader(new FileReader(file));
            String imageFileName = null;
            String st;
            //FIXME Handle map origin
            //Coordinate bottomLeftOrigin = null;
            while ((st = br.readLine()) != null) {
                if (!st.trim().startsWith("#") && !st.trim().isEmpty()) {
                    String key = st.substring(0, st.indexOf(":")).trim();
                    String value = st.substring(st.indexOf(":") + 1).trim();
                    if (key.equals("image")) imageFileName = file.getParentFile() + File.separator + value;
                    else if (key.equals("resolution"))
                        BrowserVisualizationSocket.resolution = Double.parseDouble(value);
                    else if (key.equals("origin")) {
                        String x = value.substring(1, value.indexOf(",")).trim();
                        String y = value.substring(value.indexOf(",") + 1, value.indexOf(",", value.indexOf(",") + 1)).trim();
                        BrowserVisualizationSocket.origin = new Coordinate(Double.parseDouble(x), Double.parseDouble(y));
                        //bottomLeftOrigin = new Coordinate(Double.parseDouble(x),Double.parseDouble(y));
                    }
                }
            }
            br.close();
            BrowserVisualizationSocket.map = ImageIO.read(new File(imageFileName));
            //BrowserVisualizationSocket.origin = new Coordinate(bottomLeftOrigin.x, BrowserVisualizationSocket.map.getHeight()*BrowserVisualizationSocket.resolution-bottomLeftOrigin.y);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public void setMapYAML(String mapYAMLSpec, String pathPrefix) {
        try {
            String imageFileName = "";
            if (pathPrefix != null) imageFileName = pathPrefix + File.separator;
            for (String st : mapYAMLSpec.split("\n")) {
                if (!st.trim().startsWith("#") && !st.trim().isEmpty()) {
                    String key = st.substring(0, st.indexOf(":")).trim();
                    String value = st.substring(st.indexOf(":") + 1).trim();
                    if (key.equals("image")) imageFileName += value;
                    else if (key.equals("resolution"))
                        BrowserVisualizationSocket.resolution = Double.parseDouble(value);
                    else if (key.equals("origin")) {
                        String x = value.substring(1, value.indexOf(",")).trim();
                        String y = value.substring(value.indexOf(",") + 1, value.indexOf(",", value.indexOf(",") + 1)).trim();
                        BrowserVisualizationSocket.origin = new Coordinate(Double.parseDouble(x), Double.parseDouble(y));
                        //bottomLeftOrigin = new Coordinate(Double.parseDouble(x),Double.parseDouble(y));
                    }
                }
            }
            System.out.println(imageFileName);
            BrowserVisualizationSocket.map = ImageIO.read(new File(imageFileName));
            //BrowserVisualizationSocket.origin = new Coordinate(bottomLeftOrigin.x, BrowserVisualizationSocket.map.getHeight()*BrowserVisualizationSocket.resolution-bottomLeftOrigin.y);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    @Override
    public int periodicEnvelopeRefreshInMillis() {
        return 1000;
    }

}
