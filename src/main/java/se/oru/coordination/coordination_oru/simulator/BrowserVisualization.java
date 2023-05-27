package se.oru.coordination.coordination_oru.simulator;


import java.awt.Desktop;
import java.awt.Dimension;
import java.awt.image.BufferedImage;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.net.URI;
import java.net.URISyntaxException;
import java.util.ArrayList;

import javax.imageio.ImageIO;

import org.eclipse.jetty.server.Server;
import org.eclipse.jetty.server.ServerConnector;
import org.eclipse.jetty.servlet.ServletContextHandler;
import org.eclipse.jetty.servlet.ServletHolder;
import org.eclipse.jetty.websocket.api.RemoteEndpoint;
import org.metacsp.multi.spatial.DE9IM.GeometricShapeDomain;
import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.PoseSteering;
import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelope;

import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.Geometry;
import com.vividsolutions.jts.geom.GeometryFactory;
import com.vividsolutions.jts.geom.Polygon;
import com.vividsolutions.jts.geom.util.AffineTransformation;

import se.oru.coordination.coordination_oru.robots.RobotHashMap;
import se.oru.coordination.coordination_oru.utility.RobotReport;

/**
 * BrowserVisualization provides a way to visualize robot fleets and their states
 * in a web browser using Jetty websocket server.
 * It maintains a queue of messages which are periodically dispatched to the client(s) to update the visualization.
 * It's part of the FleetVisualization interface which provides functionality to display the state of a fleet of robots.
 *
 * @author fpa
 */
public class BrowserVisualization implements FleetVisualization {

    private final ArrayList<String> msgQueue = new ArrayList<String>();
    private static int UPDATE_PERIOD = 30;
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
        Thread updateThread = new Thread("Visualization update thread") {
            public void run() {
                while (true) {
                    sendMessages();
                    try { Thread.sleep(UPDATE_PERIOD); }
                    catch (InterruptedException e) { e.printStackTrace(); }
                }
            }
        };
        updateThread.start();
        BrowserVisualization.setupVizServer(serverHostNameOrIP);
        startOpenInBrowser(serverHostNameOrIP);
    }

    /**
     * Opens a web browser and navigates to a specified server host.
     *
     * @param serverHostNameOrIP The hostname or IP address of the server.
     */
    private void startOpenInBrowser(String serverHostNameOrIP) {
        if (Desktop.isDesktopSupported() && Desktop.getDesktop().isSupported(Desktop.Action.BROWSE)) {
            try { Desktop.getDesktop().browse(new URI("http://" + serverHostNameOrIP + ":8080")); }
            catch (IOException | URISyntaxException e) { e.printStackTrace(); }
        }
    }

    /**
     * Updates the overlay text on the visualization if it's not null.
     */
    private void updateOverlayText() {
        if (this.overlayText != null) {
            String jsonString = "{ \"operation\" : \"setOverlayText\","
                    + "\"data\" : "
                    + "{ \"text\" : \""+ this.overlayText + "\" }}";
            sendMessage(jsonString);
        }
    }

    /**
     * Sets the overlay text to be displayed on the visualization.
     *
     * @param text The text to be set as the overlay text.
     */
    public void setOverlayText(String text) {
        this.overlayText = text;
    }

    /**
     * Updates the robot footprint area based on the given geometry.
     * If the robot footprint area is -1, it will be set to the area of the geometry,
     * and the robot footprint's X dimension will be calculated.
     *
     * @param geometry The geometry representing the robot's footprint.
     */
    private void updateRobotFootprintArea(Geometry geometry) {
        if (robotFootprintArea == -1) {
            robotFootprintArea = geometry.getArea();
            double minX = Double.MAX_VALUE;
            double maxX = Double.MIN_VALUE;
            for (Coordinate coordinate : geometry.getCoordinates()) {
                if (coordinate.x < minX) minX = coordinate.x;
                if (coordinate.x > maxX) maxX = coordinate.x;
            }
            this.robotFootprintXDim = maxX - minX;
        }
    }

    /**
     * Sets the initial transformation parameters for the visualization.
     *
     * @param scale The initial scale of the visualization.
     * @param xTranslation The initial X translation of the visualization.
     * @param yTranslation The initial Y translation of the visualization.
     */
    public void setInitialTransform(double scale, double xTranslation, double yTranslation) {
        BrowserVisualizationSocket.initialScale = scale;
        BrowserVisualizationSocket.initialTranslation = new Coordinate(xTranslation,yTranslation);
    }

    /**
     * Sets the font scale of the visualization.
     *
     * @param scale The font scale to be set.
     */
    public void setFontScale(double scale) {
        BrowserVisualizationSocket.fontScale = scale;
    }

    /**
     * Returns the screen's dots per inch (DPI).
     *
     * @return The screen's DPI.
     */
    private static int getScreenDPI() {
        return java.awt.Toolkit.getDefaultToolkit().getScreenResolution();
    }

    /**
     * Returns the screen's height in pixels.
     *
     * @return The screen's height.
     */
    private static double getScreenHeight() {
        Dimension screen = java.awt.Toolkit.getDefaultToolkit().getScreenSize();
        return screen.getHeight();
    }

    /**
     * Guesses and sets the initial transformation parameters for the visualization.
     * The parameters are set based on the robot dimension and an average of given robot poses.
     *
     * @param robotDimension The size of the robot.
     * @param robotPoses The poses of the robot(s).
     */
    public void guessInitialTransform(double robotDimension, Pose ... robotPoses) {
        BrowserVisualizationSocket.initialScale = getScreenDPI()/robotDimension;
        double avgX = 0;
        double avgY = 0;
        for (Pose robotPose : robotPoses) {
            avgX += robotPose.getX();
            avgY += robotPose.getY();
        }
        avgX /= robotPoses.length;
        avgY /= robotPoses.length;
        avgY -= 0.45*(getScreenHeight()/getScreenDPI());
        BrowserVisualizationSocket.initialTranslation = new Coordinate(avgX,avgY);
    }

    /**
     * Sets up the visualization server.
     *
     * @param serverHostNameOrIP The hostname or IP address of the server.
     */
    private static void setupVizServer(String serverHostNameOrIP) {
        Server server = new Server(8080);
        server.setHandler(new BrowserVisualizationServer(serverHostNameOrIP));
        try {
            server.start();
        }
        catch (Throwable t) { t.printStackTrace(System.err); }
    }

    /**
     * Sets up the visualization message server.
     */
    private static void setupVizMessageServer() {
        Server server = new Server();
        ServerConnector connector = new ServerConnector(server);
        connector.setPort(8081);
        server.addConnector(connector);

        // Set up the basic application "context" for this application at "/"
        // This is also known as the handler tree (in jetty speak)
        var context = new ServletContextHandler(ServletContextHandler.SESSIONS);
        context.setContextPath("/");
        server.setHandler(context);

        // Add a websocket to a specific path spec
        var holderEvents = new ServletHolder("ws-events", BrowserVisualizationServlet.class);
        context.addServlet(holderEvents, "/fleet-events/*");

        try {
            server.start();
            server.dump(System.err);
        }
        catch (Throwable t) { t.printStackTrace(System.err); }
    }

    /**
     * Sets up the visualization message server.
     */
    private void enqueueMessage(String message) {
        if (BrowserVisualizationSocket.ENDPOINTS != null && BrowserVisualizationSocket.ENDPOINTS.size() > 0) {
            synchronized (BrowserVisualizationSocket.ENDPOINTS) {
                this.msgQueue.add(message);
            }
        }
    }

    /**
     * Sends all the messages in the queue and then clears the queue.
     * It also updates the overlay text and sends an update to the visualization.
     */
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

    /**
     * Sends a message to all registered endpoints.
     *
     * @param text The message to be sent.
     */
    private void sendMessage(String text) {
        if (BrowserVisualizationSocket.ENDPOINTS != null) {
            for (RemoteEndpoint rep : BrowserVisualizationSocket.ENDPOINTS) {
                try {
                    rep.sendString(text);
                }
                catch(IOException e) { e.printStackTrace(); }
            }
        }
    }

    /**
     * Displays the state of a robot in the trajectory envelope.
     * This includes the robot's current position, its path index, and any additional status information.
     *
     * @param trajectoryEnvelope The trajectory envelope in which the robot is moving.
     * @param robotReport The current report of the robot.
     * @param extraStatusInfo Additional status information to be displayed.
     * TODO Implement percentage complete
     * TODO May be Make color of arrow head with color of priority
     * TODO Make TE lines more bolder
     */
    @Override
    public void displayRobotState(TrajectoryEnvelope trajectoryEnvelope, RobotReport robotReport,
                                  String... extraStatusInfo) {
        double x = robotReport.getPathIndex() != -1 ? robotReport.getPose().getX() : trajectoryEnvelope.getTrajectory().
                getPose()[0].getX();
        double y = robotReport.getPathIndex() != -1 ? robotReport.getPose().getY() : trajectoryEnvelope.getTrajectory().
                getPose()[0].getY();
        double theta = robotReport.getPathIndex() != -1 ? robotReport.getPose().getTheta() : trajectoryEnvelope.getTrajectory()
                .getPose()[0].getTheta();

        String name = "R" + (robotReport.getPathIndex() != -1 ? trajectoryEnvelope.getRobotID() : robotReport.getRobotID());

        // Show path index
        var extraData = new StringBuilder(" : " + robotReport.getPathIndex());
        if (extraStatusInfo != null) {
            for (String st : extraStatusInfo) {
                extraData.append(" | ").append(st);
            }
        }

        // Default robot color is red
        String color = "#ff0000";
        int robotCount = RobotHashMap.getList().keySet().size();
        if (robotCount != 0) {
            color = RobotHashMap.getRobot(robotReport.getRobotID()).getColorCode();
        }

        Geometry geom = TrajectoryEnvelope.getFootprint(trajectoryEnvelope.getFootprint(), x, y, theta);
        this.updateRobotFootprintArea(geom);
        double scale = Math.sqrt(robotFootprintArea)*0.2;
        var arrowGeom = createArrow(robotReport.getPose(), robotFootprintXDim/scale, scale);
        String jsonString = "{ \"operation\" : \"addGeometry\", \"data\" : " + this.geometryToJSONString(name, geom, color,
                -1, true, String.valueOf(extraData)) + "}";
        String jsonStringArrow = "{ \"operation\" : \"addGeometry\", \"data\" : " + this.geometryToJSONString("_"+name,
                arrowGeom, "#ffffff", -1, true, null) + "}";
        enqueueMessage(jsonString);
        enqueueMessage(jsonStringArrow);
    }

    /**
     * Adds a path to the visualization. An arrow represents each pose in the path.
     *
     * @param pathName The name of the path.
     * @param poseSteering The poses along the path.
     * @param arrowLength The length of the arrows representing the poses.
     * @param color The color of the arrows.
     */
    public void addPath(String pathName, PoseSteering[] poseSteering, double arrowLength, String color) {
        for (int i = 0; i < poseSteering.length; i++) {
            var arrow = createArrow(poseSteering[i].getPose(), arrowLength, 0.2*arrowLength);
            String jsonStringArrow = "{ \"operation\" : \"addGeometry\", \"data\" : " + this.geometryToJSONString("_"
                    + pathName+"_" + i, arrow, color, -1, true, null) + "}";
            enqueueMessage(jsonStringArrow);
        }
    }

    /**
     * Removes a path from the visualization.
     *
     * @param pathName The name of the path.
     * @param ps The poses along the path.
     */
    public void removePath(String pathName, PoseSteering[] ps) {
        for (int i = 0; i < ps.length; i++) {
            String jsonString = "{ \"operation\" : \"removeGeometry\"," + "\"data\" : " + "{ \"name\" : \"" + "_"+pathName
                    + "_" + i + "\" }}";
            enqueueMessage(jsonString);
        }
    }

    /**
     * Displays a dependency between two robots. The dependency is represented by an arrow from the waiting robot to the driving robot.
     *
     * @param rrWaiting The report of the waiting robot.
     * @param rrDriving The report of the driving robot.
     * @param dependencyDescriptor The descriptor of the dependency.
     */
    @Override
    public void displayDependency(RobotReport rrWaiting, RobotReport rrDriving, String dependencyDescriptor) {
        Geometry arrow = createArrow(rrWaiting.getPose(), rrDriving.getPose());
        String jsonString = "{ \"operation\" : \"addGeometry\", \"data\" : " + this.geometryToJSONString(dependencyDescriptor,
                arrow, "#adccff", 1000, true, null) + "}";
        enqueueMessage(jsonString);
    }

    /**
     * Converts a geometry into a JSON string. The JSON string includes information about the geometry's name, color, age,
     * whether it is filled or not, and any extra data.
     *
     * @param name The name of the geometry.
     * @param geom The geometry object.
     * @param color The color of the geometry.
     * @param age The age of the geometry.
     * @param filled Whether the geometry is filled or not.
     * @param extraData Any additional data to include in the JSON string.
     * @return Returns the geometry represented as a JSON string.
     */
    private String geometryToJSONString(String name, Geometry geom, String color, long age, boolean filled, String extraData) {
        StringBuilder ret = new StringBuilder("{ \"name\" : \"" + name + "\", \"color\" : \"" + color + "\", ");
        if (age > 0) ret.append(" \"age\" : ").append(age).append(", ");
        ret.append(" \"filled\" : ").append(filled).append(", ");
        if (extraData != null && !extraData.trim().equals("")) ret.append(" \"extraData\" : \"").append(extraData).append("\", ");
        ret.append("\"coordinates\" : [");
        Coordinate[] coordinate = geom.getCoordinates();
        for (int i = 0; i < coordinate.length; i++) {
            ret.append("{\"x\" : ").append(coordinate[i].x).append(", \"y\" : ").append(coordinate[i].y).append("}");
            if (i < coordinate.length-1) ret.append(", ");
        }
        ret.append("]}");
        return ret.toString();
    }

    /**
     * Adds a trajectory envelope to the visualization. The envelope will be colored
     * with the color of the vehicle.
     *
     * @param te The trajectory envelope to be added.
     * TODO Implement safety distances
     */
    @Override
    public void addEnvelope(TrajectoryEnvelope te) {

        // Color the trajectory envelope with the same vehicle color
        String color = "#efe007";
        if (!RobotHashMap.getList().isEmpty()) {
            color = RobotHashMap.getRobot(te.getRobotID()).getColorCode();
        }

        GeometricShapeDomain dom = (GeometricShapeDomain)te.getEnvelopeVariable().getDomain();
        Geometry geom = dom.getGeometry();
        String jsonString = "{ \"operation\" : \"addGeometry\", \"data\" : " + this.geometryToJSONString("_" +
                te.getID(), geom, color, -1, false, null) + "}";
        enqueueMessage(jsonString);
    }

    /**
     * Removes a trajectory envelope from the visualization.
     *
     * @param te The trajectory envelope to be removed.
     */
    @Override
    public void removeEnvelope(TrajectoryEnvelope te) {
        String jsonString = "{ \"operation\" : \"removeGeometry\","
                + "\"data\" : "
                + "{ \"name\" : \""+ "_"+te.getID() +"\" }}";
        enqueueMessage(jsonString);
    }

    /**
     * Updates the visualization.
     * In this implementation, this method does nothing because viz change events are buffered and
     * sent by an internal thread in bursts every UPDATE_PERIOD ms to avoid blocking of RemoteEndpoints.
     */
    @Override
    public void updateVisualization() {
    }

    /**
     * Updates the font scale used in the visualization.
     *
     * @param scale The new scale factor for the font.
     */
    public void updateFontScale(double scale) {
        String jsonString = "{ \"operation\" : \"updateFontScale\","
                + "\"data\" : "
                + "{ \"value\" : \""+ scale +"\" }}";

        enqueueMessage(jsonString);
    }

    /**
     * Sends a refresh operation message to update the current visualization.
     */
    public void sendUpdate() {
        String callRefresh = "{ \"operation\" : \"refresh\" }";
        sendMessage(callRefresh);
    }

    /**
     * Creates an arrow geometry based on a given pose, length, and size. This can
     * be used to visually represent the orientation and position of a robot in 2D space.
     *
     * @param pose The pose from which to create the arrow.
     * @param length The length of the arrow.
     * @param size The size of the arrow.
     * @return Returns a Geometry object representing the arrow.
     */
    private Geometry createArrow(Pose pose, double length, double size) {
        var geometryFactory = new GeometryFactory();
        double aux = 1.8;
        double aux1 = 0.8;
        double aux2 = 0.3;
        double theta = pose.getTheta();
        var coordinate = new Coordinate[8];
        coordinate[0] = new Coordinate(0.0,-aux2);
        coordinate[1] = new Coordinate(length-aux,-aux2);
        coordinate[2] = new Coordinate(length-aux,-aux1);
        coordinate[3] = new Coordinate(length,0.0);
        coordinate[4] = new Coordinate(length-aux,aux1);
        coordinate[5] = new Coordinate(length-aux,aux2);
        coordinate[6] = new Coordinate(0.0,aux2);
        coordinate[7] = new Coordinate(0.0,-aux2);
        var arrow = geometryFactory.createPolygon(coordinate);
        var at = new AffineTransformation();
        at.scale(size, size);
        at.rotate(theta);
        at.translate(pose.getX(), pose.getY());
        return at.transform(arrow);
    }

    /**
     * This method creates an arrow shaped polygon representing the path from one pose to another.
     *
     * @param pose1 The starting pose of the arrow.
     * @param pose2 The ending pose of the arrow.
     * @return Returns a Geometry object which represents the arrow shaped path.
     */
    private Geometry createArrow(Pose pose1, Pose pose2) {
        var geometryFactory = new GeometryFactory();
        double aux = 1.8;
        double aux1 = 0.8;
        double aux2 = 0.3;
        double factor = Math.sqrt(robotFootprintArea)*0.5;
        double distance = Math.sqrt(Math.pow((pose2.getX()-pose1.getX()),2)+Math.pow((pose2.getY()-pose1.getY()),2))/factor;
        double theta = Math.atan2(pose2.getY() - pose1.getY(), pose2.getX() - pose1.getX());
        Coordinate[] coordinate = new Coordinate[8];
        coordinate[0] = new Coordinate(0.0,-aux2);
        coordinate[1] = new Coordinate(distance-aux,-aux2);
        coordinate[2] = new Coordinate(distance-aux,-aux1);
        coordinate[3] = new Coordinate(distance,0.0);
        coordinate[4] = new Coordinate(distance-aux,aux1);
        coordinate[5] = new Coordinate(distance-aux,aux2);
        coordinate[6] = new Coordinate(0.0,aux2);
        coordinate[7] = new Coordinate(0.0,-aux2);
        Polygon arrow = geometryFactory.createPolygon(coordinate);
        AffineTransformation at = new AffineTransformation();
        at.scale(factor, factor);
        at.rotate(theta);
        at.translate(pose1.getX(), pose1.getY());
        return at.transform(arrow);
    }

    /**
     * Sets the map using a BufferedImage object, resolution, and origin.
     *
     * @param mapImage The map represented as a BufferedImage object.
     * @param resolution The resolution of the map.
     * @param origin The origin coordinate of the map.
     */
    public void setMap(BufferedImage mapImage, double resolution, Coordinate origin) {
        BrowserVisualizationSocket.map = mapImage;
        BrowserVisualizationSocket.resolution = resolution;
        BrowserVisualizationSocket.origin = origin;
    }

    /**
     * Sets the map using a YAML file. The YAML file contains the properties of the map.
     *
     * @param mapYAMLFile The path to the YAML file containing map properties.
     */
    public void setMap(String mapYAMLFile) {
        try {
            File file = new File(mapYAMLFile);
            BufferedReader br = new BufferedReader(new FileReader(file));
            String imageFileName = null;
            String st;
            while((st=br.readLine()) != null){
                if (!st.trim().startsWith("#") && !st.trim().isEmpty()) {
                    String key = st.substring(0, st.indexOf(":")).trim();
                    String value = st.substring(st.indexOf(":")+1).trim();
                    switch (key) {
                        case "image":
                            imageFileName = file.getParentFile() + File.separator + value;
                            break;
                        case "resolution":
                            BrowserVisualizationSocket.resolution = Double.parseDouble(value);
                            break;
                        case "origin":
                            String x = value.substring(1, value.indexOf(",")).trim();
                            String y = value.substring(value.indexOf(",") + 1, value.indexOf(",",
                                    value.indexOf(",") + 1)).trim();
                            BrowserVisualizationSocket.origin = new Coordinate(Double.parseDouble(x), Double.parseDouble(y));
                            break;
                    }
                }
            }
            br.close();
            assert imageFileName != null;
            BrowserVisualizationSocket.map = ImageIO.read(new File(imageFileName));
        }
        catch (IOException e) { e.printStackTrace(); }
    }

    /**
     * This method provides the refresh rate for the envelope in milliseconds.
     *
     * @return Returns the refresh rate in milliseconds.
     */
    @Override
    public int periodicEnvelopeRefreshInMillis() {
        return 1000;
    }

}