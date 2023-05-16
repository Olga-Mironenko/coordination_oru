package se.oru.coordination.coordination_oru.util;


import java.awt.Desktop;
import java.awt.Dimension;
import java.awt.image.BufferedImage;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.net.URI;
import java.net.URISyntaxException;
import java.util.*;

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

import se.oru.coordination.coordination_oru.CriticalSection;
import se.oru.coordination.coordination_oru.Mission;
import se.oru.coordination.coordination_oru.RobotReport;
import se.oru.coordination.coordination_oru.code.AbstractVehicle;
import se.oru.coordination.coordination_oru.code.VehiclesHashMap;
import se.oru.coordination.coordination_oru.simulation2D.TrajectoryEnvelopeCoordinatorSimulation;
import se.oru.coordination.coordination_oru.simulation2D.TrajectoryEnvelopeTrackerRK4;
import se.oru.coordination.coordination_oru.util.gates.GatedThread;

public class BrowserVisualization implements FleetVisualization {
	
	private ArrayList<String> msgQueue = new ArrayList<String>();
	private static int UPDATE_PERIOD = 30;
	private String overlayText = null;

	public static boolean isStatusText = false;

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
        			try { GatedThread.sleep(UPDATE_PERIOD); }
        			catch (InterruptedException e) { e.printStackTrace(); }
        		}
        	}
        };
        updateThread.start();
        BrowserVisualization.setupVizServer(serverHostNameOrIP);
        startOpenInBrowser(serverHostNameOrIP);
	}
	
	private void startOpenInBrowser(String serverHostNameOrIP) {
		if (Desktop.isDesktopSupported() && Desktop.getDesktop().isSupported(Desktop.Action.BROWSE)) {
			try { Desktop.getDesktop().browse(new URI("http://" + serverHostNameOrIP + ":8080")); }
			catch (IOException e) { e.printStackTrace(); }
			catch (URISyntaxException e) { e.printStackTrace(); }
		}
	}
	
	private void updateOverlayText() {
		if (this.overlayText != null) {
			String jsonString = "{ \"operation\" : \"setOverlayText\","
					+ "\"data\" : "
					+ "{ \"text\" : \""+ this.overlayText + "\" }}";
			sendMessage(jsonString);
		}
	}
	
	public void setOverlayText(String text) {
		this.overlayText = text;
	}
	
	public void setInitialTransform(double scale, double xTrans, double yTrans) {
		BrowserVisualizationSocket.initialScale = scale;
		BrowserVisualizationSocket.initialTranslation = new Coordinate(xTrans,yTrans);		
	}
	
	public void setFontScale(double scale) {
		BrowserVisualizationSocket.fontScale = scale;
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
	
	public void guessInitialTransform(double robotDimension, Pose ... robotPoses) {
		BrowserVisualizationSocket.initialScale = getScreenDPI()/robotDimension;
		double avgX = 0;
		double avgY = 0;
		for (int i = 0; i < robotPoses.length; i++) {
			avgX += robotPoses[i].getX();
			avgY += robotPoses[i].getY();
		}
		avgX /= robotPoses.length;
		avgY /= robotPoses.length;
		avgY -= 0.45*(getScreenHeight()/getScreenDPI());
		BrowserVisualizationSocket.initialTranslation = new Coordinate(avgX,avgY);		
	}

	private static void setupVizServer(String serverHostNameOrIP) {
		Server server = new Server(8080);
		server.setHandler(new BrowserVisualizationServer(serverHostNameOrIP));
		try {
			server.start();
			//server.join();
		}
        catch (Throwable t) { t.printStackTrace(System.err); }
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
        }
        catch (Throwable t) { t.printStackTrace(System.err); }		
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
				}
				catch(IOException e) { e.printStackTrace(); }
			}
		}
	}

	@Override
	public void displayRobotState(TrajectoryEnvelope te, RobotReport rr, String... extraStatusInfo) {
		double x = rr.getPathIndex() != -1 ? rr.getPose().getX() : te.getTrajectory().getPose()[0].getX();
		double y = rr.getPathIndex() != -1 ? rr.getPose().getY() : te.getTrajectory().getPose()[0].getY();
		double theta = rr.getPathIndex() != -1 ? rr.getPose().getTheta() : te.getTrajectory().getPose()[0].getTheta();

		String name = "R" + te.getRobotID();

		// Show percentage of path completed
		String extraData = " : " + "0 %";
		if (rr.getPathIndex() >= 0) {
			extraData = " : " + Math.round((double) rr.getPathIndex() / (double) te.getPathLength() * 100) + " %";
		}

		// Show path Index
//		String extraData = " : " + rr.getPathIndex();
		if (extraStatusInfo != null) {
			for (String st : extraStatusInfo) {
				extraData += (" | " + st);
			}
		}

		String color = "#000000";
		int vehicleCount = VehiclesHashMap.getInstance().getList().keySet().size();
		if (vehicleCount != 0) color = VehiclesHashMap.getVehicle(rr.getRobotID()).getColorCode();

		drawRobotFootprint(x, y, theta, rr.getPose(), "#bbbbbb", name, extraData, false, te.getFootprint());

		TrajectoryEnvelopeCoordinatorSimulation tec = TrajectoryEnvelopeCoordinatorSimulation.tec;
		Coordinate[] innerFootprint = tec.getInnerFootprint(rr.getRobotID());
		Polygon innerFootprintPolygon = TrajectoryEnvelope.createFootprintPolygon(innerFootprint);
		drawRobotFootprint(x, y, theta, null, color, "_" + name + "-inner", "", true, innerFootprintPolygon);

		if (isStatusText) {
			setStatusText();
		}
	}

	protected void drawRobotFootprint(double x, double y, double theta, Pose poseArrow, String color, String name, String extraData, boolean filled, Polygon footprint) {
		Geometry geom = TrajectoryEnvelope.getFootprint(footprint, x, y, theta);
		double robotFootprintArea = geom.getArea();
		double minX = Double.MAX_VALUE;
		double maxX = Double.MIN_VALUE;
		for (Coordinate coord : geom.getCoordinates()) {
			if (coord.x < minX) minX = coord.x;
			if (coord.x > maxX) maxX = coord.x;
		}
		double robotFootprintXDim = maxX - minX;

		double scale = Math.sqrt(robotFootprintArea) * 0.2;
		String jsonString = "{ \"operation\" : \"addGeometry\", \"data\" : " + this.geometryToJSONString(name, geom, color, -1, filled, extraData) + "}";
		enqueueMessage(jsonString);

		if (poseArrow != null) {
			Geometry arrowGeom = createArrow(poseArrow, robotFootprintXDim / scale, scale);
			String jsonStringArrow = "{ \"operation\" : \"addGeometry\", \"data\" : " + this.geometryToJSONString("_" + name, arrowGeom, "#ffffff", -1, true, null) + "}";
			enqueueMessage(jsonStringArrow);
		}
	}

	protected static double round(double value) {
		return (double) Math.round(value * 10) / 10;
	}
	
	protected void setStatusText() {
		HashMap<Integer, AbstractVehicle> idToVehicle = VehiclesHashMap.getInstance().getList();
		String text = "";
		if (idToVehicle.keySet().contains(MissionUtils.idHuman) && MissionUtils.targetVelocityHuman != Double.POSITIVE_INFINITY) {
			text += "targetVelocityHuman: " + round(MissionUtils.targetVelocityHuman) + "<br>";
		}
		text += TrajectoryEnvelopeTrackerRK4.emergencyBreaker.toString() + "<br>";
		for (int id : idToVehicle.keySet()) {
			text += "(Robot " + id + ") ";

			RobotReport rr = TrajectoryEnvelopeCoordinatorSimulation.tec.getRobotReport(id);
			if (rr == null) {
				text += "no robot report";
			} else {
				text += "i=" + rr.getPathIndex() + "; p=" + rr.getPose() + "; ";

				double velocity = rr.getVelocity();
				text += "v=" + round(velocity) + " m/s";

				int numCalls = 0;
				var numIntegrateCalls = TrajectoryEnvelopeCoordinatorSimulation.tec.numIntegrateCalls;
				if (numIntegrateCalls.containsKey(id)) {
					numCalls = numIntegrateCalls.get(id);
				}
				text += "; numIntegrateCalls: " + numCalls;
			}

			text += "; " + stringifyMissions(Missions.getMissions(id));

			text += "<br>";
		}
		text += stringifyCriticalSections(TrajectoryEnvelopeCoordinatorSimulation.tec.allCriticalSections);
		setOverlayText(text);
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
		Integer robotID = MissionUtils.idHuman;

		ArrayList<CriticalSection> criticalSections =
				robotID == null
				? CriticalSection.sortCriticalSections(allCriticalSections)
				: CriticalSection.sortCriticalSectionsForRobotID(allCriticalSections, MissionUtils.idHuman);

		String text = "Critical sections" + (robotID == null ? "" : " for " + robotID) + ":";
		if (criticalSections.isEmpty()) {
			text += " none<br>";
		} else {
			text += "<br>";
			for (CriticalSection cs : criticalSections) {
				text += "- " + cs.toStringForVisualization() + "<br>";
			}
		}
		return text;
	}
	
	public void addPath(String pathName, PoseSteering[] ps, double arrowLength, String color) {
		for (int i = 0; i < ps.length; i++) {
			Geometry arrowGeom = createArrow(ps[i].getPose(), arrowLength, 0.2*arrowLength);		
			String jsonStringArrow = "{ \"operation\" : \"addGeometry\", \"data\" : " + this.geometryToJSONString("_"+pathName+"_"+i, arrowGeom, color, -1, true, null) + "}";
			enqueueMessage(jsonStringArrow);
		}
	}
	
	public void removePath(String pathName, PoseSteering[] ps) {
		for (int i = 0; i < ps.length; i++) {
			String jsonString = "{ \"operation\" : \"removeGeometry\"," + "\"data\" : " + "{ \"name\" : \"" + "_"+pathName+"_"+i +"\" }}";
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
			if (i < coords.length-1) ret += ", ";
		}
		ret += "]}";
		return ret;
	}

	@Override
	public void addEnvelope(TrajectoryEnvelope te) {

		// Color the trajectory envelope with the same vehicle color
		String color = "#efe007";
		if (!VehiclesHashMap.getList().isEmpty()) {
			color = VehiclesHashMap.getVehicle(te.getRobotID()).getColorCode();
		}

		GeometricShapeDomain dom = (GeometricShapeDomain)te.getEnvelopeVariable().getDomain();
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
		// in bursts every UPDATE_PERIOD ms to avoid blocking of RemoteEndpopints
	}
	
	public void updateFontScale(double scale) {
		String jsonString = "{ \"operation\" : \"updateFontScale\","
				+ "\"data\" : "
				+ "{ \"value\" : \""+ scale +"\" }}";
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
		coords[0] = new Coordinate(0.0,-aux2);
		coords[1] = new Coordinate(length-aux,-aux2);
		coords[2] = new Coordinate(length-aux,-aux1);
		coords[3] = new Coordinate(length,0.0);
		coords[4] = new Coordinate(length-aux,aux1);
		coords[5] = new Coordinate(length-aux,aux2);
		coords[6] = new Coordinate(0.0,aux2);
		coords[7] = new Coordinate(0.0,-aux2);
		Polygon arrow = gf.createPolygon(coords);
		AffineTransformation at = new AffineTransformation();
		at.scale(size, size);
		at.rotate(theta);
		at.translate(pose.getX(), pose.getY());
		Geometry ret = at.transform(arrow);
		return ret;
	}

	public static double computeDistanceBetweenPoses(Pose pose1, Pose pose2) {
		return Math.sqrt(Math.pow((pose2.getX()-pose1.getX()),2)+Math.pow((pose2.getY()-pose1.getY()),2));
	}
	
	private Geometry createArrow(Pose pose1, Pose pose2) {		
		GeometryFactory gf = new GeometryFactory();
		double aux = 1.8;
		double aux1 = 0.8;
		double aux2 = 0.3;
		double factor = 1.5;
		double distance = computeDistanceBetweenPoses(pose1, pose2)/factor;
		double theta = Math.atan2(pose2.getY() - pose1.getY(), pose2.getX() - pose1.getX());
		Coordinate[] coords = new Coordinate[8];
		coords[0] = new Coordinate(0.0,-aux2);
		coords[1] = new Coordinate(distance-aux,-aux2);
		coords[2] = new Coordinate(distance-aux,-aux1);
		coords[3] = new Coordinate(distance,0.0);
		coords[4] = new Coordinate(distance-aux,aux1);
		coords[5] = new Coordinate(distance-aux,aux2);
		coords[6] = new Coordinate(0.0,aux2);
		coords[7] = new Coordinate(0.0,-aux2);
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
			while((st=br.readLine()) != null){ 
				if (!st.trim().startsWith("#") && !st.trim().isEmpty()) {
					String key = st.substring(0, st.indexOf(":")).trim();
					String value = st.substring(st.indexOf(":")+1).trim();
					if (key.equals("image")) imageFileName = file.getParentFile()+File.separator+value;
					else if (key.equals("resolution")) BrowserVisualizationSocket.resolution = Double.parseDouble(value);
					else if (key.equals("origin")) {
						String x = value.substring(1, value.indexOf(",")).trim();
						String y = value.substring(value.indexOf(",")+1, value.indexOf(",", value.indexOf(",")+1)).trim();
						BrowserVisualizationSocket.origin = new Coordinate(Double.parseDouble(x),Double.parseDouble(y));
						//bottomLeftOrigin = new Coordinate(Double.parseDouble(x),Double.parseDouble(y));
					}
				}
			}
			br.close();
			BrowserVisualizationSocket.map = ImageIO.read(new File(imageFileName));
			//BrowserVisualizationSocket.origin = new Coordinate(bottomLeftOrigin.x, BrowserVisualizationSocket.map.getHeight()*BrowserVisualizationSocket.resolution-bottomLeftOrigin.y);
		}
		catch (IOException e) { e.printStackTrace(); }
	}
	
	public void setMapYAML(String mapYAMLSpec, String pathPrefix) {
		try {
			String imageFileName = "";
			if (pathPrefix != null) imageFileName = pathPrefix+File.separator;
			for (String st : mapYAMLSpec.split("\n")) { 
				if (!st.trim().startsWith("#") && !st.trim().isEmpty()) {
					String key = st.substring(0, st.indexOf(":")).trim();
					String value = st.substring(st.indexOf(":")+1).trim();
					if (key.equals("image")) imageFileName += value;
					else if (key.equals("resolution")) BrowserVisualizationSocket.resolution = Double.parseDouble(value);
					else if (key.equals("origin")) {
						String x = value.substring(1, value.indexOf(",")).trim();
						String y = value.substring(value.indexOf(",")+1, value.indexOf(",", value.indexOf(",")+1)).trim();
						BrowserVisualizationSocket.origin = new Coordinate(Double.parseDouble(x),Double.parseDouble(y));
						//bottomLeftOrigin = new Coordinate(Double.parseDouble(x),Double.parseDouble(y));
					}
				}
			}
			System.out.println(imageFileName);
			BrowserVisualizationSocket.map = ImageIO.read(new File(imageFileName));
			//BrowserVisualizationSocket.origin = new Coordinate(bottomLeftOrigin.x, BrowserVisualizationSocket.map.getHeight()*BrowserVisualizationSocket.resolution-bottomLeftOrigin.y);
		}
		catch (IOException e) { e.printStackTrace(); }
	}
	
	@Override
	public int periodicEnvelopeRefreshInMillis() {
		return 1000;
	}

}
