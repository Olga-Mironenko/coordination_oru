package se.oru.coordination.coordination_oru.util;

import java.awt.Desktop;
import java.awt.Dimension;
import java.io.File;
import java.io.IOException;
import java.net.URI;
import java.net.URISyntaxException;
import java.time.format.DateTimeFormatter;
import java.util.*;

import com.google.gson.Gson;
import com.google.gson.JsonObject;

import org.apache.commons.io.FilenameUtils;
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


import se.oru.coordination.coordination_oru.*;
import se.oru.coordination.coordination_oru.code.AbstractVehicle;
import se.oru.coordination.coordination_oru.code.VehiclesHashMap;
import se.oru.coordination.coordination_oru.simulation2D.AdaptiveTrajectoryEnvelopeTrackerRK4;
import se.oru.coordination.coordination_oru.simulation2D.TrajectoryEnvelopeCoordinatorSimulation;
import se.oru.coordination.coordination_oru.util.gates.GatedThread;
import se.oru.coordination.coordination_oru.util.gates.Timekeeper;

public class BrowserVisualization implements FleetVisualization {
	private final ArrayList<String> msgQueue = new ArrayList<String>();
	private static int UPDATE_PERIOD = 30;
	private String overlayText = null;

	public static boolean isStatusText = false;
	public static boolean isExtendedText = false;

	public static boolean areAllVehiclesStarted = false;

	public static Map<String, String> mapPretable = null;
	public static String[] statsColumns = null;
	public static TreeMap<Integer, String[]> statsIdToRow = null;

	public static int runProcess(String... args) {
		Process process;
		try {
			ProcessBuilder builder = new ProcessBuilder(args);
			builder.redirectError(ProcessBuilder.Redirect.INHERIT);
			builder.redirectOutput(ProcessBuilder.Redirect.INHERIT);
			process = builder.start();
		} catch (IOException e) {
			throw new RuntimeException(e);
		}
		int code;
		try {
			code = process.waitFor();
		} catch (InterruptedException e) {
			throw new RuntimeException(e);
		}
		return code;
	}

	public static void makeScreenshot() {
		File fileScenario = new File(AbstractVehicle.scenarioFilename);
		File dirScreenshots = new File(fileScenario.getParent() + "/screenshots");
		if (! dirScreenshots.exists()) {
			boolean isCreated = dirScreenshots.mkdir();
			assert isCreated;
		}

		String basename = FilenameUtils.getBaseName(fileScenario.getName());
		File fileScreenshotFull = new File(String.format("%s/fullscreen_%s.png", dirScreenshots, basename));
		File fileScreenshotCropped = new File(String.format("%s/%s.png", dirScreenshots, basename));
		if (fileScreenshotCropped.exists()) {
			return;
		}

		int codeMake = runProcess("screenshotting/make-screenshot.sh", fileScreenshotFull.toString());
		assert codeMake == 0;
		int codeCrop = runProcess("screenshotting/crop-screenshot.sh",
				fileScreenshotFull.toString(), fileScreenshotCropped.toString());
		assert codeCrop == 0;
	}

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
        			catch (InterruptedException e) { e.printStackTrace(); return; }
        		}
        	}
        };
        updateThread.start();
        BrowserVisualization.setupVizServer(serverHostNameOrIP);
        startOpenInBrowser(serverHostNameOrIP);

		new GatedThread("screenshot thread") { // path planning takes a while
			@Override
			public void runCore() {
				while (true) {
					boolean isReady = true;
					for (AbstractVehicle vehicle : VehiclesHashMap.getVehicles()) {
						AdaptiveTrajectoryEnvelopeTrackerRK4 tracker = vehicle.getAdaptiveTracker();
						if (tracker == null || ! tracker.isRunCalled) {
							isReady = false;
							break;
						}
					}
					if (isReady) {
						areAllVehiclesStarted = true;
						makeScreenshot();
						break;
					}
					GatedThread.skipTimesteps(1);
				}
			}
		}.start();
	}
	
	private void startOpenInBrowser(String serverHostNameOrIP) {
		String url = "http://" + serverHostNameOrIP + ":8080";
		if (Containerization.IS_CONTAINER) {
			int code = runProcess("container/chromium.sh", url);
			assert code == 0;
		} else if (Desktop.isDesktopSupported() && Desktop.getDesktop().isSupported(Desktop.Action.BROWSE)) {
			try { Desktop.getDesktop().browse(new URI(url)); }
			catch (IOException | URISyntaxException e) { e.printStackTrace(); }
        }
	}
	
	private void updateOverlayText() {
		if (this.overlayText != null) {
			JsonObject jsonObject = new JsonObject();
			jsonObject.addProperty("text", this.overlayText);
			String payload = new Gson().toJson(jsonObject);

			String jsonString = "{ \"operation\": \"setOverlayText\", \"data\": " + payload + "}";
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
		server.setStopAtShutdown(true);
		server.setHandler(new BrowserVisualizationServer(serverHostNameOrIP));
		try {
			server.start();
			//server.join();
		}
        catch (Throwable t) { t.printStackTrace(System.err); }
	}
	
	private static void setupVizMessageServer() {
        Server server = new Server();
		server.setStopAtShutdown(true);
        ServerConnector connector = new ServerConnector(server);
        connector.setPort(8081);
		connector.setIdleTimeout(300 * 60 * 1000); // 300 min. (for long pauses inside the debugger)
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
		synchronized (BrowserVisualizationSocket.ENDPOINTS) {
			if (!BrowserVisualizationSocket.ENDPOINTS.isEmpty()) {
				this.msgQueue.add(message);
			}
		}
	}
	
	private void sendMessages() {
		synchronized (BrowserVisualizationSocket.ENDPOINTS) {
			if (!BrowserVisualizationSocket.ENDPOINTS.isEmpty()) {
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
		for (RemoteEndpoint rep : BrowserVisualizationSocket.ENDPOINTS) {
			try {
				rep.sendString(text);
			}
			catch(IOException e) { e.printStackTrace(); }
		}
	}

	@Override
	public void displayRobotState(TrajectoryEnvelope te, RobotReport rr, String... extraStatusInfo) {
		double x = rr.getPathIndex() != -1 ? rr.getPose().getX() : te.getTrajectory().getPoseSteering()[0].getPose().getX();
		double y = rr.getPathIndex() != -1 ? rr.getPose().getY() : te.getTrajectory().getPoseSteering()[0].getPose().getY();
		double theta = rr.getPathIndex() != -1 ? rr.getPose().getTheta() : te.getTrajectory().getPoseSteering()[0].getPose().getTheta();

		String name = "V" + te.getRobotID();

		// Show percentage of path completed
		int percent = rr.getPathIndex() < 0 ? 0 : (int) Math.round((double) rr.getPathIndex() / (double) te.getPathLength() * 100);
//		String extraData = ":" + (extraStatusInfo == null || extraStatusInfo.length == 0 ? "" : " ") + percent + "%";
		StringBuilder extraData = new StringBuilder();

		// Show path Index
//		String extraData = " : " + rr.getPathIndex();
		if (extraStatusInfo != null) {
			for (String st : extraStatusInfo) {
				extraData.append(" | ").append(st);
			}
		}

		String colorOuter = "#bbbbbb";

		String colorInner = "#000000";
		int vehicleCount = VehiclesHashMap.getList().keySet().size();
		if (vehicleCount != 0) colorInner = VehiclesHashMap.getVehicle(rr.getRobotID()).getColorCode();

		drawRobotFootprint(x, y, theta, rr.getPose(), colorOuter, name, extraData.toString(), false, te.getFootprint());

		TrajectoryEnvelopeCoordinatorSimulation tec = TrajectoryEnvelopeCoordinatorSimulation.tec;
		Coordinate[] innerFootprint = tec.getInnerFootprint(rr.getRobotID());
		if (innerFootprint != null) {
			Polygon innerFootprintPolygon = TrajectoryEnvelope.createFootprintPolygon(innerFootprint);
			drawRobotFootprint(x, y, theta, null, colorInner, "_" + name + "-inner", "", true, innerFootprintPolygon);
		}

		if (isStatusText) {
			setStatusText();
		}
	}

	protected void drawRobotFootprint(double x, double y, double theta, Pose poseArrow, String color, String name, String extraData, boolean filled, Polygon footprint) {
		Geometry geom = TrajectoryEnvelope.getFootprint(footprint, x, y, theta);
		double robotFootprintArea = geom.getArea();
		double minX = Double.POSITIVE_INFINITY;
		double maxX = Double.NEGATIVE_INFINITY;
		double minY = Double.POSITIVE_INFINITY;
		double maxY = Double.NEGATIVE_INFINITY;
		for (Coordinate coord : geom.getCoordinates()) {
			minX = Math.min(minX, coord.x);
			maxX = Math.max(maxX, coord.x);
			minY = Math.min(minY, coord.y);
			maxY = Math.max(maxY, coord.y);
		}
		double robotLength = Math.max(maxX - minX, maxY - minY); // as approximation

		double scale = Math.sqrt(robotFootprintArea) * 0.2;
		String jsonString = "{ \"operation\" : \"addGeometry\", \"data\" : " + this.geometryToJSONString(name, geom, color, -1, filled, extraData) + "}";
		enqueueMessage(jsonString);

		if (poseArrow != null) { // theta
			Geometry arrowGeom = createArrow(poseArrow, robotLength / (1.5 * scale), scale);
			String jsonStringArrow = "{ \"operation\" : \"addGeometry\", \"data\" : " + this.geometryToJSONString("_" + name, arrowGeom, "#ffffff", -1, true, null) + "}";
			enqueueMessage(jsonStringArrow);
		}
	}

	protected static double round(double value) {
		return (double) Math.round(value * 10) / 10;
	}

	public static String secondsToHMS(long seconds) {
		long time = seconds;

		long s = time % 60;
		time /= 60;

		long m = time % 60;
		time /= 60;

		long h = time;

		return String.format("%d:%02d:%02d", h, m, s);
	}

	protected String center(Object object) {
		return "<div style=\"text-align: center;\">" + object.toString() + "</div>";
	}

	protected Set<Integer> findDeadlockedRobotsImperfect() {
		TrajectoryEnvelopeCoordinatorSimulation tec = TrajectoryEnvelopeCoordinatorSimulation.tec;
		TreeSet<Integer> robotIDsDeadlocked = new TreeSet<>();
		for (CriticalSection cs : tec.allCriticalSections) {
			int[] robotIDs = cs.getRobotIDs();
			if (robotIDs.length == 0) {
				continue;
			}
			assert robotIDs.length == 2;

			boolean isDeadlocked = true;
			for (int robotID : robotIDs) {
				AdaptiveTrajectoryEnvelopeTrackerRK4 tracker = VehiclesHashMap.getVehicle(robotID).getAdaptiveTracker();
				RobotReport rr = tec.getRobotReport(robotID);
				boolean isStopped = (
					tracker != null &&
					rr.getVelocity() == 0.0 &&
					// TODO: check that we are in the CP of the CS
					(tracker.statusLast == AdaptiveTrajectoryEnvelopeTrackerRK4.Status.STOPPED_AT_CP
						|| cs.getStart(robotID) <= rr.getPathIndex() && rr.getPathIndex() <= cs.getEnd(robotID)
					)
				);
				if (! isStopped) {
					isDeadlocked = false;
					break;
				}
			}
			if (isDeadlocked) {
				robotIDsDeadlocked.add(robotIDs[0]);
				robotIDsDeadlocked.add(robotIDs[1]);
			}
		}
		return robotIDsDeadlocked;
	}

	protected static String htmlToCsv(String html) {
		String text = html
				.replace("<br>", " ")
				.replaceAll("<[^>]+>", "");
		if (text.matches("[^\"\\t\\n]*")) {
			return text;
		}
		return "\"" + text.replace("\"", "\"\"") + "\"";
	}

	public static StringBuilder[] makeHeaderHtmls(String input) {
		String[] parts = input.split(" [|]");
		StringBuilder[] htmls = new StringBuilder[parts.length];

		int colIndex = 0;
		for (int i = 0; i < parts.length; i++) {
			String part = parts[i];

			int lengthNumber = 0;
			while (lengthNumber < part.length() && Character.isDigit(part.charAt(lengthNumber))) {
				lengthNumber++;
			}

			int numCols = 1;
			String style = null;
			if (lengthNumber > 0) {
				numCols = Integer.parseInt(part.substring(0, lengthNumber));
				if (part.charAt(lengthNumber) == ' ') {
					part = part.substring(lengthNumber + 1);
				} else if (part.charAt(lengthNumber) == '{') {
					int end = part.indexOf('}', lengthNumber + 1);
					assert end > 0;
					style = part.substring(lengthNumber, end + 1);
					assert part.charAt(end + 1) == ' ';
					part = part.substring(end + 2);
				} else {
					throw new RuntimeException();
				}
			}

			StringBuilder builderHtml = new StringBuilder("<th class=\"");
			StringBuilder builderCssHeader = style == null ? null : new StringBuilder();
			for (int j = 0; j < numCols; j++) {
				if (j > 0) {
					builderHtml.append(" ");
					if (builderCssHeader != null) {
						builderCssHeader.append(", ");
					}
				}
				builderHtml.append("col").append(colIndex);
				if (builderCssHeader != null) {
					builderCssHeader.append("table.info .col").append(colIndex);
				}
				colIndex++;
			}
			builderHtml.append("\"");
			if (numCols > 1) {
				builderHtml.append(" colspan=").append(numCols);
			}
			builderHtml.append(">");
			builderHtml.append(part);
			builderHtml.append("</th>");

			if (builderCssHeader != null) {
				builderHtml.append("<style>").append(builderCssHeader).append(" ").append(style).append("</style>");
			}

			htmls[i] = builderHtml;
		}

		return htmls;
	}

	protected StringBuilder makeVehicleTableHtml() {
        StringBuilder thead1 = null;
        StringBuilder theadHints = null;
		StringBuilder thead2 = null;
		StringBuilder tbodyHtml = new StringBuilder();

		HashMap<Integer, AbstractVehicle> idToVehicle = VehiclesHashMap.getList();
		TrajectoryEnvelopeCoordinatorSimulation tec = TrajectoryEnvelopeCoordinatorSimulation.tec;

		HashSet<Integer> robotIDsPassFirstCan = new HashSet<>();
		HashSet<Integer> robotIDsPassFirstAffected = new HashSet<>();
		HashSet<Integer> robotIDsPassFirstCannot = new HashSet<>();
		HashSet<Integer> robotIDsPassFirstUnaffected = new HashSet<>();
		for (CriticalSection cs : tec.allCriticalSections) {
			for (Map.Entry<Integer, HashSet<CriticalSection>> entry : tec.robotIDToCriticalSectionsPassFirstAffected.entrySet()) {
				if (entry.getValue().contains(cs)) {
					robotIDsPassFirstCan.add(entry.getKey());
					robotIDsPassFirstAffected.add(cs.getOtherRobotID(entry.getKey()));
				}
			}
			for (Map.Entry<Integer, HashSet<CriticalSection>> entry : tec.robotIDToCriticalSectionsPassFirstUnaffected.entrySet()) {
				if (entry.getValue().contains(cs)) {
					robotIDsPassFirstCannot.add(entry.getKey());
					robotIDsPassFirstUnaffected.add(cs.getOtherRobotID(entry.getKey()));
				}
			}
		}

		TreeMap<Integer, String[]> idToRow = new TreeMap<>();
		for (int id : idToVehicle.keySet()) {
			AbstractVehicle vehicle = idToVehicle.get(id);
			boolean isHuman = VehiclesHashMap.isHuman(id);
			AbstractTrajectoryEnvelopeTracker tracker = tec.trackers.get(id);
			AdaptiveTrajectoryEnvelopeTrackerRK4 trackerAdaptive =
					(tracker instanceof AdaptiveTrajectoryEnvelopeTrackerRK4)
							? (AdaptiveTrajectoryEnvelopeTrackerRK4) tracker
							: null;

			TrajectoryEnvelope te = tracker.getTrajectoryEnvelope();
            StringBuilder row = new StringBuilder("<div style=\"text-align: left;\"><b>V" + id + "</b>, " +
                    vehicle.getTypeForVisualization() +
                    "</div>");
			thead1 = new StringBuilder();
			theadHints = new StringBuilder();
			thead2 = new StringBuilder("Vehicle ID and type");

			RobotReport rr = tec.getRobotReport(id);
            if (rr != null) {
				double velocity = rr.getVelocity();
                row.append(String.format(" | %s%.1f | %.1f",
                        String.format("%.1f", velocity).equals("0.0") && velocity != 0.0 ? "~" : "",
                        velocity,
                        vehicle.getMaxVelocity()
                ));
				thead1.append(" |2 Velocity, m/s");
				theadHints.append(" |2 ");
				thead2.append(" | [v_]current | [v_]max");

				thead1.append(" |4{border-color: yellow;} Human (mis)behaviour actions");
				theadHints.append(" |4 ");
				thead2.append(" | can pass<br>first | violation of<br>priorities | moving<br>slowly | improper<br>parking");
				if (! isHuman) {
					KnobsAfterForcing knobsAfterForcing = ForcingMaintainer.getKnobsOfTheHuman();
					boolean isToRestore = knobsAfterForcing != null && knobsAfterForcing.isToRestore(id);
					boolean isToResume = knobsAfterForcing != null && knobsAfterForcing.isToResume(id);
					row.append(String.format(" | %s | %s |  | ",
                            center(
									robotIDsPassFirstAffected.contains(id)
											? "affected"
											: robotIDsPassFirstUnaffected.contains(id)
											? "*"
											: ""
							),
                            center(isToRestore || isToResume ? "affected" : "")
                    ));
				} else {
					boolean isForcingOngoing = (
							trackerAdaptive != null &&
							trackerAdaptive.forcingMaintainer != null &&
							trackerAdaptive.forcingMaintainer.isForcingOngoing()
					);
					row.append(String.format(" | %s | %s",
                            center(
									robotIDsPassFirstCan.contains(id)
											? "yes"
											: robotIDsPassFirstCannot.contains(id)
											? "no"
											: ""
							),
                            center(
                                    !isForcingOngoing
                                            ? ""
                                            : AdaptiveTrajectoryEnvelopeTrackerRK4.probabilityForcingForHuman < 1.0
                                            ? "random"
                                            : "constant"
                            )
                    ));
					row.append(String.format(" | %s",
                            center(
                                    !vehicle.isMaxVelocityLowered()
                                            ? ""
                                            : "yes"
                            )
                    ));
					row.append(" | ");
				}

				thead1.append(" |5{border-color: blue;} Coordination strategies for AVs");
				theadHints.append(" | proactive |4 reactive");
				thead2.append(" | cautious<br>mode | rerouting at<br>parked / slow | moving<br>backwards" + " | change of<br>priorities | stops");
				if (isHuman) {
					row.append(" |  |  |  |  | ");
				} else {
					KnobsAfterForcing knobsAfterForcing = ForcingMaintainer.getKnobsOfTheHuman();
					boolean isToRestore = knobsAfterForcing != null && knobsAfterForcing.isToRestore(id);
					boolean isToResume = knobsAfterForcing != null && knobsAfterForcing.isToResume(id);
					row.append(String.format(" | %s | %s |  | %s | %s",
                            center(
                                    !AdaptiveTrajectoryEnvelopeTrackerRK4.isCautiousModeAllowed ? "" :
                                            vehicle.isMaxVelocityLowered() ? "yes" : "no"
                            ),
                            center(
                                    String.format("%s / %s",
                                            !AdaptiveTrajectoryEnvelopeTrackerRK4.isReroutingNearParkedVehicleForNonHuman
                                                    ? "-"
                                                    : tec.robotIDToNumReroutingsNearParkedVehicle.getOrDefault(id, 0).toString(),
                                            !AdaptiveTrajectoryEnvelopeTrackerRK4.isReroutingNearSlowVehicleForNonHuman
                                                    ? "-"
                                                    : tec.robotIDToNumReroutingsNearSlowVehicle.getOrDefault(id, 0).toString()
                                    )
                            ),
                            center(isToRestore && !isToResume ? "temporary" : ""),
                            center(isToResume ? Forcing.isGlobalTemporaryStop ? "global" : "local" : "")
                    ));
				}

				List<CollisionEvent> allCollisions = tec.robotIDToAllCollisions.getOrDefault(id, new ArrayList<>());
				List<CollisionEvent> minorCollisions = tec.robotIDToMinorCollisions.getOrDefault(id, new ArrayList<>());
				List<CollisionEvent> majorCollisions = tec.robotIDToMajorCollisions.getOrDefault(id, new ArrayList<>());
				assert allCollisions.size() == minorCollisions.size() + majorCollisions.size();

				thead1.append(" |3{border-color: red;} Safety-critical events");
				theadHints.append(" |3 ");
				thead2.append(" | violations | near<br>misses | collisions");
				if (! isHuman) {
					row.append(" | ");
				} else {
					int numForcings = Forcing.robotIDToNumForcingEvents.getOrDefault(id, 0);
					int numUselessForcings = Forcing.robotIDToNumUselessForcingEvents.getOrDefault(id, 0);
					int numViolations = numForcings - numUselessForcings;
					row.append(String.format(" | %d", numViolations));
				}
                row.append(String.format(" | %d</b> | %d", minorCollisions.size(), majorCollisions.size()));

                row.append(String.format(" | %.1f | %d | %s",
                        vehicle.totalDistance,
                        vehicle.getNumMissions(),
                        center(vehicle.isBlocked()
								? secondsToHMS((int) vehicle.getAdaptiveTracker().durationStopped)
								: "")
                ));
				thead1.append(" |3{border-color: green;} Efficiency");
				theadHints.append(" |3 ");
				thead2.append(" | traveled<br>total, m | no.<br>missions | blocked");

				if (isExtendedText) {
                    row.append(String.format(" | (%.1f, %.1f) | %.1f",
                            rr.getPose().getX(), rr.getPose().getY(),
                            rr.getDistanceTraveled()
                    ));
					thead1.append(" |8 Tracker state (current mission)");
					theadHints.append(" |8 ");
					thead2.append(" | position<br>(x, y), m | traveled,<br>m");

                    Double positionToSlowDown = trackerAdaptive == null ? null : trackerAdaptive.positionToSlowDown;
					double distanceToCP =
							trackerAdaptive == null ? Double.POSITIVE_INFINITY : trackerAdaptive.distanceToCP;
					row.append(String.format(" | %s | %d | %s | %s | %s | <div style=\"text-align: left;\">%s</div>",
                            rr.getPathIndex() == -1 ? "" : String.format("%d", rr.getPathIndex()),
                            te.getPathLength(),
                            rr.getCriticalPoint() == -1
                                    ? ""
                                    : rr.getCriticalPoint() == TrajectoryEnvelopeCoordinatorSimulation.CP_ASAP
                                    ? "ASAP"
                                    : String.format("%d", rr.getCriticalPoint()),
                            positionToSlowDown == null ? "" : String.format("%.1f", positionToSlowDown),
                            Double.isInfinite(distanceToCP) ? "" : String.format("%.1f", distanceToCP),
                            rr.statusString == null ? "-" : rr.statusString.replace("STOPPED_AT_CP", "STOP@CP")
                    ));
					thead2.append(" | path<br>index | no.<br>poses | CP<br>(index) | posTo<br>Slow, m | distance<br>ToCP, m | status");
				}
			}

            if (isExtendedText) {
				ArrayList<Mission> missions = Missions.getMissions(id);
				if (missions == null) {
					missions = new ArrayList<Mission>();
				}
				synchronized (missions) {
                    row.append(" | <div style=\"text-align: left;\">").append(missions.size()).append(": [");
					thead1.append(" | Dispatcher");
					theadHints.append(" | ");
					thead2.append(" | future<br>missions");
					for (int i = 0; i < missions.size(); i++) {
						if (i > 0) {
                            row.append(", ");
						}
						Mission mission = missions.get(i);
                        row.append(String.format("%d", mission.getPath().length));
					}
                    row.append("]</div>");
				}
			}

            tbodyHtml.append("<tr>");
			String[] parts = row.toString().split(" [|] ");
			for (int i = 0; i < parts.length; i++) {
				tbodyHtml.append("<td class=col").append(i).append(">").append(parts[i]).append("</td>");
			}
			tbodyHtml.append("</tr>\n");

			String[] rowData = row.toString().split(" [|] ");
			for (int i = 0; i < rowData.length; i++) {
				rowData[i] = htmlToCsv(rowData[i]);
			}
			idToRow.put(id, rowData);
		}

		assert thead1 != null;
		String[] columns = thead2.toString().split(" [|] ");
		for (int i = 0; i < columns.length; i++) {
			columns[i] = htmlToCsv(columns[i])
					.replace("[", "")
					.replace("]", "");
		}
		statsColumns = columns;
		statsIdToRow = idToRow;
		for (String[] rowData : statsIdToRow.values()) {
			assert rowData.length == statsColumns.length;
		}

		StringBuilder theadHtml = new StringBuilder();
		for (StringBuilder thead : List.of(thead1, thead2, theadHints)) {
			theadHtml.append("<tr>");
			String text = thead.toString().replaceAll("\\[.*?]", "");
			for (StringBuilder html : makeHeaderHtmls(text)) {
				theadHtml.append(html);
			}
			theadHtml.append("</tr>\n");
		}

		return new StringBuilder(
				"<style>\n" +
				"  table.info td {\n" +
				"    text-align: right;\n" +
				"  }\n" +
				"  table.info td,\n" +
				"  table.info th,\n" +
				"  table.info {\n" +
				"    border: 1px solid;\n" +
				"    border-collapse: collapse;\n" +
				"    padding: 3px;\n" +
				"  }\n" +
				"  table.info {\n" +
				"    margin: 5px;\n" +
				"  }\n" +
				"</style>\n" +
				"<table class=\"info\">\n" +
				"  <thead>\n"
		).append(theadHtml).append(
				"  </thead>\n" +
				"\n" +
				"  <tbody>\n"
		).append(tbodyHtml).append(
				"  </tbody>\n" +
				"</table>\n"
		);
	}

	private boolean isAnyCSWithHuman() {
		TrajectoryEnvelopeCoordinatorSimulation tec = TrajectoryEnvelopeCoordinatorSimulation.tec;
		for (CriticalSection cs : tec.allCriticalSections) {
			for (int id : cs.getRobotIDs()) {
				if (VehiclesHashMap.isHuman(id)) {
					return true;
				}
			}
		}
		return false;
	}

	public static String escapeHTML(String s) {
		// https://stackoverflow.com/a/25228492
		StringBuilder out = new StringBuilder(Math.max(16, s.length()));
		for (int i = 0; i < s.length(); i++) {
			char c = s.charAt(i);
			if (c > 127 || c == '"' || c == '\'' || c == '<' || c == '>' || c == '&') {
				out.append("&#");
				out.append((int) c);
				out.append(';');
			} else {
				out.append(c);
			}
		}
		return out.toString();
	}

	protected StringBuilder makePretable() {
		TrajectoryEnvelopeCoordinatorSimulation tec = TrajectoryEnvelopeCoordinatorSimulation.tec;

		Map<String, String> map = new LinkedHashMap<>();

		if (AbstractVehicle.scenarioId != null) {
			map.put("Scenario", AbstractVehicle.scenarioId);
		}

		map.put("Current datetime", java.time.LocalDateTime.now().format(
				DateTimeFormatter.ofPattern("yyyy-MM-dd HH:mm:ss")
		));

		if (Timekeeper.isTimekeeperActive()) {
			map.put("Time passed (real)", secondsToHMS(Timekeeper.getRealMillisPassed() / 1000));
			map.put("Time passed (sim.)", String.format("%s (x%.1f)",
					secondsToHMS(Timekeeper.getVirtualMillisPassed() / 1000),
					(double) Timekeeper.getVirtualMillisPassed() / Timekeeper.getRealMillisPassed()
			));
		}

		map.put("isCanPassFirstActive", String.format("hum=%b, aut=%b",
				CriticalSection.isCanPassFirstActiveHum,
				CriticalSection.isCanPassFirstActiveAut));
		map.put("isRacingThroughCrossroadAllowed", String.format("%b", AdaptiveTrajectoryEnvelopeTrackerRK4.isRacingThroughCrossroadAllowed));
		map.put("probabilitySlowingDownForHuman", String.valueOf(AdaptiveTrajectoryEnvelopeTrackerRK4.probabilitySlowingDownForHuman));

		if (HumanControl.status != null) {
			map.put("Human control status", HumanControl.status);
		}

		for (int id : new TreeSet<>(VehiclesHashMap.getList().keySet())) {
			if (!VehiclesHashMap.isHuman(id)) {
				continue;
			}

			RobotReport rr = TrajectoryEnvelopeCoordinatorSimulation.tec.getRobotReport(id);
			if (rr == null) {
				continue;
			}

			int numForcings = Forcing.robotIDToNumForcingEvents.getOrDefault(id, 0);

			String comment = "";
			if (isExtendedText && Forcing.isForcingActive()) {
				comment = " (forcing is active since " +
						secondsToHMS((long) Forcing.forcingSinceTimestep * Timekeeper.virtualMillisPerTimestep / 1000) +
						")";
				;
			}

			map.put("Human V" + id, String.format("<b>%d</b> forcing events",
					numForcings
			) + comment);
		}

		List<CollisionEvent> minorCollisions = tec.robotIDToMinorCollisions.getOrDefault(-1, new ArrayList<>());
		List<CollisionEvent> majorCollisions = tec.robotIDToMajorCollisions.getOrDefault(-1, new ArrayList<>());
		map.put("Collision events", String.format("<b>%d</b> minor%s, <b>%d</b> major%s",
				tec.allCollisionsList.size() - tec.majorCollisionsList.size(),
				stringifyCollisions(minorCollisions),
				tec.majorCollisionsList.size(),
				stringifyCollisions(majorCollisions)
		));

		String textEmergencyBreaker = AdaptiveTrajectoryEnvelopeTrackerRK4.emergencyBreaker.toString();
		if (isExtendedText && textEmergencyBreaker != null) {
			map.put("EmergencyBreaker", textEmergencyBreaker);
		}

		map.put("Vehicle size (m)", VehiclesHashMap.getTheHuman().vehicleSize.toString());

		StringBuilder htmlOutput = new StringBuilder(
				"<style>ul.pretable li { white-space: nowrap; }</style>\n<ul class=\"pretable\">"
		);
		Map<String, String> mapCsv = new LinkedHashMap<>();
		for (Map.Entry<String, String> entry : map.entrySet()) {
			String csv = htmlToCsv(entry.getValue());
			String title = escapeHTML(csv);
			htmlOutput.append("<li title=\"").append(title).append("\"><b>").append(
					entry.getKey()
			).append("</b>: ").append(entry.getValue()).append("</li>");
			mapCsv.put(htmlToCsv(entry.getKey()), csv);
		}
		htmlOutput.append("</ul>");
		BrowserVisualization.mapPretable = mapCsv;

		return htmlOutput;
	}

	private String stringifyCollisions(List<CollisionEvent> collisions) {
		if (collisions.isEmpty()) {
			return "";
		}
		StringBuilder output = new StringBuilder(" (");
		for (int i = 0; i < collisions.size(); i++) {
			CollisionEvent collision = collisions.get(i);
			if (i > 0) {
				output.append(", ");
			}
			output.append(String.format("%s: <b>V%d</b>[%d]-<b>V%d</b>[%d]",
					secondsToHMS(collision.getMillis() / 1000),
					collision.getReports()[0].getRobotID(),
					collision.getReports()[0].getPathIndex(),
					collision.getReports()[1].getRobotID(),
					collision.getReports()[1].getPathIndex()
			));
		}
		output.append(")");
		return output.toString();
	}

	protected void setStatusText() {
		StringBuilder output = new StringBuilder(makePretable());
		output.append(makeVehicleTableHtml());

		if (isExtendedText && areAllVehiclesStarted) {
//			html += "Last `getOrderOfCriticalSection` call was at step " + TrajectoryEnvelopeCoordinator.timestepOfLastCallOfGetOrderOfCriticalSection + "<br>";
			output.append(stringifyCriticalSections(TrajectoryEnvelopeCoordinatorSimulation.tec.allCriticalSections));
		}

		setOverlayText(output.toString());
	}

	protected static StringBuilder stringifyCriticalSections(HashSet<CriticalSection> allCriticalSections) {
		Integer robotID = null;

		ArrayList<CriticalSection> criticalSections =
				robotID == null
				? CriticalSection.sortCriticalSections(allCriticalSections)
				: CriticalSection.sortCriticalSectionsForRobotID(allCriticalSections, robotID);

		StringBuilder text = new StringBuilder("Critical sections" + (robotID == null ? "" : " for " + robotID) + ":");
		if (criticalSections.isEmpty()) {
			text.append(" none<br>\n");
		} else {
			text.append("<ul style=\"margin-block: 0px;\">");
			for (CriticalSection cs : criticalSections) {
				text.append("<li>").append(cs.toString(true)).append("</li>\n");
			}
			text.append("</ul>\n");
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
		StringBuilder ret = new StringBuilder("{ \"name\" : \"" + name + "\", \"color\" : \"" + color + "\", ");
		if (age > 0) ret.append(" \"age\" : ").append(age).append(", ");
		ret.append(" \"filled\" : ").append(filled).append(", ");
		if (extraData != null && !extraData.trim().isEmpty()) ret.append(" \"extraData\" : \"").append(extraData).append("\", ");
		ret.append("\"coordinates\" : [");
		Coordinate[] coords = geom.getCoordinates();
		for (int i = 0; i < coords.length; i++) {
			ret.append("{\"x\" : ").append(coords[i].x).append(", \"y\" : ").append(coords[i].y).append("}");
			if (i < coords.length-1) ret.append(", ");
		}
		ret.append("]}");
		return ret.toString();
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
		double factor = 10.0 * Missions.getDynamicMap().resolution;
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

	@Override
	public void setMap(String mapYAMLFile) {
		BrowserVisualizationSocket.dynamicMap = new DynamicMap(mapYAMLFile);
	}
	
	@Override
	public int periodicEnvelopeRefreshInMillis() {
		return 1000;
	}
}
