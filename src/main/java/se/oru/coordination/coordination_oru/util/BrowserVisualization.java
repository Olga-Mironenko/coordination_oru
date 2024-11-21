package se.oru.coordination.coordination_oru.util;


import java.awt.Desktop;
import java.awt.Dimension;
import java.io.IOException;
import java.net.URI;
import java.net.URISyntaxException;
import java.time.format.DateTimeFormatter;
import java.util.*;

import com.google.gson.Gson;
import com.google.gson.JsonObject;

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
	
	private ArrayList<String> msgQueue = new ArrayList<String>();
	private static int UPDATE_PERIOD = 30;
	private String overlayText = null;

	public static boolean isStatusText = false;
	public static boolean isExtendedText = false;
	public static boolean isCollisionInfo = false;

	public static boolean areAllVehiclesStarted = false;

	public static void makeScreenshot() {
        Process process;
        try {
			ProcessBuilder builder = new ProcessBuilder("screenshotting/make-screenshot.sh",
					"screenshotting/screenshots/" + AbstractVehicle.getScenarioIdAsBasename() + ".png");
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
        assert code == 0;
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
		if (Desktop.isDesktopSupported() && Desktop.getDesktop().isSupported(Desktop.Action.BROWSE)) {
			try { Desktop.getDesktop().browse(new URI("http://" + serverHostNameOrIP + ":8080")); }
			catch (IOException e) { e.printStackTrace(); }
			catch (URISyntaxException e) { e.printStackTrace(); }
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
		connector.setIdleTimeout(30 * 60 * 1000); // 30 min. (for long pauses inside the debugger)
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
		String extraData = "";

		// Show path Index
//		String extraData = " : " + rr.getPathIndex();
		if (extraStatusInfo != null) {
			for (String st : extraStatusInfo) {
				extraData += " | " + st;
			}
		}

		String colorOuter = "#bbbbbb";

		String colorInner = "#000000";
		int vehicleCount = VehiclesHashMap.getList().keySet().size();
		if (vehicleCount != 0) colorInner = VehiclesHashMap.getVehicle(rr.getRobotID()).getColorCode();

		drawRobotFootprint(x, y, theta, rr.getPose(), colorOuter, name, extraData, false, te.getFootprint());

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

	protected String makeVehicleTableHtml() {
		String text = "";
		String thead1 = "";
		String thead2 = "";
		String tbodyHtml = "";

		HashMap<Integer, AbstractVehicle> idToVehicle = VehiclesHashMap.getList();
		TrajectoryEnvelopeCoordinatorSimulation tec = TrajectoryEnvelopeCoordinatorSimulation.tec;

		for (int id : idToVehicle.keySet()) {
			AbstractVehicle vehicle = idToVehicle.get(id);
			boolean isHuman = VehiclesHashMap.isHuman(id);
			AbstractTrajectoryEnvelopeTracker tracker = tec.trackers.get(id);
			AdaptiveTrajectoryEnvelopeTrackerRK4 trackerAdaptive =
					(tracker instanceof AdaptiveTrajectoryEnvelopeTrackerRK4)
							? (AdaptiveTrajectoryEnvelopeTrackerRK4) tracker
							: null;

			TrajectoryEnvelope te = tracker.getTrajectoryEnvelope();
			text += "(V" + id + ", " + vehicle.getType() + ") ";
			String row = "<div style=\"text-align: left;\"><b>V" + id + "</b>, " + vehicle.getType() + "</div>";
			thead1 = "";
			thead2 = "Vehicle ID and type";

			RobotReport rr = tec.getRobotReport(id);
			if (rr == null) {
				text += "no robot report";
			} else {
				double velocity = rr.getVelocity();
				text += String.format("v=<b>%.1f</b> m/s (max: %.1f m/s)", velocity, vehicle.getMaxVelocity());
				row += String.format(" | %s%.1f | %.1f",
						String.format("%.1f", velocity).equals("0.0") && velocity != 0.0 ? "~" : "",
						velocity,
						vehicle.getMaxVelocity()
				);
				thead1 += " |2 Velocity, m/s";
				thead2 += " | current | max";

				thead1 += " |3 Human (mis)behaviour actions";
				thead2 += " | violation of<br>priorities | moving<br>slowly | improper<br>parking";
				if (! isHuman) {
					KnobsAfterForcing knobsAfterForcing = ForcingMaintainer.getKnobsOfTheHuman();
					boolean isToRestore = knobsAfterForcing != null && knobsAfterForcing.isToRestore(id);
					boolean isToResume = knobsAfterForcing != null && knobsAfterForcing.isToResume(id);
					row += String.format(" | %s |  | ",
							center(isToRestore || isToResume ? "affected" : "")
					);
				} else {
					boolean isForcingOngoing = (
							trackerAdaptive != null &&
							trackerAdaptive.forcingMaintainer != null &&
							trackerAdaptive.forcingMaintainer.isForcingOngoing()
					);
					row += String.format(" | %s",
							center(
								! isForcingOngoing
										? ""
										: AdaptiveTrajectoryEnvelopeTrackerRK4.probabilityForcingForHuman < 1.0
										? "random"
										: "constant"
							)
					);
					row += String.format(" | %s",
							center(
								! vehicle.isMaxVelocityLowered()
										? ""
										: "yes"
							)
					);
					row += " | ";
				}

				thead1 += " |5 Coordination strategies for AVs";
				thead2 += (
						" | cautious<br>mode | rerouting at<br>parked / slow | moving<br>backwards" +
						" | change of<br>priorities | stops"
				);
				if (isHuman) {
					row += " |  |  |  |  | ";
				} else {
					KnobsAfterForcing knobsAfterForcing = ForcingMaintainer.getKnobsOfTheHuman();
					boolean isToRestore = knobsAfterForcing != null && knobsAfterForcing.isToRestore(id);
					boolean isToResume = knobsAfterForcing != null && knobsAfterForcing.isToResume(id);
					row += String.format(" | %s | %s |  | %s | %s",
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
							center(isToRestore && ! isToResume ? "temporary" : ""),
							center(isToResume ? Forcing.isGlobalTemporaryStop ? "global" : "local" : "")
					);
				}

				List<CollisionEvent> allCollisions = tec.robotIDToAllCollisions.getOrDefault(id, new ArrayList<>());
				List<CollisionEvent> minorCollisions = tec.robotIDToMinorCollisions.getOrDefault(id, new ArrayList<>());
				List<CollisionEvent> majorCollisions = tec.robotIDToMajorCollisions.getOrDefault(id, new ArrayList<>());
				assert allCollisions.size() == minorCollisions.size() + majorCollisions.size();

				thead1 += " |3 Safety-critical events";
				thead2 += " | violations | near<br>misses | collisions";
				if (! isHuman) {
					row += " | ";
				} else {
					int numForcings = Forcing.robotIDToNumForcingEvents.getOrDefault(id, 0);
					int numUselessForcings = Forcing.robotIDToNumUselessForcingEvents.getOrDefault(id, 0);
					int numViolations = numForcings - numUselessForcings;
					row += String.format(" | %d", numViolations);
				}
				text += String.format("; collision events: <b>%d</b> minor, <b>%d</b> major", minorCollisions.size(), majorCollisions.size());
				row += String.format(" | %d</b> | %d", minorCollisions.size(), majorCollisions.size());

				if (isCollisionInfo && ! isHuman) {
					if (!allCollisions.isEmpty()) {
						for (CollisionEvent ce : allCollisions) {
							text += "- " + ce.toCompactString(rr.getRobotID()) + "<br>";
						}
					}
				}

				text += String.format("; traveled <b>%.1f m</b>", vehicle.totalDistance);
				row += String.format(" | %.1f | %d | %s",
						vehicle.totalDistance,
 						vehicle.getNumMissions(),
						center(vehicle.isBlocked() ? "yes" : "")
				);
				thead1 += " |3 Efficiency";
				thead2 += " | traveled<br>total, m | no.<br>missions | blocked";

				if (isExtendedText) {
					text += String.format("; p=(%.1f, %.1f)", rr.getPose().getX(), rr.getPose().getY());
					row += String.format(" | (%.1f, %.1f) | %.1f",
							rr.getPose().getX(), rr.getPose().getY(),
							rr.getDistanceTraveled()
					);
					thead1 += " |8 Tracker state (current mission)";
					thead2 += " | position<br>(x, y), m | traveled,<br>m";

					text += String.format("; i=%d (CP=%d, %s)",
							rr.getPathIndex(), rr.getCriticalPoint(), rr.statusString != null ? rr.statusString : "-"
					);
					Double positionToSlowDown = trackerAdaptive == null ? null : trackerAdaptive.positionToSlowDown;
					double distanceToCP =
							trackerAdaptive == null ? Double.POSITIVE_INFINITY : trackerAdaptive.distanceToCP;
					row += String.format(" | %s | %d | %s | %s | %s | <div style=\"text-align: left;\">%s</div>",
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
					);
					thead2 += " | path<br>index | no.<br>poses | CP<br>(index) | posTo<br>Slow, m | distance<br>ToCP, m | status";

//					int numCalls = 0;
//					var numIntegrateCalls = TrajectoryEnvelopeCoordinatorSimulation.tec.numIntegrateCalls;
//					if (numIntegrateCalls.containsKey(id)) {
//						numCalls = numIntegrateCalls.get(id);
//					}
//					text += "; numIntegrateCalls: " + numCalls;

//					text += "; traveled " + round(rr.getElapsedTrackingTime()) + " s (sim. time)";
				}
			}

			if (isExtendedText) {
				ArrayList<Mission> missions = Missions.getMissions(id);
				if (missions == null) {
					missions = new ArrayList<Mission>();
				}
				synchronized (missions) {
					text += "; " + missions.size() + " future missions: [";
					row += " | <div style=\"text-align: left;\">" + missions.size() + ": [";
					thead1 += " | Dispatcher";
					thead2 += " | future<br>missions";
					for (int i = 0; i < missions.size(); i++) {
						if (i > 0) {
							text += ", ";
							row += ", ";
						}
						Mission mission = missions.get(i);
						text += String.format("%d poses", mission.getPath().length);
						row += String.format("%d", mission.getPath().length);
					}
					text += "]";
					row += "]</div>";
				}
			}

			text += "<br>";
			tbodyHtml += "<tr> <td>" + row.replace(" | ", "</td> <td>") + "</td> </tr>\n";
		}

		String thead2Html = "<tr> <th>" + thead2.replace(" | ", "</th> <th>") + "</th> </tr>\n";
		String thead1Html = "<tr> <th>" + thead1.replaceAll(" [|]([2-9]?) ", "</th> <th colspan=\"$1\">") + "</th> </tr>\n";
		return "<style>\n" +
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
				"  <thead>\n" + thead1Html + thead2Html +
				"  </thead>\n" +
				"\n" +
				"  <tbody>\n" + tbodyHtml +
				"  </tbody>\n" +
				"</table>\n";
	}
	
	protected void setStatusText() {
		TrajectoryEnvelopeCoordinatorSimulation tec = TrajectoryEnvelopeCoordinatorSimulation.tec;

		String html = "";

		if (AbstractVehicle.scenarioId != null) {
			html += "Scenario: " + AbstractVehicle.scenarioId + "<br>";
		}

		html += String.format("Current datetime: %s<br>", java.time.LocalDateTime.now().format(
				DateTimeFormatter.ofPattern("yyyy-MM-dd HH:mm:ss")
		));
		if (Timekeeper.isTimekeeperActive()) {
			html += String.format("Time passed (real): %s<br>", secondsToHMS(Timekeeper.getRealMillisPassed() / 1000));
			html += String.format("Time passed (sim.): %s<br>", secondsToHMS(Timekeeper.getVirtualMillisPassed() / 1000));
		}

		html += String.format("isCanPassFirstActive: %b<br>", CriticalSection.isCanPassFirstActive);
		html += String.format("isRacingThroughCrossroadAllowed: %b<br>", AdaptiveTrajectoryEnvelopeTrackerRK4.isRacingThroughCrossroadAllowed);

		if (HumanControl.status != null) {
			html += HumanControl.status + "<br>";
		}

		for (int id : new TreeSet<>(VehiclesHashMap.getList().keySet())) {
			if (! VehiclesHashMap.isHuman(id)) {
				continue;
			}

			RobotReport rr = TrajectoryEnvelopeCoordinatorSimulation.tec.getRobotReport(id);
			if (rr == null) {
				continue;
			}

			int numForcings = Forcing.robotIDToNumForcingEvents.getOrDefault(id, 0);
			html += String.format(
					"Human V%d: <b>%d</b> forcing events",
					id,
					numForcings
			);
			if (isExtendedText && Forcing.forcingSinceTimestep != -1) {
				html += " (forcing is active since step " + Forcing.forcingSinceTimestep + ")";
			}
			html += "<br>";
		}

		html += String.format(
				"Collision events total: <b>%d</b> minor, <b>%d</b> major<br>",
				tec.allCollisionsList.size() - tec.majorCollisionsList.size(),
				tec.majorCollisionsList.size()
		);

		String textEmergencyBreaker = AdaptiveTrajectoryEnvelopeTrackerRK4.emergencyBreaker.toString();
		if (isExtendedText && textEmergencyBreaker != null) {
			html += "EmergencyBreaker: " + textEmergencyBreaker + "<br>";
		}

		html += "Vehicle size (m): " + VehiclesHashMap.getTheHuman().vehicleSize + "<br>";

		html += makeVehicleTableHtml();

		if (isExtendedText && areAllVehiclesStarted) {
//			html += "Last `getOrderOfCriticalSection` call was at step " + TrajectoryEnvelopeCoordinator.timestepOfLastCallOfGetOrderOfCriticalSection + "<br>";
			html += stringifyCriticalSections(TrajectoryEnvelopeCoordinatorSimulation.tec.allCriticalSections);
		}

		setOverlayText(html);
	}

	protected static String stringifyCriticalSections(HashSet<CriticalSection> allCriticalSections) {
		Integer robotID = null;

		ArrayList<CriticalSection> criticalSections =
				robotID == null
				? CriticalSection.sortCriticalSections(allCriticalSections)
				: CriticalSection.sortCriticalSectionsForRobotID(allCriticalSections, robotID);

		String text = "Critical sections" + (robotID == null ? "" : " for " + robotID) + ":";
		if (criticalSections.isEmpty()) {
			text += " none<br>\n";
		} else {
			text += "<ul style=\"margin-block: 0px;\">";
			for (CriticalSection cs : criticalSections) {
				text += "<li>" + cs.toString(true) + "</li>\n";
			}
			text += "</ul>\n";
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

	@Override
	public void setMap(String mapYAMLFile) {
		BrowserVisualizationSocket.dynamicMap = new DynamicMap(mapYAMLFile);
	}
	
	@Override
	public int periodicEnvelopeRefreshInMillis() {
		return 1000;
	}
}
