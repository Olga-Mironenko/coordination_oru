package se.oru.coordination.coordination_oru.util;

import org.apache.commons.lang.ArrayUtils;
import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.PoseSteering;

import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelope;
import se.oru.coordination.coordination_oru.*;
import se.oru.coordination.coordination_oru.code.AbstractVehicle;
import se.oru.coordination.coordination_oru.code.BarrierPhantomVehicle;
import se.oru.coordination.coordination_oru.code.VehiclesHashMap;
import se.oru.coordination.coordination_oru.simulation2D.State;
import se.oru.coordination.coordination_oru.simulation2D.TrajectoryEnvelopeCoordinatorSimulation;
import se.oru.coordination.coordination_oru.util.gates.GatedThread;

import java.util.*;

public class MissionUtils {
    public static final double targetVelocityHumanInitial = Double.POSITIVE_INFINITY; // essentially a limit
    public static double targetVelocityHuman = targetVelocityHumanInitial;
    public static int idHuman = 0;
    public static boolean isWorking = false;
    public static boolean arePhantomsVisible = false; // TODO: remove
    public static HashMap<Integer, Integer> robotIDToFreezingCounter = new HashMap<>(); // TODO: use semaphores
    public static double stopDistance = Double.NEGATIVE_INFINITY;
    public static double priorityDistance = Double.NEGATIVE_INFINITY;

    protected static HashMap<Integer, BarrierPhantomVehicle> robotIDToBarrier = new HashMap<>(); // TODO: remove

    // TODO: race condition (click/keypress)
    // TODO: crashes on click and then (immediately) keypress
    // TODO: crashes on large initial velocity

    protected static Object pathLock = new Object(); // sentinel

    public static TreeMap<Integer, Integer> robotIDToNumForcingEvents = new TreeMap<>();

    protected static void removeMissions(int robotID) {
        while (true) {
            var mission = Missions.dequeueMission(robotID);
            if (mission == null) {
                break;
            }
        }
    }

    public static void moveRobot(int robotID, Pose goal) {
        if (isWorking) {
            return;
        }
        isWorking = true;
        try {
            synchronized (pathLock) {
                //waitUntilScheduledMissionStarts(robotID);

                TrajectoryEnvelopeCoordinatorSimulation tec = TrajectoryEnvelopeCoordinatorSimulation.tec;
                Object stateOrNothingAsLock = getRobotState(robotID);
                if (stateOrNothingAsLock == null) {
                    stateOrNothingAsLock = new Object();
                }

                //synchronized (stateOrNothingAsLock) { // seems like a deadlock with the tracker
                RobotReport rr = tec.getRobotReport(robotID);
                Pose currentPose = rr.getPose();
                var vehicle = VehiclesHashMap.getVehicle(robotID);

                PoseSteering[] newPath = null;
                try {
                    newPath = vehicle.getPlan(currentPose, new Pose[]{goal}, Missions.getMapYAMLFilename(), false);
                } catch (Error exc) { // TODO: check for NoPathFound only
                    System.out.println("moveRobot: no path found (or another error): " + exc);
                    return;
                }

                PoseSteering[] currentPath = getCurrentPath(robotID);
                if (currentPath == null || rr.getPathIndex() == -1) {
                    targetVelocityHuman = targetVelocityHumanInitial;
                    Missions.enqueueMission(new Mission(robotID, newPath));
                } else {
                    int replacementIndex = getReplacementIndex(robotID);
                    PoseSteering[] replacementPath = computeReplacementPath(currentPath, replacementIndex, newPath);
                    MissionUtils.changePath(robotID, replacementPath, replacementIndex);
                }
            }
        } finally {
            isWorking = false;
        }
    }

    protected static void waitUntilScheduledMissionStarts(int robotID) {
        TrajectoryEnvelopeCoordinatorSimulation tec = TrajectoryEnvelopeCoordinatorSimulation.tec;

        // If there is a scheduled mission, wait until it starts.
        try {
            while (! tec.isMissionsPoolEmpty() || Missions.hasMissions(robotID))
                GatedThread.sleep(50);
        }
        catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    protected static State getRobotState(int robotID) {
        TrajectoryEnvelopeCoordinatorSimulation tec = TrajectoryEnvelopeCoordinatorSimulation.tec;
        AbstractTrajectoryEnvelopeTracker tracker = tec.getTracker(robotID);
        if (tracker == null) {
            return null;
        }
        return tracker.getState();
    }

    // 0.1->1.1: OK
    // 0.1->1.1->0.1: OK
    // 0.1->1.1->2.1: breaks
    // 0.1->1.1->0.1->1.1: breaks
    public static void changeTargetVelocityHuman(double delta) {
        if (isWorking) {
            return;
        }
        isWorking = true;
        try {
            // TODO: sometimes doesn't get called
            // ("hint": try to pause the websocket thread in the debugger)
            int robotID = idHuman;
            double targetVelocityNew = targetVelocityHuman + delta;
            if (targetVelocityNew > 0) {
                targetVelocityHuman = targetVelocityNew;

                synchronized (pathLock) {
                    PoseSteering[] currentPath = getCurrentPath(robotID);
                    if (currentPath != null) {
                        changePath(robotID, currentPath, getReplacementIndex(robotID));
                    }
                }
            }
        }
        finally {
            isWorking = false;
        }
    }

    protected static PoseSteering[] getCurrentPath(int robotID) {
        TrajectoryEnvelope te = TrajectoryEnvelopeCoordinatorSimulation.tec.getCurrentTrajectoryEnvelope(robotID);
        if (te == null) {
            return null;
        }
        return te.getSpatialEnvelope().getPath();
    }

    protected static int getPathIndex(int robotID) {
        TrajectoryEnvelopeCoordinatorSimulation tec = TrajectoryEnvelopeCoordinatorSimulation.tec;
        RobotReport rr = tec.getRobotReport(robotID);
        return rr.getPathIndex();
    }

    protected static int getReplacementIndex(int robotID) {
        return Math.max(0, getPathIndex(robotID) + 2);
        // TODO: why does `- 10` work better than `+ 10`?
    }

    protected static void changePath(int robotID, PoseSteering[] replacementPath, int replacementIndex) {
        TrajectoryEnvelopeCoordinatorSimulation tec = TrajectoryEnvelopeCoordinatorSimulation.tec;

        tec.replacePath(robotID, replacementPath, replacementIndex, false, null);

        int replacementIndexNew = getReplacementIndex(robotID);
        System.out.println("changePath1: replacementIndex: " + replacementIndex + " -> " + replacementIndexNew);
        // TODO: It changes significantly sometimes.
    }

    public static PoseSteering[] computeReplacementPath(PoseSteering[] initialPath, int replacementIndex, PoseSteering[] newPath) {
        replacementIndex = Math.min(replacementIndex, initialPath.length - 1); // TODO
        PoseSteering[] replacementPath = new PoseSteering[replacementIndex + newPath.length];
        // TODO: replacementIndex: off by one error? (replacementIndex=2 -> preserve 3 points)
        for (int i = 0; i < replacementIndex; i++) replacementPath[i] = initialPath[i];
        for (int i = 0; i < newPath.length; i++) replacementPath[i + replacementIndex] = newPath[i];
        return replacementPath;
    }

    public static void forceDriving(int robotID) {
        robotIDToNumForcingEvents.put(robotID, robotIDToNumForcingEvents.getOrDefault(robotID, 0) + 1);

        TrajectoryEnvelopeCoordinatorSimulation tec = TrajectoryEnvelopeCoordinatorSimulation.tec;

        final ArrayList<CriticalSection> criticalSectionsForPriority =
                selectCriticalSections(robotID, tec.allCriticalSections, priorityDistance, Integer.MAX_VALUE);
        for (CriticalSection cs : criticalSectionsForPriority) {
            cs.setHigher(robotID, true);
        }

        final ArrayList<CriticalSection> criticalSectionsForStop =
                selectCriticalSections(robotID, tec.allCriticalSections, stopDistance, Integer.MAX_VALUE);

        TreeSet<Integer> robotsToStop = new TreeSet<>();
        for (CriticalSection cs : criticalSectionsForStop) {
            int robotToStop;
            if (cs.isTe1(robotID)) {
                robotToStop = cs.getTe2RobotID();
            } else if (cs.isTe2(robotID)) {
                robotToStop = cs.getTe1RobotID();
            } else {
                throw new RuntimeException();
            }
            assert robotToStop != robotID;

            if (! cs.isRobotOnCS(robotToStop)) {
                robotsToStop.add(robotToStop);
            }
        }
        for (int robot : robotsToStop) {
            stopRobot(robot);
        }

        TrackingCallback cb = new TrackingCallback(null) {
            @Override
            public void onTrackingStart() { }

            @Override
            public void onTrackingFinished() { }

            @Override
            public String[] onPositionUpdate() {
                if (areSomeCriticalSectionsWithHighPriorityGone(tec.allCriticalSections, criticalSectionsForPriority)) {
                    for (CriticalSection cs : criticalSectionsForPriority) {
                        if (tec.allCriticalSections.contains(cs)) {
                            cs.setHigher(robotID, false);
                        }
                    }
                    criticalSectionsForPriority.clear();

                    for (int robot : robotsToStop) {
                        resumeRobot(robot);
                    }
                }
                return null;
            }

            @Override
            public void onNewGroundEnvelope() { }

            @Override
            public void beforeTrackingStart() { }

            @Override
            public void beforeTrackingFinished() { }
        };

        tec.addTrackingCallback(robotID, cb);
    }

    protected static ArrayList<CriticalSection> selectCriticalSections(
            int robotID,
            HashSet<CriticalSection> allCriticalSections,
            double maxDistance,
            int maxCount) {
        assert maxCount >= 0;
        if (maxCount == 0) {
            return new ArrayList<>();
        }

        ArrayList<CriticalSection> criticalSectionsSorted =
                CriticalSection.sortCriticalSectionsForRobotID(allCriticalSections, robotID);
        ArrayList<CriticalSection> criticalSectionsSelected = new ArrayList<>();

        if (maxDistance == Double.NEGATIVE_INFINITY) {
            return criticalSectionsSelected;
        }
        assert maxDistance >= 0.0;

        PoseSteering[] currentPath = getCurrentPath(robotID);
        int indexCurrent = getPathIndex(robotID);
        double distance = 0.0;

        for (CriticalSection cs : criticalSectionsSorted) {
            if (cs.getTe1() == null || cs.getTe2() == null) {
                continue;
            }

            Integer indexSection = cs.getStart(robotID);
            assert indexSection != null;
            if (indexSection == -1) {
                continue;
            }

            //assert indexSection >= indexCurrent;  // commented because of a bug
            while (indexSection > indexCurrent) {
                PoseSteering poseCurrent = currentPath[indexCurrent];
                PoseSteering poseNext = currentPath[indexCurrent + 1];
                double step = BrowserVisualization.computeDistanceBetweenPoses(poseCurrent.getPose(), poseNext.getPose());
                assert step >= 0.0;
                distance += step;
                indexCurrent += 1;

                if (distance > maxDistance) {
                    break;
                }
            }
            if (distance > maxDistance) {
                break;
            }

            criticalSectionsSelected.add(cs);
            if (criticalSectionsSelected.size() == maxCount) {
                break;
            }
        }

        assert criticalSectionsSelected.size() <= maxCount;
        return criticalSectionsSelected;
    }

    protected static boolean areAllCriticalSectionsWithHighPriorityGone(
            HashSet<CriticalSection> allCriticalSections,
            ArrayList<CriticalSection> criticalSectionsWithHighPriority) {
        for (CriticalSection cs : criticalSectionsWithHighPriority) {
            if (allCriticalSections.contains(cs)) {
                return false;
            }
        }
        return true;
    }

    protected static boolean areSomeCriticalSectionsWithHighPriorityGone(
            HashSet<CriticalSection> allCriticalSections,
            ArrayList<CriticalSection> criticalSectionsWithHighPriority) {
        for (CriticalSection cs : criticalSectionsWithHighPriority) {
            if (! allCriticalSections.contains(cs)) {
                return true;
            }
        }
        return false;
    }

    protected static void stopRobot(int robotID) {
        System.out.println(robotID);

        robotIDToFreezingCounter.put(robotID, robotIDToFreezingCounter.getOrDefault(robotID, 0) + 1);

        /*
        BarrierPhantomVehicle vehicleBarrier = robotIDToBarrier.get(robotID);
        if (vehicleBarrier == null) {
            AbstractVehicle vehicleToStop = VehiclesHashMap.getVehicle(robotID);
            vehicleBarrier = new BarrierPhantomVehicle(vehicleToStop);
            vehicleBarrier.isActive = true;
            robotIDToBarrier.put(robotID, vehicleBarrier);
            // TODO: narrow robot or (0, 0)?
        }

        TrajectoryEnvelopeCoordinatorSimulation tec = TrajectoryEnvelopeCoordinatorSimulation.tec;
        RobotReport rr = tec.getRobotReport(robotID);

        PoseSteering[] currentPath = getCurrentPath(robotID);
        int numIndices = 10;
        // 2:  1- 2+
        // 3:  1- 2-
        // 4:  1- 2+
        // 5:  1- 2-
        // 10: 1+ 2-
        int currentIndex = rr.getPathIndex();
        if (currentIndex == -1) {
            return;
        }
        int lastIndex = currentIndex + numIndices - 1;
        if (lastIndex >= currentPath.length) {
            return;
        }
        PoseSteering[] path = new PoseSteering[numIndices];
        for (int offset = 0; offset < numIndices; offset++) {
            path[offset] = currentPath[currentIndex + offset];
        }

        tec.placeRobot(vehicleBarrier.getID(), path[0].getPose());
        Missions.dispatchableRobots.add(vehicleBarrier.getID());
        Missions.enqueueMission(new Mission(vehicleBarrier.getID(), path));
         */
    }

    protected static void resumeRobot(int robotID) {
        System.out.println(robotID);

        robotIDToFreezingCounter.put(robotID, robotIDToFreezingCounter.getOrDefault(robotID, 0) - 1);

        /*
        BarrierPhantomVehicle vehicleBarrier = robotIDToBarrier.get(robotID);
        vehicleBarrier.isActive = false;
        TrajectoryEnvelopeCoordinatorSimulation tec = TrajectoryEnvelopeCoordinatorSimulation.tec;
        //tec.placeRobot(vehicleBarrier.getID(), new Pose(0, 0, 0));
        tec.truncateEnvelope(vehicleBarrier.getID());
        //tec.placeRobot(vehicleBarrier.getID(), new Pose(0, 0, 0));

         */
    }
}