package se.oru.coordination.coordination_oru.util;

import org.metacsp.multi.spatioTemporal.paths.PoseSteering;
import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelope;
import se.oru.coordination.coordination_oru.CriticalSection;
import se.oru.coordination.coordination_oru.RobotReport;
import se.oru.coordination.coordination_oru.code.AbstractVehicle;
import se.oru.coordination.coordination_oru.code.VehiclesHashMap;
import se.oru.coordination.coordination_oru.simulation2D.AdaptiveTrajectoryEnvelopeTrackerRK4;
import se.oru.coordination.coordination_oru.simulation2D.TrajectoryEnvelopeCoordinatorSimulation;
import se.oru.coordination.coordination_oru.util.gates.GatedThread;
import se.oru.coordination.coordination_oru.util.gates.Timekeeper;

import java.util.*;

public class Forcing {
    public static TreeMap<Integer, Integer> robotIDToNumForcingEvents = new TreeMap<>();
    public static TreeMap<Integer, Integer> robotIDToNumUselessForcingEvents = new TreeMap<>();

    /**
     * Each time a robot does forcing, it increases counters for all affected robots.
     * Each time a robot stops forcing, it decreases counters for all affected robots.
     * An affected robot may drive only if its counter is zero (or if it's on an intersection).
     * Namely, if an affected robot is already on an intersection, it ignores the counter.
     */
    public static HashMap<Integer, Integer> robotIDToFreezingCounter = new HashMap<>(); // TODO: use semaphores
    /**
     * If a robot should have been frozen while on an intersection,
     * it postpones freezing until the specified path index.
     */
    public static HashMap<Integer, Integer> robotIDToPathIndexToStop = new HashMap<>();

    /**
     * Forcing parameters:
     */
    // - The priority effect of forcing will be the first `priorityDistance` (plus `distanceToCP`
    //   with `isDistanceToCPForPriority`) meters after the forcing begins.
    public static double priorityDistance = Double.NEGATIVE_INFINITY;
    public static boolean isDistanceToCPAddedToPriorityDistance = false;
    // - During forcing, its priority effect will affect only critical sections that are at least
    //   `priorityDistanceMin` meters after the beginning of the forcing.
    public static double priorityDistanceMin = Double.NEGATIVE_INFINITY;
    // - The stop effect of forcing will be the first `priorityDistance` meters after forcing begins.
    public static double stopDistance = Double.NEGATIVE_INFINITY;
    public static boolean isDistanceToCPAddedToStopDistance = false;
    // - During forcing, its stop effect will affect only critical sections that are at least
    //   `stopDistanceMin` meters after the beginning of the forcing.
    //   Note that stop effect without priority effect makes little sense.
    public static double stopDistanceMin = Double.NEGATIVE_INFINITY;
    // - When both change of priorities and stops apply, the latter is selected with the following probability.
    public static double probabilityStopNotChangeOfPriorities = 0.5;
    // - If `isGlobalTemporaryStop` is true, then all other robots are stopped during forcing.
    public static boolean isGlobalTemporaryStop = false;
    // - If `isResetAfterCurrentCrossroad` is true, then forcing finishes automatically when its priority and stop
    //   effects end. If the flag is false, then it's needed to finish forcing manually.
    public static boolean isResetAfterCurrentCrossroad = true;
    public static boolean isForcingToBeIgnored = false;

    private final static int maxNumberOfHumans = 1;

    public static int forcingSinceTimestep = -1;

    private static double computeDistanceToCS(int humanID, int inferiorID, ArrayList<CriticalSection> cses) {
        assert ! cses.isEmpty();

        RobotReport rr = TrajectoryEnvelopeCoordinatorSimulation.tec.getRobotReport(inferiorID);
        int inferiorPathIndex = rr.getPathIndex();
        int minPathIndex = Integer.MAX_VALUE;
        for (CriticalSection cs : cses) {
            assert cs.getTe1RobotID() == humanID && cs.getTe2RobotID() == inferiorID;
            minPathIndex = Math.min(minPathIndex, cs.getTe2Start());
        }
        if (minPathIndex <= inferiorPathIndex) {
            return 0.0; // The robot is already on a CS with forcing.
        }

        TrajectoryEnvelope teInferior =
                TrajectoryEnvelopeCoordinatorSimulation.tec.getTracker(inferiorID).getTrajectoryEnvelope();
        return AdaptiveTrajectoryEnvelopeTrackerRK4.computeDistance(
                teInferior.getTrajectory(), inferiorPathIndex, minPathIndex
        );
    }

    public static void startManualForcing(int robotID) {
        priorityDistance = 10;
//        stopDistance = 10;

        AbstractVehicle hum0 = VehiclesHashMap.getVehicle(robotID);
        RobotReport rrAtForcingStart = hum0.getCurrentRobotReport();
        KnobsAfterForcing knobsAfterForcing = Forcing.forceDriving(robotID);
        if (knobsAfterForcing == null) {
            return;
        }

        new GatedThread("manual forcing thread") {
            @Override
            public void runCore() {
                while (true) {
                    double distanceTraveled = hum0.getCurrentRobotReport().getDistanceTraveled() - rrAtForcingStart.getDistanceTraveled();

                    if (!knobsAfterForcing.updateForcing(distanceTraveled)) {
                        break;
                    }

                    GatedThread.sleepWithoutTryCatch(100);
                }
            }
        }.start();
    }

    public static boolean isForcingActive() {
        return forcingSinceTimestep != -1;
    }

    public static KnobsAfterForcing forceDriving(int robotID) {
        assert ! isForcingActive();
        new Event.ForcingStarted(robotID).write();

        forcingSinceTimestep = Timekeeper.getTimestepsPassed();
        robotIDToNumForcingEvents.put(robotID, robotIDToNumForcingEvents.getOrDefault(robotID, 0) + 1);

        final TrajectoryEnvelopeCoordinatorSimulation tec = TrajectoryEnvelopeCoordinatorSimulation.tec;


        return new KnobsAfterForcing() {
            boolean isForcingFinished = false;
            final Random rand = new DeterministicRandom(Forcing.class.getName(), robotID);

            final HashSet<CriticalSection> criticalSectionsToRestorePrioritiesLater = new HashSet<>();
            HashSet<CriticalSection> criticalSectionsAtDone = null;

            final TreeSet<Integer> robotsToRestoreLater = new TreeSet<>();
            final TreeSet<Integer> robotsToResumeLater = new TreeSet<>();
            final TreeMap<Integer, Boolean> robotToIsStopInPast = new TreeMap<>();

            @Override
            public boolean isToRestore(int robotID) {
                return robotsToRestoreLater.contains(robotID);
            }

            @Override
            public boolean isToResume(int robotID) {
                return robotsToResumeLater.contains(robotID);
            }

            @Override
            public boolean updateForcing(double distanceTraveledAfterForcing) {
                assert ! isForcingFinished;

                double priorityDistanceRemaining = priorityDistance - distanceTraveledAfterForcing;
                if (isDistanceToCPAddedToPriorityDistance) {
                    assert distanceToCP != null;
                    priorityDistanceRemaining += distanceToCP;
                }
                priorityDistanceRemaining = Math.max(0, priorityDistanceRemaining);

                double stopDistanceRemaining = 0;
                stopDistanceRemaining = stopDistance - distanceTraveledAfterForcing;
                if (isDistanceToCPAddedToStopDistance) {
                    assert distanceToCP != null;
                    stopDistanceRemaining += distanceToCP;
                }
                stopDistanceRemaining = Math.max(0, stopDistanceRemaining);

                assert criticalSectionsToRestorePrioritiesLater.isEmpty() == robotsToRestoreLater.isEmpty();

                if (
                        isResetAfterCurrentCrossroad &&
                        priorityDistanceRemaining == 0 &&
                        stopDistanceRemaining == 0
                ) {
                    if (robotsToRestoreLater.isEmpty() && robotsToResumeLater.isEmpty()) {
                        return false;
                    }

                    if (criticalSectionsAtDone == null) {
                        criticalSectionsAtDone = new HashSet<>(criticalSectionsToRestorePrioritiesLater);
                        criticalSectionsAtDone.retainAll(tec.allCriticalSections);
                    } else if (areSomeCriticalSectionsWithHighPriorityGone(tec.allCriticalSections, criticalSectionsAtDone)) {
                        return false;
                    }
                }

                affectRobots(distanceTraveledAfterForcing, priorityDistanceRemaining, stopDistanceRemaining);
                return true;
            }

            private TreeMap<Integer, ArrayList<CriticalSection>> findRobotToCSesPriority(
                    double distanceTraveledAfterForcing,
                    double priorityDistanceRemaining
            ) {
                TreeMap<Integer, ArrayList<CriticalSection>> robotToCSesPriority = new TreeMap<>();
                if (priorityDistanceRemaining > 0) {
                    final ArrayList<CriticalSection> criticalSectionsForPriority =
                            selectCriticalSections(
                                    robotID,
                                    tec.allCriticalSections,
                                    Math.max(0, priorityDistanceMin - distanceTraveledAfterForcing),
                                    priorityDistanceRemaining,
                                    Integer.MAX_VALUE
                            );
                    for (CriticalSection cs : criticalSectionsForPriority) {
                        if (criticalSectionsToRestorePrioritiesLater.contains(cs)) {
                            continue;
                        }

                        int otherID = cs.getOtherRobotID(robotID);
                        if (! robotToCSesPriority.containsKey(otherID)) {
                            robotToCSesPriority.put(otherID, new ArrayList<>());
                        }
                        robotToCSesPriority.get(otherID).add(cs);
                    }
                }
                return robotToCSesPriority;
            }

            private TreeSet<Integer> findRobotsToStop(
                    double distanceTraveledAfterForcing,
                    double stopDistanceRemaining
            ) {
                TreeSet<Integer> robotsToStop = new TreeSet<>();
                if (isGlobalTemporaryStop) {
                    robotsToStop.addAll(VehiclesHashMap.getList().keySet());
                    robotsToStop.remove(robotID);
                } else if (stopDistanceRemaining > 0) {
                    final ArrayList<CriticalSection> criticalSectionsForStop =
                            selectCriticalSections(
                                    robotID,
                                    tec.allCriticalSections,
                                    Math.max(0, stopDistanceMin - distanceTraveledAfterForcing),
                                    stopDistanceRemaining,
                                    Integer.MAX_VALUE
                            );

                    for (CriticalSection cs : criticalSectionsForStop) {
                        int robotToStop = cs.getOtherRobotID(robotID);
                        assert robotToStop != robotID;

                        robotsToStop.add(robotToStop);
                    }
                }
                return robotsToStop;
            }

            private void affectRobots(
                    double distanceTraveledAfterForcing,
                    double priorityDistanceRemaining,
                    double stopDistanceRemaining
            ) {
                TreeMap<Integer, ArrayList<CriticalSection>> robotToCSesPriority = findRobotToCSesPriority(
                        distanceTraveledAfterForcing, priorityDistanceRemaining
                );
                TreeSet<Integer> robotsToStop = findRobotsToStop(distanceTraveledAfterForcing, stopDistanceRemaining);

                TreeMap<Integer, Boolean> robotToIsStop = new TreeMap<>();
                TreeSet<Integer> robotsToConsider = new TreeSet<>();
                robotsToConsider.addAll(robotToCSesPriority.keySet());
                robotsToConsider.addAll(robotsToStop);
                for (int robotID : robotsToConsider) {
                    Boolean isStop = null;
                    if (robotToIsStopInPast.containsKey(robotID)) {
                        isStop = robotToIsStopInPast.get(robotID);
                    } else if (! robotToCSesPriority.containsKey(robotID)) {
                        assert robotsToStop.contains(robotID);
                        isStop = true;
                    } else if (! robotsToStop.contains(robotID)) {
                        assert robotToCSesPriority.containsKey(robotID);
                        isStop = false;
                    }

                    if (isStop == null) {
                        isStop = rand.nextDouble() < probabilityStopNotChangeOfPriorities;
                    }
                    robotToIsStop.put(robotID, isStop);
                }

                for (Map.Entry<Integer, Boolean> entry : robotToIsStop.entrySet()) {
                    int otherID = entry.getKey();
                    boolean isStop = entry.getValue();


                    ArrayList<CriticalSection> cses = robotToCSesPriority.getOrDefault(otherID, new ArrayList<>());
                    for (CriticalSection cs : cses) {
                        increaseRobotPriorityOnCS(cs);
                    }
                    if (isStop) {
                        stopRobotIfNeeded(otherID);
                    }

                    if (robotToIsStopInPast.containsKey(otherID)) {
                        assert robotToIsStopInPast.get(otherID) == isStop;
                    } else {
                        robotToIsStopInPast.put(otherID, isStop);
                        double distanceToCS = computeDistanceToCS(robotID, otherID, cses);
                        new Event.ForcingReactionStarted(otherID, isStop, distanceToCS).write();
                    }
                }
            }

            private void increaseRobotPriorityOnCS(CriticalSection cs) {
                cs.setWeight(robotID, CriticalSection.Weight.WEIGHT_FORCING);

                int otherID = cs.getOtherRobotID(robotID);
                if (TrajectoryEnvelopeCoordinatorSimulation.isCPForcingHack) {
                    TrajectoryEnvelopeCoordinatorSimulation.tec.addStoppingPoint(
                            otherID, TrajectoryEnvelopeCoordinatorSimulation.CP_FORCING_HACK, -1
                    );
                }

                robotsToRestoreLater.add(otherID);

                assert ! criticalSectionsToRestorePrioritiesLater.contains(cs);
                criticalSectionsToRestorePrioritiesLater.add(cs);
            }

            private void stopRobotIfNeeded(int robotID) {
                if (robotsToResumeLater.contains(robotID)) {
                    return;
                }

                // Note: If the robot is not in `criticalSectionsToRestorePrioritiesLater`,
                // then it's after all critical sections with the human, so no priority change is needed.

                stopRobot(robotID);
                robotsToResumeLater.add(robotID);
            }

            public void restorePriorities() {
                for (CriticalSection cs : criticalSectionsToRestorePrioritiesLater) {
                    if (tec.allCriticalSections.contains(cs)) {
                        cs.setWeight(robotID, CriticalSection.Weight.WEIGHT_NORMAL);
                    }
                }
                if (TrajectoryEnvelopeCoordinatorSimulation.isCPForcingHack) {
                    for (int otherID : robotsToRestoreLater) {
                        TrajectoryEnvelopeCoordinatorSimulation.tec.removeStoppingPoint(otherID, TrajectoryEnvelopeCoordinatorSimulation.CP_FORCING_HACK);
                    }
                }
                forcingSinceTimestep = -1;
            }

            public void resumeRobots() {
                for (int robot : robotsToResumeLater) {
                    resumeRobot(robot);
                }
                forcingSinceTimestep = -1;
            }

            @Override
            public void finishForcing() {
                assert ! isForcingFinished;

                boolean isEmpty = robotsToRestoreLater.isEmpty() && robotsToResumeLater.isEmpty();
                if (isEmpty) {
                    robotIDToNumUselessForcingEvents.put(robotID, robotIDToNumUselessForcingEvents.getOrDefault(robotID, 0) + 1);
                }

                restorePriorities();
                resumeRobots();

                for (int otherID : robotToIsStopInPast.keySet()) {
                    new Event.ForcingReactionFinished(otherID).write();
                }
                new Event.ForcingFinished(robotID).write();

                isForcingFinished = true;
            }
        };
    }

    protected static ArrayList<CriticalSection> selectCriticalSections(
            int robotID,
            HashSet<CriticalSection> allCriticalSections,
            double minDistance,
            double maxDistance,
            int maxCount) {
        assert minDistance <= maxDistance;

        assert maxCount >= 0;
        if (maxCount == 0) {
            return new ArrayList<>();
        }

        if (maxDistance == Double.NEGATIVE_INFINITY) {
            return new ArrayList<>();
        }
        assert maxDistance >= 0.0;

        PoseSteering[] currentPath = HumanControl.getCurrentPath(robotID);
        int indexCurrent = HumanControl.getPathIndex(robotID);
        if (indexCurrent == -1) {
            return new ArrayList<>();
        }

        final ArrayList<CriticalSection> criticalSectionsSorted =
                CriticalSection.sortCriticalSectionsForRobotID(allCriticalSections, robotID);
        ArrayList<CriticalSection> criticalSectionsSelected = new ArrayList<>();

        double distanceUntilCS = 0.0;

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
                distanceUntilCS += step;
                indexCurrent += 1;

                if (distanceUntilCS > maxDistance) {
                    break;
                }
            }
            if (distanceUntilCS > maxDistance) {
                break;
            }

            if (distanceUntilCS < minDistance) {
                continue;
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
            Set<CriticalSection> criticalSectionsWithHighPriority) {
        for (CriticalSection cs : criticalSectionsWithHighPriority) {
            if (! allCriticalSections.contains(cs)) {
                return true;
            }
        }
        return false;
    }

    protected static void stopRobot(int robotID) {
        System.out.println(robotID);

        int counter = robotIDToFreezingCounter.getOrDefault(robotID, 0);
        assert 0 <= counter && counter < maxNumberOfHumans;
        robotIDToFreezingCounter.put(robotID, counter + 1);
        if (robotIDToFreezingCounter.get(robotID) == 1) {
            TrajectoryEnvelopeCoordinatorSimulation.tec.addStoppingPoint(robotID, TrajectoryEnvelopeCoordinatorSimulation.CP_ASAP, -1);
        }
    }

    public static void resumeRobot(int robotID) {
        System.out.println(robotID);

        int counter = robotIDToFreezingCounter.getOrDefault(robotID, 0);
        assert 0 < counter && counter <= maxNumberOfHumans;
        robotIDToFreezingCounter.put(robotID, counter - 1);
        if (robotIDToFreezingCounter.get(robotID) == 0) {
            TrajectoryEnvelopeCoordinatorSimulation.tec.removeStoppingPoint(robotID, TrajectoryEnvelopeCoordinatorSimulation.CP_ASAP);
        }
    }

    public static boolean isRobotFrozen(int robotID) {
        int counter = robotIDToFreezingCounter.getOrDefault(robotID, 0);
        assert 0 <= counter && counter <= maxNumberOfHumans;
        return counter > 0;
    }
}
