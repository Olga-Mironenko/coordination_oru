package se.oru.coordination.coordination_oru.util;

import org.metacsp.multi.spatioTemporal.paths.PoseSteering;
import se.oru.coordination.coordination_oru.CriticalSection;
import se.oru.coordination.coordination_oru.RobotReport;
import se.oru.coordination.coordination_oru.code.AbstractVehicle;
import se.oru.coordination.coordination_oru.code.VehiclesHashMap;
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
    // - If `isGlobalTemporaryStop` is true, then all other robots are stopped during forcing.
    public static boolean isGlobalTemporaryStop = false;
    // - If `isResetAfterCurrentCrossroad` is true, then forcing finishes automatically when its priority and stop
    //   effects end. If the flag is false, then it's needed to finish forcing manually
    public static boolean isResetAfterCurrentCrossroad = true;

    private final static int maxNumberOfHumans = 1;

    public static int forcingSinceTimestep = -1;

    public static void startForcing(int robotID) {
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

    public static KnobsAfterForcing forceDriving(int robotID) {
        if (forcingSinceTimestep != -1) { // another forcing is already in progress
            return null;
        }
        forcingSinceTimestep = Timekeeper.getTimestepsPassed();
        robotIDToNumForcingEvents.put(robotID, robotIDToNumForcingEvents.getOrDefault(robotID, 0) + 1);

        final TrajectoryEnvelopeCoordinatorSimulation tec = TrajectoryEnvelopeCoordinatorSimulation.tec;

        HashSet<CriticalSection> criticalSectionsToRestorePrioritiesLater = new HashSet<>();
        TreeSet<Integer> robotsToResumeLater = new TreeSet<>();

        KnobsAfterForcing knobsAfterForcing = new KnobsAfterForcing() {
            @Override
            public boolean updateForcing(double distanceTraveledAfterForcing) {
                double priorityDistanceRemaining = priorityDistance - distanceTraveledAfterForcing;
                if (isDistanceToCPAddedToPriorityDistance) {
                    assert distanceToCP != null;
                    priorityDistanceRemaining += distanceToCP;
                }
                priorityDistanceRemaining = Math.max(0, priorityDistanceRemaining);

                double stopDistanceRemaining = stopDistance - distanceTraveledAfterForcing;
                if (isDistanceToCPAddedToStopDistance) {
                    assert distanceToCP != null;
                    stopDistanceRemaining += distanceToCP;
                }
                stopDistanceRemaining = Math.max(0, stopDistanceRemaining);

                boolean isDone = priorityDistanceRemaining == 0 && stopDistanceRemaining == 0 && ! isGlobalTemporaryStop;

                if (isResetAfterCurrentCrossroad && isDone) {
                    boolean isEmpty = criticalSectionsToRestorePrioritiesLater.isEmpty() && robotsToResumeLater.isEmpty();
                    if (isEmpty || areSomeCriticalSectionsWithHighPriorityGone(tec.allCriticalSections, criticalSectionsToRestorePrioritiesLater)) {
                        if (isEmpty) {
                            robotIDToNumUselessForcingEvents.put(robotID, robotIDToNumUselessForcingEvents.getOrDefault(robotID, 0) + 1);
                        }
                        restorePriorities();
                        resumeRobots();
                        return false;
                    }
                }

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

                        cs.setHigher(robotID, 3);
                        criticalSectionsToRestorePrioritiesLater.add(cs);
                    }
                }

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
                        int robotToStop;
                        if (cs.isTe1(robotID)) {
                            robotToStop = cs.getTe2RobotID();
                        } else if (cs.isTe2(robotID)) {
                            robotToStop = cs.getTe1RobotID();
                        } else {
                            throw new RuntimeException();
                        }
                        assert robotToStop != robotID;

                        if (!cs.isRobotOnCS(robotToStop)) {
                            robotsToStop.add(robotToStop);
                        }
                    }
                }
                for (int robot : robotsToStop) {
                    if (robotsToResumeLater.contains(robot)) {
                        continue;
                    }

                    // Note: If the robot is not in `criticalSectionsToRestorePrioritiesLater`,
                    // then it's after all critical sections with the human, so no priority change is needed.

                    stopRobot(robot);
                    robotsToResumeLater.add(robot);
                }

                return true;
            }

            @Override
            public void resumeRobots() {
                for (int robot : robotsToResumeLater) {
                    resumeRobot(robot);
                }
                robotsToResumeLater.clear();
                forcingSinceTimestep = -1;
            }

            @Override
            public void restorePriorities() {
                for (CriticalSection cs : criticalSectionsToRestorePrioritiesLater) {
                    if (tec.allCriticalSections.contains(cs)) {
                        cs.setHigher(robotID, 0);
                    }
                }
                criticalSectionsToRestorePrioritiesLater.clear();
                forcingSinceTimestep = -1;
            }
        };

        return knobsAfterForcing;
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
    }

    public static void resumeRobot(int robotID) {
        System.out.println(robotID);

        int counter = robotIDToFreezingCounter.getOrDefault(robotID, 0);
        assert 0 < counter && counter <= maxNumberOfHumans;
        robotIDToFreezingCounter.put(robotID, counter - 1);
    }

    public static boolean isRobotFrozen(int robotID) {
        int counter = robotIDToFreezingCounter.getOrDefault(robotID, 0);
        assert 0 <= counter && counter <= maxNumberOfHumans;
        return counter > 0;
    }
}
