package se.oru.coordination.coordination_oru.util;

import org.apache.commons.collections.comparators.ComparatorChain;
import org.metacsp.multi.spatioTemporal.paths.PoseSteering;
import se.oru.coordination.coordination_oru.CriticalSection;
import se.oru.coordination.coordination_oru.TrackingCallback;
import se.oru.coordination.coordination_oru.code.Heuristics;
import se.oru.coordination.coordination_oru.code.VehiclesHashMap;
import se.oru.coordination.coordination_oru.simulation2D.TrajectoryEnvelopeCoordinatorSimulation;

import java.util.*;

public class Forcing {
    public static HashMap<Integer, Integer> robotIDToFreezingCounter = new HashMap<>(); // TODO: use semaphores
    public static HashMap<Integer, Integer> robotIDToPathIndexToStop = new HashMap<>();
    public static TreeMap<Integer, Integer> robotIDToNumForcingEvents = new TreeMap<>();

    // Distances between the current robot and intersections to check:
    public static double priorityDistance = Double.NEGATIVE_INFINITY; // change the priority of that intersection?
    public static double stopDistance = Double.NEGATIVE_INFINITY; // stop the other robot of that intersection?

    public static boolean isGlobalTemporaryStop = false;
    public static boolean isResetAfterCurrentCrossroad = true;

    private final static int maxNumberOfHumans = 1;

    public static KnobsAfterForcing forceDriving(int robotID) {
        robotIDToNumForcingEvents.put(robotID, robotIDToNumForcingEvents.getOrDefault(robotID, 0) + 1);

        final TrajectoryEnvelopeCoordinatorSimulation tec = TrajectoryEnvelopeCoordinatorSimulation.tec;

        HashSet<CriticalSection> criticalSectionsToRestorePrioritiesLater = new HashSet<>();
        TreeSet<Integer> robotsToResumeLater = new TreeSet<>();

        KnobsAfterForcing knobsAfterForcing = new KnobsAfterForcing() {
            @Override
            public boolean updateForcing(double distanceTraveled) {
                if (isResetAfterCurrentCrossroad && areSomeCriticalSectionsWithHighPriorityGone(tec.allCriticalSections, criticalSectionsToRestorePrioritiesLater)) {
                    restorePriorities();
                    resumeRobots();
                    return false;
                }

                double priorityDistanceRemaining = Math.max(0, priorityDistance - distanceTraveled);
                if (priorityDistanceRemaining > 0) {
                    final ArrayList<CriticalSection> criticalSectionsForPriority =
                            selectCriticalSections(robotID, tec.allCriticalSections, priorityDistanceRemaining, Integer.MAX_VALUE);
                    for (CriticalSection cs : criticalSectionsForPriority) {
                        if (criticalSectionsToRestorePrioritiesLater.contains(cs)) {
                            continue;
                        }
                        cs.setHigher(robotID, 1);
                        criticalSectionsToRestorePrioritiesLater.add(cs);
                    }
                }

                TreeSet<Integer> robotsToStop = new TreeSet<>();
                if (isGlobalTemporaryStop) {
                    robotsToStop.addAll(VehiclesHashMap.getList().keySet());
                    robotsToStop.remove(robotID);
                } else {
                    double stopDistanceRemaining = Math.max(0, stopDistance - distanceTraveled);
                    if (stopDistanceRemaining > 0) {
                        final ArrayList<CriticalSection> criticalSectionsForStop =
                                selectCriticalSections(robotID, tec.allCriticalSections, stopDistanceRemaining, Integer.MAX_VALUE);

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
                }
                for (int robot : robotsToStop) {
                    if (robotsToResumeLater.contains(robot)) {
                        continue;
                    }

                    boolean willBeRestored = false;
                    for (CriticalSection cs : criticalSectionsToRestorePrioritiesLater) {
                        if (cs.getInferior() == robot) {
                            willBeRestored = true;
                            break;
                        }
                    }
                    assert willBeRestored; // otherwise, `stopDistance` is not consistent with `priorityDistance`

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
            }

            @Override
            public void restorePriorities() {
                for (CriticalSection cs : criticalSectionsToRestorePrioritiesLater) {
                    if (tec.allCriticalSections.contains(cs)) {
                        cs.setHigher(robotID, 0);
                    }
                }
                criticalSectionsToRestorePrioritiesLater.clear();
            }
        };

        return knobsAfterForcing;
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

        PoseSteering[] currentPath = HumanControl.getCurrentPath(robotID);
        int indexCurrent = HumanControl.getPathIndex(robotID);
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
}
