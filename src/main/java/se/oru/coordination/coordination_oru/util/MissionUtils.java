package se.oru.coordination.coordination_oru.util;

import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.PoseSteering;

import se.oru.coordination.coordination_oru.Mission;
import se.oru.coordination.coordination_oru.RobotReport;
import se.oru.coordination.coordination_oru.code.VehiclesHashMap;
import se.oru.coordination.coordination_oru.simulation2D.TrajectoryEnvelopeCoordinatorSimulation;

public class MissionUtils {
    public static double targetVelocity1 = 0.1;
    // TODO: stops when it's 0.1 again
    // TODO: reset velocity (to ~0) when mission finishes
    // TODO: race condition (click/keypress)
    // TODO: crashes on click and then (immediately) keypress
    // TODO: crashes on large initial velocity

    protected static PoseSteering[] lastUsedPath = null; // added as a new mission or as a replacement
    protected static Object pathLock = new Object(); // sentinel

    protected static void removeMissions(int robotID) {
        while (true) {
            var mission = Missions.dequeueMission(robotID);
            if (mission == null) {
                break;
            }
        }
    }

    public static void moveRobot(int robotID, Pose goal) {
        synchronized (pathLock) {
            TrajectoryEnvelopeCoordinatorSimulation tec = TrajectoryEnvelopeCoordinatorSimulation.tec;

            RobotReport rr = tec.getRobotReport(robotID);
            Pose currentPose = rr.getPose();
            var vehicle = VehiclesHashMap.getVehicle(robotID);

            PoseSteering[] newPath = null;
            try {
                newPath = vehicle.getPath(currentPose, goal, false);
            } catch (NoPathFound exc) {
                System.out.println("moveRobot: no path found");
                return;
            }

            // If there is a scheduled mission, wait until it starts.
            // TODO: Fix the hack.
            try {
                while (! tec.isMissionsPoolEmpty() || Missions.hasMissions(1))
                    Thread.sleep(50);
            }
            catch (InterruptedException e) {
                e.printStackTrace();
            }

            if (lastUsedPath == null || rr.getPathIndex() == -1) {
                Missions.enqueueMission(new Mission(robotID, newPath));
            } else {
                int replacementIndex = getReplacementIndex(robotID);
                PoseSteering[] replacementPath = computeReplacementPath(lastUsedPath, replacementIndex, newPath);
                MissionUtils.changePath(robotID, replacementPath, replacementIndex);
            }

            lastUsedPath = newPath;
        }
    }

    public static void changeTargetVelocity1(double delta) {
        double targetVelocity1New = targetVelocity1 + delta;
        if (targetVelocity1New > 0) {
            targetVelocity1 = targetVelocity1New;

            synchronized (pathLock) {
                if (lastUsedPath != null) {
                    int replacementIndex = getReplacementIndex(1);

                    // TODO: Instead of `lastUsedPath`, get the actual TE (because `replacementIndex` is the index
                    // of the latter).
                    changePath(1, lastUsedPath, replacementIndex);
                }
            }
        }
    }

    protected static int getReplacementIndex(int robotID) {
        TrajectoryEnvelopeCoordinatorSimulation tec = TrajectoryEnvelopeCoordinatorSimulation.tec;
        RobotReport rr = tec.getRobotReport(robotID);
        return Math.max(0, rr.getPathIndex() - 10);
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
        PoseSteering[] replacementPath = new PoseSteering[replacementIndex+newPath.length];
        for (int i = 0; i < replacementIndex; i++) replacementPath[i] = initialPath[i];
        for (int i = 0; i < newPath.length; i++) replacementPath[i+replacementIndex] = newPath[i];
        return replacementPath;
    }
}
