package se.oru.coordination.coordination_oru.util;

import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.PoseSteering;

import se.oru.coordination.coordination_oru.Mission;
import se.oru.coordination.coordination_oru.RobotReport;
import se.oru.coordination.coordination_oru.code.VehiclesHashMap;
import se.oru.coordination.coordination_oru.simulation2D.TrajectoryEnvelopeCoordinatorSimulation;

public class MissionUtils {
    public static double targetVelocity1 = 5.0;
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

            PoseSteering[] path = null;
            try {
                path = vehicle.getPath(currentPose, goal, false);
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

            if (rr.getPathIndex() == -1) {
                Missions.enqueueMission(new Mission(robotID, path));
                lastUsedPath = path;
            } else {
                MissionUtils.changePath1(path);
            }
        }
    }

    public static void changeTargetVelocity1(double delta) {
        double targetVelocity1New = targetVelocity1 + delta;
        if (targetVelocity1New > 0) {
            targetVelocity1 = targetVelocity1New;
            synchronized (pathLock) {
                if (lastUsedPath != null) {
                    changePath1(lastUsedPath);
                }
            }
        }
    }

    protected static void changePath1(PoseSteering[] newPath) {
        TrajectoryEnvelopeCoordinatorSimulation tec = TrajectoryEnvelopeCoordinatorSimulation.tec;

        RobotReport rr = tec.getRobotReport(1);
        int replacementIndex = Math.max(0, rr.getPathIndex() - 10); // TODO: why does `- 10` work better than `+ 10`?

        PoseSteering[] replacementPath = computeReplacementPath(lastUsedPath, replacementIndex, newPath);
        tec.replacePath(1, replacementPath, replacementIndex, false, null);

        RobotReport rrNew = tec.getRobotReport(1);
        System.out.println("changePath1: pose changed: " + rr.getPose() + " -> " + rrNew.getPose());
        // TODO: It changes significantly sometimes.

        lastUsedPath = replacementPath;
    }

    public static PoseSteering[] computeReplacementPath(PoseSteering[] initialPath, int replacementIndex, PoseSteering[] newPath) {
        PoseSteering[] replacementPath = new PoseSteering[replacementIndex+newPath.length];
        for (int i = 0; i < replacementIndex; i++) replacementPath[i] = initialPath[i];
        for (int i = 0; i < newPath.length; i++) replacementPath[i+replacementIndex] = newPath[i];
        return replacementPath;
    }
}
