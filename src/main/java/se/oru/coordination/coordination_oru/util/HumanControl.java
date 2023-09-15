package se.oru.coordination.coordination_oru.util;

import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.PoseSteering;

import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelope;
import se.oru.coordination.coordination_oru.*;
import se.oru.coordination.coordination_oru.code.VehiclesHashMap;
import se.oru.coordination.coordination_oru.simulation2D.TrajectoryEnvelopeCoordinatorSimulation;

public class HumanControl {
    public static double targetVelocityHumanInitial = Double.POSITIVE_INFINITY; // essentially a limit
    public static double targetVelocityHuman = targetVelocityHumanInitial;
    public static int idHuman = 0;
    public static boolean isWorking = false;

    // TODO: race condition (click/keypress)
    // TODO: crashes on click and then (immediately) keypress
    // TODO: crashes on large initial velocity

    public static void moveRobot(int robotID, Pose goal) {
        if (isWorking) {
            return;
        }
        isWorking = true;
        try {
            TrajectoryEnvelopeCoordinatorSimulation tec = TrajectoryEnvelopeCoordinatorSimulation.tec;

            RobotReport rr = tec.getRobotReport(robotID);
            var vehicle = VehiclesHashMap.getVehicle(robotID);

            Pose currentPose = rr.getPose();
            try {
                vehicle.getPlan(currentPose, new Pose[]{goal}, Missions.getMapYAMLFilename(), false);
            }
            catch (NoPathFoundError error) {
                isWorking = false;
                return;
            }

            var newPath = vehicle.getPath();

            PoseSteering[] currentPath = getCurrentPath(robotID);
            if (currentPath == null || rr.getPathIndex() == -1) { // TODO: check for a dummy tracker too
                targetVelocityHuman = targetVelocityHumanInitial; // TODO: move the code to the dummy tracker or somewhere
                Missions.enqueueMission(new Mission(robotID, newPath));
            } else {
                int replacementIndex = getReplacementIndex(robotID);
                PoseSteering[] replacementPath = computeReplacementPath(currentPath, replacementIndex, newPath);
                HumanControl.changePath(robotID, replacementPath, replacementIndex);
            }
        } finally {
            isWorking = false;
        }
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

                PoseSteering[] currentPath = getCurrentPath(robotID);
                if (currentPath != null) {
                    changePath(robotID, currentPath, getReplacementIndex(robotID));
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
}