package se.oru.coordination.coordination_oru.util;

import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.PoseSteering;

import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelope;
import se.oru.coordination.coordination_oru.*;
import se.oru.coordination.coordination_oru.code.VehiclesHashMap;
import se.oru.coordination.coordination_oru.simulation2D.TrajectoryEnvelopeCoordinatorSimulation;
import se.oru.coordination.coordination_oru.util.gates.Timekeeper;

public class HumanControl {
    public static boolean isEnabledForBrowser = false;
    public static double targetVelocityHumanInitial = Double.POSITIVE_INFINITY; // essentially a limit
    public static double targetVelocityHuman = targetVelocityHumanInitial;
    public static int idHuman = 0;
    public static boolean isWorking = false;

    public static String status = null;

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
            // currentPose = (105, 200)
            // [(90, 200), (100, 200), (110, 200), (120, 200), (130, 200)]
            //                  ^                                old goal

            String statusPrefix = String.format("Human controlled path for %s -> %s", currentPose, goal);

            try {
                status = String.format("%s: finding...", statusPrefix);
                vehicle.getPlan(currentPose, new Pose[]{goal}, Missions.getMapYAMLFilename(), false);
            }
            catch (NoPathFoundError error) {
                status = String.format("%s: failed to find", statusPrefix);
                isWorking = false;
                return;
            }

            assert tec.getRobotReport(robotID).toString().equals(rr.toString());

            var newPath = vehicle.getPath();
            status = String.format("%s: found a path with %d poses", statusPrefix, newPath.length);
            // [(105, 200), (115, 300), (120, 400), (130, 400)]
            //                                       (new) goal

            PoseSteering[] currentPath = getCurrentPath(robotID);
            if (currentPath == null || rr.getPathIndex() == -1) {
                targetVelocityHuman = targetVelocityHumanInitial; // TODO: move the code to the dummy tracker or somewhere
                Missions.enqueueMission(new Mission(robotID, newPath));
            } else {
                int replacementIndex = getReplacementIndex(robotID);
                PoseSteering[] replacementPath = computeReplacementPath(currentPath, replacementIndex, newPath);
                // [(90, 200), (100, 200), (105, 200), (115, 300), (120, 400), (130, 400)]
                //                  ^     |                                     (new) goal
                //           replacementIndex=1
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
        return getPathIndex(robotID);
    }

    protected static void changePath(int robotID, PoseSteering[] replacementPath, int replacementIndex) {
        TrajectoryEnvelopeCoordinatorSimulation tec = TrajectoryEnvelopeCoordinatorSimulation.tec;

        tec.replacePath(robotID, replacementPath, replacementIndex, false, null);

        int replacementIndexNew = getReplacementIndex(robotID);
        System.out.println("changePath1: replacementIndex: " + replacementIndex + " -> " + replacementIndexNew);
        // TODO: It changes significantly sometimes.
    }

    public static PoseSteering[] computeReplacementPath(PoseSteering[] oldPath, int replacementIndex, PoseSteering[] newPath) {
        // [(90, 200), (100, 200), (105, 200), (115, 300), (120, 400), (130, 400)]
        //                  ^     |                                     (new) goal
        //           replacementIndex=1
        int numOld = replacementIndex + 1;

        PoseSteering[] replacementPath = new PoseSteering[numOld + newPath.length];

        // replacementPath = oldPath[:replacementIndex + 1]
        System.arraycopy(oldPath, 0, replacementPath, 0, numOld);

        // replacementPath += newPath
        System.arraycopy(newPath, 0, replacementPath, numOld, newPath.length);

        return replacementPath;
    }
}