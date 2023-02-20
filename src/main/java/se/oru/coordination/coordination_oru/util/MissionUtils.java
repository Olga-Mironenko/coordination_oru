package se.oru.coordination.coordination_oru.util;

import java.util.Arrays;

import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.PoseSteering;

import se.oru.coordination.coordination_oru.Mission;
import se.oru.coordination.coordination_oru.RobotReport;
import se.oru.coordination.coordination_oru.code.VehiclesHashMap;
import se.oru.coordination.coordination_oru.simulation2D.TrajectoryEnvelopeCoordinatorSimulation;

public class MissionUtils {
    public static double targetVelocity1 = 5.0;
    public static PoseSteering[] lastChunk = null;

    public static void removeMissions(int robotID) {
        while (true) {
            var mission = Missions.dequeueMission(robotID);
            if (mission == null) {
                break;
            }
        }
    }

    public static void moveRobot(int robotID, Pose goal) {
        removeMissions(robotID);

        Pose currentOrig = TrajectoryEnvelopeCoordinatorSimulation.tec.getRobotReport(robotID).getPose();
        var vehicle = VehiclesHashMap.getVehicle(robotID);
        PoseSteering[] path = vehicle.getPath(currentOrig, goal, true);

        // nChunks=2, path=[p0, p1, p2]:
        // chunkSize: 1
        // chunk1: path[0:2] = [p0, p1]
        // chunk2: path[1:3] = [p1, p2]
        int nChunks = 1;
        int chunkSize = Math.max(0, path.length / nChunks - 1); // make the last chunk always greater than others
        for (int i = 0; i < nChunks; i++) {
            // length=43 (0..42), nChunks=8: chunkSize=5
            // i=0: start=0, finish=5
            // i=1: start=5, finish=10
            // i=2: start=10, finish=15
            // ...
            // i=7: start=35, finish=42
            int start = chunkSize * i;
            int finish = i < nChunks - 1 ? chunkSize * (i + 1) : path.length - 1;
            PoseSteering[] chunk = Arrays.copyOfRange(path, start, finish + 1);

            boolean isFirst = lastChunk == null;
            lastChunk = chunk;
            if (! isFirst) {
                MissionUtils.changeTargetVelocity1(0);
            }
            Missions.enqueueMission(new Mission(robotID, chunk)); // TODO: cancel slowDown for all but the last chunk
        }
    }

    public static void changeTargetVelocity1(double delta) {
        double targetVelocity1New = targetVelocity1 + delta;
        if (targetVelocity1New > 0) {
            targetVelocity1 = targetVelocity1New;

            TrajectoryEnvelopeCoordinatorSimulation tec = TrajectoryEnvelopeCoordinatorSimulation.tec;
            //tec.replanEnvelope(1); // doesn't work

            RobotReport rr = TrajectoryEnvelopeCoordinatorSimulation.tec.getRobotReport(1);
            tec.replacePath(1, lastChunk, rr.getPathIndex() + 20, false, null);
        }
    }

}
