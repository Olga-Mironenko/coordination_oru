package se.oru.coordination.coordination_oru;

import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelope;

public interface ForwardModel {

    boolean canStop(TrajectoryEnvelope te, RobotReport currentState, int targetPathIndex, boolean useVelocity);

    int getEarliestStoppingPathIndex(TrajectoryEnvelope te, RobotReport currentState);

}