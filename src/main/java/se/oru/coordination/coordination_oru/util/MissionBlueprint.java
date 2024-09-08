package se.oru.coordination.coordination_oru.util;

import org.metacsp.multi.spatioTemporal.paths.Pose;
import se.oru.coordination.coordination_oru.code.AutonomousVehicle;

public class MissionBlueprint {
    public enum Direction {
        FORWARD_ONLY,
        FORWARD_BACKWARD_SINGLE_MISSION,
        FORWARD_BACKWARD_SEPARATE_MISSIONS,
    }

    public AutonomousVehicle vehicle;
    public Pose start;
    public Pose middle = null;
    public Pose finish;

    public Direction direction = Direction.FORWARD_ONLY;
    public boolean isToCleanForward = false;

    public MissionBlueprint(AutonomousVehicle vehicle, Pose start, Pose finish) {
        this.vehicle = vehicle;
        this.start = start;
        this.finish = finish;
    }

    public MissionBlueprint setMiddle(Pose middle) {
        this.middle = middle;
        return this;
    }

    public MissionBlueprint setDirection(Direction direction) {
        this.direction = direction;
        return this;
    }

    public MissionBlueprint setIsToCleanForward(boolean isToCleanForward) {
        this.isToCleanForward = isToCleanForward;
        return this;
    }
}
