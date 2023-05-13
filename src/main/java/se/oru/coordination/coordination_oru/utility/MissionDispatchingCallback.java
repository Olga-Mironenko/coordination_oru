package se.oru.coordination.coordination_oru.utility;

public interface MissionDispatchingCallback {

    void beforeMissionDispatch(Mission m);

    void afterMissionDispatch(Mission m);

}
