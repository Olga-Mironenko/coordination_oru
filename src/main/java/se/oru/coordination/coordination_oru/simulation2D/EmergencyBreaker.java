package se.oru.coordination.coordination_oru.simulation2D;

import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelope;
import se.oru.coordination.coordination_oru.CriticalSection;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashSet;
import java.util.TreeSet;
import java.util.stream.Collectors;

public class EmergencyBreaker {
    protected boolean useGlobal;
    protected boolean useLocal;
    protected boolean isGlobal = false;
    protected TreeSet<Integer> localIds = new TreeSet<>();

    public EmergencyBreaker(boolean useGlobal, boolean useLocal) {
        this.useGlobal = useGlobal;
        this.useLocal = useLocal;
    }

    public void addLocalid(int id) {
        localIds.add(id);
    }

    public String toString() {
        ArrayList<String> texts = new ArrayList<>();
        if (useGlobal) {
            String text = "global";
            if (isGlobal) {
                text += " (active)";
            }
            texts.add(text);
        }
        if (useLocal) {
            String text = "local";
            if (! localIds.isEmpty()) {
                text += " (active for " + stringifyLocalIds() + ")";
            }
            texts.add(text);
        }
        return texts.isEmpty() ? null : String.join(" ", texts);
    }

    protected String stringifyLocalIds() {
        return localIds.stream().map(String::valueOf).collect(Collectors.joining(", "));
    }

    public boolean isStopped(int id) {
        return isGlobal || localIds.contains(id);
    }

    public void stopRobots(int id, int pathIndex) {
        if (useGlobal) {
            assert ! isGlobal;
            isGlobal = true;
        }

        if (useLocal) {
            localIds.add(id);

            HashSet<Integer> idsOpponents = findOpponentsInCriticalSections(id, pathIndex);
            localIds.addAll(idsOpponents);
        }
    }

    protected HashSet<Integer> findOpponentsInCriticalSections(int myRobotID, int pathIndex) {
        HashSet<Integer> idsOpponents = new HashSet<>();
        for (CriticalSection cs : TrajectoryEnvelopeCoordinatorSimulation.tec.allCriticalSections) {
            int myStart = -1;
            int myEnd = -1;
            TrajectoryEnvelope opponentsEnvelope = null;
            if (cs.getTe1() != null && cs.getTe1().getRobotID() == myRobotID) {
                myStart = cs.getTe1Start();
                myEnd = cs.getTe1End();
                opponentsEnvelope = cs.getTe2();
            } else if (cs.getTe2() != null && cs.getTe2().getRobotID() == myRobotID)  {
                myStart = cs.getTe2Start();
                myEnd = cs.getTe2End();
                opponentsEnvelope = cs.getTe1();
            }

            if (opponentsEnvelope != null) {
                assert myStart != -1;
                assert myEnd != -1;

                if (myStart <= pathIndex && pathIndex <= myEnd) {
                    idsOpponents.add(opponentsEnvelope.getRobotID());
                }
            }
        }
        return idsOpponents;
    }
}
