package se.oru.coordination.coordination_oru.util;

/**
 * After forcing starts, an object of this class is returned to affect the process of forcing.
 * Namely, to do it continuously and to stop it.
 */
public abstract class KnobsAfterForcing {
    public abstract boolean updateForcing(double distanceTraveled);

    public abstract void resumeRobots();

    public abstract void restorePriorities();
}
