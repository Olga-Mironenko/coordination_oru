package se.oru.coordination.coordination_oru.util;

import java.util.Random;
import se.oru.coordination.coordination_oru.util.gates.GatedCalendar;

public class DeterministicRandom extends Random {
    public DeterministicRandom(String className, int robotID) {
        // Compute a seed by XOR-ing the current time with the hash code of the class name.
        // Note: the cast to long ensures that the int hashCode() is properly promoted.
        super(GatedCalendar.getInstance().getTimeInMillis() ^ (long) className.hashCode() ^ (long) robotID);
    }
}
