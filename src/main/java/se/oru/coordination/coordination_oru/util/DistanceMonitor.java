package se.oru.coordination.coordination_oru.util;

public class DistanceMonitor {
    double lengthPreviousIntervals = 0.0;

    public boolean update(double lengthIntervalLimit, double distance) {
        double lengthInterval = distance - lengthPreviousIntervals;
        boolean isNewInterval = false;
        while (lengthInterval >= lengthIntervalLimit) {
            isNewInterval = true;
            lengthInterval -= lengthIntervalLimit;
            lengthPreviousIntervals += lengthIntervalLimit;
        }
        return isNewInterval;
    }
}
