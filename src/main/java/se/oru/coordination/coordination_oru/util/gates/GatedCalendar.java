package se.oru.coordination.coordination_oru.util.gates;

import java.util.Calendar;

public class GatedCalendar {
    public static GatedCalendar getInstance() {
        return new GatedCalendar();
    }

    public long getTimeInMillis() {
        if (! GatedThread.isGated) {
            return Calendar.getInstance().getTimeInMillis();
        }

        return Timekeeper.getVirtualMillisPassed();
    }
}
