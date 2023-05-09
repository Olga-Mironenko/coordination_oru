package se.oru.coordination.coordination_oru;

public class RobotAtCriticalSection {

    private final RobotReport rr;
    private final CriticalSection cs;

    public RobotAtCriticalSection(RobotReport rr, CriticalSection cs) {
        this.rr = rr;
        this.cs = cs;
    }

    public RobotReport getRobotReport() {
        return this.rr;
    }

    public CriticalSection getCriticalSection() {
        return this.cs;
    }

}
