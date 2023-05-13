package se.oru.coordination.coordination_oru.simulator;

import com.vividsolutions.jts.geom.Polygon;
import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelope;
import se.oru.coordination.coordination_oru.utility.RobotReport;

public interface FleetVisualization {

    void displayRobotState(TrajectoryEnvelope te, RobotReport rr, String... extraStatusInfo);

    @Deprecated
    void displayRobotState(Polygon fp, RobotReport rr, String... extraStatusInfo);

    void displayDependency(RobotReport rrWaiting, RobotReport rrDriving, String dependencyDescriptor);

    void addEnvelope(TrajectoryEnvelope te);

    void removeEnvelope(TrajectoryEnvelope te);

    void updateVisualization();

    void setMap(String yamlFile);

    int periodicEnvelopeRefreshInMillis();

}
