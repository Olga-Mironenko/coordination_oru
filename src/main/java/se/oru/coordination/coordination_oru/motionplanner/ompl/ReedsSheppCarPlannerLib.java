package se.oru.coordination.coordination_oru.motionplanner.ompl;

import com.sun.jna.Library;
import com.sun.jna.Pointer;
import com.sun.jna.Structure;
import com.sun.jna.ptr.IntByReference;
import com.sun.jna.ptr.PointerByReference;

import java.util.Arrays;
import java.util.List;

public interface ReedsSheppCarPlannerLib extends Library {

    boolean plan(String mapFilename, double mapResolution, double mapOriginX, double mapOriginY, double robotRadius, double startX, double startY, double startTheta, double goalX, double goalY, double goalTheta, PointerByReference path, IntByReference pathLength, double distanceBetweenPathPoints, double turningRadius, double planningTimeInSecs);

    //public boolean plan_multiple_circles(String mapFilename, double occupiedThreshold, double mapResolution, double robotRadius, double[] xCoords, double[] yCoords, int numCoords, double startX, double startY, double startTheta, double goalX, double goalY, double goalTheta, PointerByReference path, IntByReference pathLength, double distanceBetweenPathPoints, double turningRadius, double planningTimeInSecs);
    //char* occupancyMap, int mapWidth, int mapHeight, double occupiedThreshold, double mapResolution, double robotRadius, double* xCoords, double* yCoords, int numCoords, double startX, double startY, double startTheta, double goalX, double goalY, double goalTheta, PathPose** path, int* pathLength, double distanceBetweenPathPoints, double turningRadius, double planningTimeInSecs
    boolean plan_multiple_circles(byte[] occupancyMap, int width, int height, double mapResolution, double mapOriginX, double mapOriginY, double robotRadius, double[] xCoords, double[] yCoords, int numCoords, double startX, double startY, double startTheta, double goalX, double goalY, double goalTheta, PointerByReference path, IntByReference pathLength, double distanceBetweenPathPoints, double turningRadius, double planningTimeInSecs, int algo);

    boolean plan_multiple_circles_nomap(double[] xCoords, double[] yCoords, int numCoords, double startX, double startY, double startTheta, double goalX, double goalY, double goalTheta, PointerByReference path, IntByReference pathLength, double distanceBetweenPathPoints, double turningRadius, double planningTimeInSecs, int algo);

    void cleanupPath(Pointer p);

    class PathPose extends Structure {
        public double x;
        public double y;
        public double theta;

        public PathPose() {
        }

        public PathPose(Pointer p) {
            super(p);
        }

        @Override
        protected List<String> getFieldOrder() {
            return Arrays.asList("x", "y", "theta");
        }

        public static class ByReference extends PathPose implements Structure.ByReference {
        }
    }
}
