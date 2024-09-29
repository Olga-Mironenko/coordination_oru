package se.oru.coordination.coordination_oru.motionplanning.ompl;

import com.sun.jna.Library;
import com.sun.jna.Pointer;
import com.sun.jna.Structure;
import com.sun.jna.ptr.IntByReference;
import com.sun.jna.ptr.PointerByReference;

import java.util.Arrays;
import java.util.List;

public interface CachingPlannerLib extends Library {
	boolean plan(
            byte[] occupancyMap, int width, int height,
            double mapResolution, double mapOriginX, double mapOriginY, double robotRadius,
            double[] xCoords, double[] yCoords, int numCoords,
            double startX, double startY, double startTheta,
            double goalX, double goalY, double goalTheta,
            PointerByReference path, IntByReference pathLength,
            double distanceBetweenPathPoints, double turningRadius,
            int numIterations, int algo);
	
	void cleanupPath(Pointer p);
	
	class PathPose extends Structure {
		public static class ByReference extends PathPose implements Structure.ByReference {}

		public double x;
		public double y;
		public double theta;
		
		public PathPose() {}
		public PathPose(Pointer p) {
			super(p);
		}
		@Override
		protected List<String> getFieldOrder() {
			return Arrays.asList(new String[] {"x", "y", "theta"});
		}
	}
}
