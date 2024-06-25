package se.oru.coordination.coordination_oru.tests.util;

import org.metacsp.multi.spatioTemporal.paths.Pose;

import static se.oru.coordination.coordination_oru.motionplanning.AbstractMotionPlanner.inverseYaw;

public class GridMapConstants extends BaseMapConstants {
    public final static double xLeft = 4.0;
    public final static double xRight = 56.0;
    public final static double yTop = 56.0;
    public final static double yBottom = 4.0;

    public final static double xColumn1 = 14.5;
    public final static double xColumn2 = 30.0;
    public final static double xColumn3 = 45.5;
    public final static double yRow1 = 44.0;
    public final static double yRow2 = 30.0;
    public final static double yRow3 = 16.0;

    /*
    x:
       4   14.5   30   45.5   56
       <     +     +     +     >
        10.5   15.5  15.5  10.5

    y:
        56  ^
               12
        44  +
               14
        30  +
               14
        16  +
               |
        8   F  |12
               |
        4   V
     */

    public final static Pose column1TopStart = new Pose(xColumn1, yTop, thetaDown);
    public final static Pose column2TopStart = new Pose(xColumn2, yTop, thetaDown);
    public final static Pose column3TopStart = new Pose(xColumn3, yTop, thetaDown);

    public final static Pose column1BottomStart = new Pose(xColumn1, yBottom, thetaUp);
    public final static Pose column2BottomStart = new Pose(xColumn2, yBottom, thetaUp);
    public final static Pose column3BottomStart = new Pose(xColumn3, yBottom, thetaUp);

    public final static Pose column2Row1Down = new Pose(xColumn2, yRow1, thetaDown);
    public final static Pose column2Row2Down = new Pose(xColumn2, yRow2, thetaDown);
    public final static Pose column2Row3Down = new Pose(xColumn2, yRow3, thetaDown);

    public final static Pose column3Row1Down = new Pose(xColumn3, yRow1, thetaDown);
    public final static Pose column3Row2Down = new Pose(xColumn3, yRow2, thetaDown);
    public final static Pose column3Row3Down = new Pose(xColumn3, yRow3, thetaDown);

    public final static Pose row1LeftStart = new Pose(xLeft, yRow1, thetaRight);
    public final static Pose row2LeftStart = new Pose(xLeft, yRow2, thetaRight);
    public final static Pose row3LeftStart = new Pose(xLeft, yRow3, thetaRight);

    public final static Pose row1RightStart = new Pose(xRight, yRow1, thetaLeft);
    public final static Pose row2RightStart = new Pose(xRight, yRow2, thetaLeft);
    public final static Pose row3RightStart = new Pose(xRight, yRow3, thetaLeft);

    public final static Pose centerDown = new Pose(xColumn2, yRow2, thetaDown);

    public static Pose turnAround(Pose pose) {
        return new Pose(pose.getX(), pose.getY(), inverseYaw(pose.getTheta()));
    }
}
