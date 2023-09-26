package se.oru.coordination.coordination_oru.tests.util;

import org.metacsp.multi.spatioTemporal.paths.Pose;

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

    public final static Pose column1Top = new Pose(xColumn1, yTop, thetaDown);
    public final static Pose column2Top = new Pose(xColumn2, yTop, thetaDown);
    public final static Pose column3Top = new Pose(xColumn3, yTop, thetaDown);

    public final static Pose column1Bottom = new Pose(xColumn1, yBottom, thetaUp);
    public final static Pose column2Bottom = new Pose(xColumn2, yBottom, thetaLeft); // left is for the robot 0 to look down
    public final static Pose column3Bottom = new Pose(xColumn3, yBottom, thetaUp);

    public final static Pose row1Left = new Pose(xLeft, yRow1, thetaRight);
    public final static Pose row2Left = new Pose(xLeft, yRow2, thetaRight);
    public final static Pose row3Left = new Pose(xLeft, yRow3, thetaRight);

    public final static Pose row1Right = new Pose(xRight, yRow1, thetaLeft);
    public final static Pose row2Right = new Pose(xRight, yRow2, thetaLeft);
    public final static Pose row3Right = new Pose(xRight, yRow3, thetaLeft);

    public final static Pose centerDownward = new Pose(xColumn2, yRow2, thetaDown);
}
