package se.oru.coordination.coordination_oru.gui_oru;

import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.PoseSteering;

import com.vividsolutions.jts.geom.Coordinate;

import se.oru.coordination.coordination_oru.ConstantAccelerationForwardModel;
import se.oru.coordination.coordination_oru.Mission;
import se.oru.coordination.coordination_oru.motionplanning.ompl.ReedsSheppCarPlanner;
import se.oru.coordination.coordination_oru.simulation2D.TrajectoryEnvelopeCoordinatorSimulation;
import se.oru.coordination.coordination_oru.util.Missions;

public class MyRobot {
    public double MAX_ACCEL;
    public double MAX_VEL;

    public Coordinate[] footprint;

    static public int countOfRobotID = 1;
    public int robotID;

    public ReedsSheppCarPlanner rsp;

    public int numWay=1;
    
    public static void initMyRobot()
    {
        countOfRobotID = 1;
        //Mission.initMission(); //هنا تم التعديل
    }
    
    public int countMission = 0;
    
    MyRobot()
    {
        rsp = new ReedsSheppCarPlanner();

        robotID = countOfRobotID;
        countOfRobotID++;

    }

    void setSpeed(TrajectoryEnvelopeCoordinatorSimulation tec,double MAX_VELOCITY,double MAX_ACCELERATION)
    {
        MAX_ACCEL=MAX_ACCELERATION;
        MAX_VEL = MAX_VELOCITY;
        tec.setRobotMaxAcceleration(robotID, MAX_VEL);
		tec.setRobotMaxVelocity(robotID, MAX_VEL);
        tec.setForwardModel(robotID, new ConstantAccelerationForwardModel(MAX_ACCEL, MAX_VEL, tec.getTemporalResolution(), tec.getControlPeriod(), tec.getRobotTrackingPeriodInMillis(robotID)));

    }

    void setMission(String src,String dest)
    {
        rsp.setStart(Missions.getLocation(src));
        rsp.setGoals(Missions.getLocation(dest));
        rsp.plan();
        PoseSteering [] path = rsp.getPath();
        
        //###############################################
        //لانشاء مسار
        //###############################################


        Mission mission = new Mission(robotID,path);
        Missions.enqueueMission(mission);
        countMission++;
    }


    void setFootprint(TrajectoryEnvelopeCoordinatorSimulation tec,Coordinate[] footprint2)
    {
        footprint = footprint2;
        tec.setFootprint(robotID,footprint2);
    }

    void setFootprint()
    {
        rsp.setFootprint(this.footprint);
    }

    void placeRobot(TrajectoryEnvelopeCoordinatorSimulation tec,String pos)
    {
        Pose position;
        position = Missions.getLocation(pos);
        tec.placeRobot(robotID, position);
    }

    //----------------------------------------------------------------
    void setMap(String MapYamlFile)
    {
        rsp.setMap(MapYamlFile);
    }

    void setRadius(double rad)
    {
        rsp.setRadius(rad);
    }

    void setTurningRadius(double rad)
    {
        rsp.setTurningRadius(rad);
    }

    void setDistanceBetweenPathPoints(double maxDistance)
    {
        rsp.setDistanceBetweenPathPoints(maxDistance);
    }

}
