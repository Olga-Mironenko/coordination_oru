package se.oru.coordination.coordination_oru.gui;


import com.vividsolutions.jts.geom.Coordinate;
import org.json.simple.parser.ParseException;
import se.oru.coordination.coordination_oru.*;
import se.oru.coordination.coordination_oru.simulation2D.TrajectoryEnvelopeCoordinatorSimulation;
import se.oru.coordination.coordination_oru.util.JTSDrawingPanelVisualization;
import se.oru.coordination.coordination_oru.util.Missions;

import javax.swing.*;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.util.Comparator;

public class Prog {

    TrajectoryEnvelopeCoordinatorSimulation tec = new TrajectoryEnvelopeCoordinatorSimulation();
    JTSDrawingPanelVisualization viz;
//    Boolean []whileThread;
    MyRobot [] myRobot;
    PkgRobotWithMap pkg;

    public static JTextArea textArea = null;
    
    public void progRun(int iteration_Experiment) throws InterruptedException, FileNotFoundException, IOException, ParseException{

        pkg = GUITools.pkg;

        MyRobot.initMyRobot();
        
        myRobot = new MyRobot [pkg.robotJson.length];
        for(int i=0;i<pkg.robotJson.length;i++)
        {
            myRobot[i]=new MyRobot();
        }

        //here
        runOru();   //initExperiment for all robot

        Integer[] robotIDs = new Integer[pkg.robotJson.length];
        for(int i=0;i<pkg.robotJson.length;i++)
        robotIDs[i] = i+1;
        

        //>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
        
        int []iterRobot = new int[myRobot.length];
        for(int i=0;i<myRobot.length;i++)
            iterRobot[i]=iteration_Experiment;   //**************************************************8
        
        StatusRobot.initStatus();
        StatusRobot.setCountOfRobot(myRobot.length,iterRobot);
        StatusRobot[] statusRobot = new StatusRobot[myRobot.length];
        for(int robotID=1;robotID<=robotIDs.length;robotID++)
            statusRobot[robotID-1] = new StatusRobot( Missions.getMissions(robotID));
        

        TrackingCallback cb = new TrackingCallback(null) {

                @Override
                public void onTrackingStart() { 
                    for (int robotID : robotIDs)
                    {
                        statusRobot[robotID-1].statusRobot(tec, robotID);
                        statusRobot[robotID-1].anlysis(robotID);
                    }
                }

                @Override
                public void onTrackingFinished() { 

                    for (int robotID : robotIDs)
                    {
                        statusRobot[robotID-1].statusRobot(tec, robotID);
                        statusRobot[robotID-1].anlysis(robotID);
                    }
                }

                @Override
                public String[] onPositionUpdate() {
                    for (int robotID : robotIDs)
                    {
                        statusRobot[robotID-1].statusRobot(tec, robotID);
                        statusRobot[robotID-1].anlysis(robotID);
                    }
                    return null;
                }

                @Override
                public void onNewGroundEnvelope() { 
                    for (int robotID : robotIDs)
                    {
                        statusRobot[robotID-1].statusRobot(tec, robotID);
                        statusRobot[robotID-1].anlysis(robotID);
                    }
                }

                @Override
                public void beforeTrackingStart() { 
                    for (int robotID : robotIDs)
                    {
                        statusRobot[robotID-1].statusRobot(tec, robotID);
                        statusRobot[robotID-1].anlysis(robotID);
                    }
                }

                @Override
                public void beforeTrackingFinished() {
                    for (int robotID : robotIDs)
                    {
                        statusRobot[robotID-1].statusRobot(tec, robotID);
                        statusRobot[robotID-1].anlysis(robotID);
                    }
                }
        };
        
        
        for (int robotID : robotIDs)
        tec.addTrackingCallback(robotID, cb);

        StatusExperiment.start(robotIDs.length);
        //>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

        //Start a mission dispatching thread for each robot, which will run forever
        for (final Integer robotID : robotIDs) {
                //For each robot, create a thread that dispatches the "next" mission when the robot is free 
                Thread t = new Thread() {
                        int iteration = 0;
                        @Override
                        public void run() 
                        {
                            
                            while (StatusRobot.statusThread[robotID-1]) 
                            {

                                Mission m = Missions.getMission(robotID, iteration%Missions.getMissions(robotID).size());

                                synchronized(tec) 
                                {
                                    if(StatusExperiment.isAddMission(robotID))
                                    {
                                        //addMission returns true iff the robot was free to accept a new mission
                                        if (tec.addMissions(m)) 
                                        {
                                            StatusExperiment.addMission(robotID);
                                            iteration++;
                                        }
                                    }
                                }
                                
                                StatusExperiment.update();

                                try { Thread.sleep(200); 
                                }
                                catch (InterruptedException e) { e.printStackTrace(); }
                            }
                        }
                };
                //Start the thread!
                t.start();
        }
    }

	

	public  void initFootPrint(MyRobot []myRobot,PkgRobotWithMap pkg)
	{
		Coordinate[][] fb = new Coordinate[pkg.robotJson.length][];
		for(int i=0;i<pkg.robotJson.length;i++)
		{
			fb[i]=new Coordinate[pkg.robotJson[i].indexSize];
			
			for(int j=0;j<pkg.robotJson[i].indexSize;j++)
			{
				fb[i][j]=new Coordinate(pkg.robotJson[i].size[j].x,pkg.robotJson[i].size[j].y);
			}
			myRobot[i].setFootprint(tec,fb[i]);
		}
	}


//---------------------------------------------------------------------------------------------------------------
        
	public void runOru() 
	{
                tec.addComparator(new Comparator<RobotAtCriticalSection> () {
                        @Override
                        public int compare(RobotAtCriticalSection o1, RobotAtCriticalSection o2) {
                                CriticalSection cs = o1.getCriticalSection();
                                RobotReport robotReport1 = o1.getRobotReport();
                                RobotReport robotReport2 = o2.getRobotReport();
                                
                                return ((cs.getTe1Start()-robotReport1.getPathIndex())-(cs.getTe2Start()-robotReport2.getPathIndex()));
                        }
		});
                
		tec.addComparator(new Comparator<RobotAtCriticalSection> () {
			@Override
			public int compare(RobotAtCriticalSection o1, RobotAtCriticalSection o2) {
				return (o2.getRobotReport().getRobotID()-o1.getRobotReport().getRobotID());
			}
		});
		
		JSONFile.createPathFile(new File("").getAbsolutePath()+"/"+"path.txt",pkg.robotJson);
		Missions.loadRoadMap(new File("").getAbsolutePath()+"/"+"path.txt");

		initFootPrint(myRobot,pkg);

		//Need to setup infrastructure that maintains the representation
                //100000000
		tec.setupSolver(0, 100000000);
		//Start the thread that checks and enforces dependencies at every clock tick
		tec.startInference();

		//Setup a simple GUI (null means empty map, otherwise provide yaml file)
		String yamlFile = pkg.yamlFile;
		viz = new JTSDrawingPanelVisualization(yamlFile);
                
		tec.setVisualization(viz);
                
                //====================================================================================
//		tec.setUseInternalCriticalPoints(false);
//		tec.setBreakDeadlocks(false, false, false);
//		tec.setYieldIfParking(false);

                //اذا كان في نقطة حرجة على نفس المسار يتوقف الروبوت حذرا من التصادم
                tec.setUseInternalCriticalPoints(false);
//                tec.setCheckEscapePoses(true);
//                tec.setStaticReplan(true);
                tec.setYieldIfParking(false);
                tec.setBreakDeadlocks(false, true, true);
                tec.setCheckCollisions(true);
                
                
                JSONFile.writeFile("Maria222.txt","",false);
		int way_1,way_2;
		for(int i=0;i<pkg.robotJson.length;i++)
		{	
			
                    myRobot[i].setSpeed(tec,pkg.robotJson[i].velocity, pkg.robotJson[i].accleration);

                    myRobot[i].placeRobot(tec,"a"+(i+1)+""+1);

                    //Set up private motion planners.
                    myRobot[i].setMap(pkg.yamlFile);

                    myRobot[i].setRadius(0.1);

                    myRobot[i].setTurningRadius(1);

                    myRobot[i].setFootprint();

                    myRobot[i].setDistanceBetweenPathPoints(0.05);

                    tec.setMotionPlanner(i+1, myRobot[i].rsp);

                    JSONFile.writeFile("Maria222.txt","R"+i+1+":"+pkg.robotJson[i].path.length+"\n",true);

                    for(int countRobot=1; countRobot<=pkg.robotJson[i].iterationPath;countRobot++)
                    {
                        int k=1;
                        while(true)
                        {   
                            way_1=k;
                            way_2=k+1;
                            if(way_2>pkg.robotJson[i].path.length) break;

                            myRobot[i].setMission("a"+(i+1)+""+way_1,"a"+(i+1)+""+way_2);
                            k++;
                        }
                    }
			
		}

	}

}
