/*
 * Click nbfs://nbhost/SystemFileSystem/Templates/Licenses/license-default.txt to change this license
 * Click nbfs://nbhost/SystemFileSystem/Templates/Classes/Class.java to edit this template
 */
package se.oru.coordination.coordination_oru.gui_oru;


import java.util.ArrayList;
import java.util.logging.Level;
import java.util.logging.Logger;
import javax.swing.JTextArea;
import se.oru.coordination.coordination_oru.Mission;
import se.oru.coordination.coordination_oru.simulation2D.TrajectoryEnvelopeCoordinatorSimulation;

/**
 *
 * @author bader
 */
public class StatusRobot {
    
    //finish[]: تعبر عن انتهاء مسار الروبوت
    //statusThread[]: هذه لايقاف الثريد الذي يعمل عليها الروبوت بعد انتهائه من العمل
    //endTimeOfRobot[]: هذه المصفوفة يتم التخزين فيها انتهاء وقت الروبوت لكل تجربة
    //collisions: يتم وضع فيها عدد التصادمات وتتحدث قيمتها في كل تجربة
    //setFileName_CSV(String fileName): يأخذ هذا التابع اسم ملف الCSV مع اللاحقة
    //setCountOfRobot(int count_robot): يأخذ هذا التابع عدد الروبوتات
    //setEnable_waitingTimeOfRobot(enable): يعمل هذه التابع على تخزين وقت انتظار الروبوت للروبوتات الاخرى في كل تجربة بداخل ملف السي اس في CSV 
    //setEnable_endTimeOfRobot(enable): يعمل هذه التابع على تخزين وقت انتهاء الروبوت في التجربة بداخل ملف السي اس فيCSV
    //setEnable_numberOfCollisions(enable): يعمل هذه التابع على تخزين عدد التصادمات التي حدثت بداخل التجربة التي يعمل عليها الروبوت بداخل ملف السي اس فيCSV
    //setEnable_iterationExperiment(enable): يعمل هذه التابع على تخزين رقم التجربة بداخل ملف السي اس في CSV
    //setEnable_endTimeOfExperiment(enable): يعمل هذه التابع على تخزين وقت انتهاء التجربة في ملف السي اس في CSV
    //setIterationOfExperiment(int iter): لاضافة عدد التجارب الذي سوف يعمل عليها الروبوت
    //initStatus(): لتهيئة القيم الافتراضية للروبوت
    //StatusRobot(ArrayList<Mission> arrMission): لاخذ المهمات الذي يعمل عليها الروبوت
    //statusRobot(TrajectoryEnvelopeCoordinatorSimulation tec,int robotID): لتحديث معلومات حالة الروبوت اي لتحديث هل هو متوقف ام يعمل
    
    //Driving(int robotID,boolean writeLog): لمعرفة حالة الروبوت حيث يرجع هذا التابع قيمة صحيحة اذا كان الروبوت يعمل اما اذا لم يعمل يرجع قيمة خاطئة
    //حيث الrobotID هو رقم الروبوت اماwriteLog وهذا ليتم وضع البيانات بداخل الTextArea الخاصة ب Log
    
    //Stoping(int robotID,boolean writeLog): لمعرفة حالة الروبوت حيث يرجع هذا التابع قيمة صحيحة اذا كان الروبوت متوقف و يرجع قيمة خاطئة اذا كان يعمل
    //حيث الrobotID هو رقم الروبوت اماwriteLog وهذا ليتم وضع البيانات بداخل الTextArea الخاصة ب Log
    
    //currentIsEndPosition(int robotID,boolean writeLog)  //لمعرفة هل الروبوت وصل لنهاية المسار ام لأ
    //finishExperiment(): يرجع هذا التابع قيمة صحيحة اذا كان الروبوت قد تم الانتهاء من التجربة الحالية ويرجع قيمة خطأ بخلاف ذلك
    
    //nextExperiment(int robotID): يعمل هذا التابع على اعطاء التجربة التالية وذلك عند انتهاء جميع الروبوتات من التجربة الحالية
    //stopTimer(): يعمل هذا التابع على ايقاف عمل جميع الروبوتات وايقاف المؤقت الذي يعمل
    
    //anlysis(int robotID): يعمل هذا التابع على تحليل حالة الروبوت
    
    static boolean isFirstOpen = false;
    public static JTextArea textArea=null;
    boolean is_Drive=false;  //لم يمشي قبل هل مرة
    boolean is_Stop=false;   //لم يتوقف قبل هل مرة
    
    public static String csvFileName="file.csv";
    
    int i_Stop=0;
    
    int iteration = 1; //عدد مرات سير هذا الروبوت على المسار
    
    static int countOfRobot = 0;
    public static boolean []finish;
    public static boolean []statusThread;
    static long []endTimeOfRobot;
    public static int iterRobot[];
    public static void setCountOfRobot(int count_robot,int[] iterationRobots)
    {
        countOfRobot=count_robot;
        finish = new boolean[count_robot];
        statusThread = new boolean[count_robot];
        endTimeOfRobot = new long[count_robot];
        iterRobot = new int[count_robot];
//        iterRobot[0] = 1;
//        iterRobot[1] = 4;
        for(int i=0;i<count_robot;i++)
        {
            iterRobot[i] = iterationRobots[i];
            if(iterRobot[i]>0)
            {
                finish[i]=false; //الروبوت لم ينتهي من مساره
                endTimeOfRobot[i]=0;
                statusThread[i]=true;
            }
            else
            {
                finish[i]=true; //الروبوت منتهي من مساره
                endTimeOfRobot[i]=-1;
                statusThread[i]=false;
            }
            
        }
    }
    
    public static int countCollisionsList = 0;
    public static int collisions = 0;
    public static int controlCountCollisions = 0;
    //>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
    static boolean enable_waitingTimeOfRobot=false;
    static boolean enable_endTimeOfRobot=false;
    static boolean enable_numberOfCollisions=false;
    static boolean enable_iterationExperiment=false;
    static boolean enable_endTimeOfExperiment=false;
    static int iterationOfExperiment = 1;
    
    
    
    public static void setIterationOfExperiment(int iter)
    {
        iterationOfExperiment=iter;
    }
    
    public static void setFileName_CSV(String fileName)
    {
        if(fileName!="")
            csvFileName=fileName;
        else
            csvFileName="file.csv";
    }
    
    public static void setEnable_waitingTimeOfRobot(boolean enable)
    {
        enable_waitingTimeOfRobot = enable;
    }
    public static void setEnable_endTimeOfRobot(boolean enable)
    {
        enable_endTimeOfRobot = enable;
    }
    public static void setEnable_numberOfCollisions(boolean enable)
    {
        enable_numberOfCollisions = enable;
    }
    public static void setEnable_iterationExperiment(boolean enable)
    {
        enable_iterationExperiment = enable;
    }
    public static void setEnable_endTimeOfExperiment(boolean enable)
    {
        enable_endTimeOfExperiment = enable;
    }

    public static void initStatus()
    {
//        csvFileName="file.csv";
        isFirstOpen = false;
        textArea.setText("");
        iterExperiment=1;
        isFinishExperiment = false;
        countCollisionsList = 0;
        controlCountCollisions = 0;
        collisions = 0;
        
        
        
    }
    //>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
   
    ArrayList<Mission> arrMission;
    public StatusRobot(ArrayList<Mission> arrMission)
    {
        this.arrMission = arrMission;
    }
    
    public int currentPP;
    public int currentPP_end;
    public void statusRobot(TrajectoryEnvelopeCoordinatorSimulation tec,int robotID)  //لتحديث معلومات حالة الروبوت 
    {
        currentPP = tec.getRobotReport(robotID).getPathIndex();
        currentPP_end = tec.getCurrentTrajectoryEnvelope(robotID).getTrajectory().getPositions().length - 1;
        countCollisionsList = tec.getCountCollisionsList() - controlCountCollisions;
    }
    
    public boolean isDrive()
    {
        if(currentPP>0)
            return true;
        
        return false;
    }
    
    public boolean isStop()
    {
        if(currentPP==-1)
        {
            return true;
        }
        return false;
    }
    
    public boolean Driving(int robotID,boolean writeLog)
    {

        if(isDrive() && writeLog == true && is_Drive==false)
        {
            is_Drive=true;
            is_Stop=false;
            String st="";
            
            if(i_Stop == arrMission.size()) 
            {
                //st = "*Stop: \n";
                iteration++;
                
                
                if(textArea!=null)
                {
                    st += ">> Repeat \n";
                    st += "Time: " + T_timer.currentTime + " ms \n";
                    st += "Robot ("+robotID+"): "+"Stop " +"\n";
                    st += "From Location: " + arrMission.get(0).getFromLocation() +"\n";
                    st += "Source: "+ arrMission.get(0).getFromLocation() + "\n";
                    st += "Destination: "+ arrMission.get(arrMission.toArray().length-1).getToLocation()  + "\n";
                    st += "Iteration: " + iteration +"\n";
                    textArea.setText(textArea.getText() + st + "\n");
                textArea.setSelectionStart(textArea.getText().length());
                }
                i_Stop=0;
            }
            
            if(textArea!=null)
            {
                st += "Time: " + T_timer.currentTime + " ms \n";
                st += "Robot ("+robotID+"): "+"Drive " +"\n";
                st += "To Location: " + arrMission.get(i_Stop).getToLocation() +"\n";
                st += "Source: "+ arrMission.get(0).getFromLocation() + "\n";
                st += "Destination: "+ arrMission.get(arrMission.toArray().length-1).getToLocation()  + "\n";
                st += "Iteration: " + iteration +"\n";
                textArea.setText(textArea.getText() + st + "\n");
                textArea.setSelectionStart(textArea.getText().length());
            }

            i_Stop++;
            return true;
        }
        if(isDrive() && writeLog == false)
        {
            is_Drive=true;
            is_Stop=false;
            return true;
        }
        
        return false;
    }
    
    public boolean Stoping(int robotID,boolean writeLog)
    {
        if(isFirstOpen == false)
        {
            isFirstOpen = true;
            JsonFile.writeFile(csvFileName,"",false);
            T_timer.isStop = true;
            T_timer.setTimer();
        }
        
        //isStop() يعني هلأ توقف
        //is_Stop يعني لم يتوقف قبل هل مرة او يعني يسبقه حدث السير  وذلك عندما يأخذ قيمة خطأ
        //writeLog يعني طباعة ماذا يحدث اذا كان صحيح واذا كان خطأ لم يطبع شيء وذلك لتحقيق للتابع على ان يكون فيه تابع اختبار للروبوت لكي نعرف حالته
        if(isStop() && writeLog == true && is_Stop==false)
        {
            is_Drive=false;
            is_Stop=true;
            String st="";
            IsEndPosition = false; // يعني ان الروبوت لم يتوقف عند النقطة الاخيرة من المسار
            
            if(textArea!=null)
            {
                st += "Time: " + T_timer.currentTime + " ms \n";
                st += "Robot ("+robotID+"): "+"Stop " +"\n";

                if(i_Stop == arrMission.size()) //اذا وصل للاخير
                {
                    IsEndPosition = true;
                    endTimeOfRobot[robotID-1]=T_timer.currentTime;
                    finish[robotID-1]=true;
                    StatusExperiment.stopRobot[robotID-1]=-1;
                    st += "From Location: " + arrMission.get(arrMission.size()-1).getToLocation() +"\n";
                }
                else
                {
                    st += "From Location: " + arrMission.get(i_Stop).getFromLocation() +"\n";
                }

                st += "Source: "+ arrMission.get(0).getFromLocation() + "\n";
                st += "Destination: "+ arrMission.get(arrMission.toArray().length-1).getToLocation()  + "\n";
                st += "Iteration: " + iteration +"\n";
                textArea.setText(textArea.getText() +st+"\n");
                textArea.setSelectionStart(textArea.getText().length());
            }
            
            return true;
        }
        if(isStop() && writeLog == false)
        {
            if(i_Stop == arrMission.size()) //اذا وصل للاخير
            {
                IsEndPosition = true;
                endTimeOfRobot[robotID-1]=T_timer.currentTime;
                finish[robotID-1]=true;
                StatusExperiment.stopRobot[robotID-1]=-1;
            }
            is_Drive=false;
            is_Stop=true;
            return true;
        }
        
        return false;
    }

    
    boolean IsEndPosition = false;   //لم يتوقف قبل هل مرة عند نهاية المسار
    public boolean currentIsEndPosition(int robotID,boolean writeLog)  //لمعرفة هل الروبوت وصل لنهاية المسار ام لأ
    {
        if(IsEndPosition == true)  
        {
                iterRobot[robotID-1]--;
            
            IsEndPosition = false;
            if(writeLog)
            {
                if(textArea!=null)
                {
                    textArea.setText(textArea.getText() +"##### Robot ("+robotID+") - Path Completed #####"+"\n\n");
                    textArea.setSelectionStart(textArea.getText().length());
                }
                
                if(textArea!=null && StatusExperiment.isCompleteExperiment())
                {
                    textArea.setText(textArea.getText() +"##### Experiment ("+iteration+") - Collision: "+collisions+" #####"+"\n\n");
                    textArea.setText(textArea.getText() +"##### Experiment ("+iteration+") - Completed #####"+"\n\n");
                    textArea.setSelectionStart(textArea.getText().length());
                }
            }

            return true;
        }
        return false;
    }
    
    static Thread t;
    public static int iterExperiment=1;
    static boolean isFinishExperiment = false;
    static boolean finishExperiment()
    {
        //if(!StatusExperiment.isCompleteExperiment())
        //return false;
        for(int i=0;i<countOfRobot;i++)
            if(finish[i]==false)      //اذا كانت احد الروبوتات لم ينتهي قم بالخروج من التابع وإلا استمر في التعليمات التالية
                return false;
        
        if(isFinishExperiment==false)
        {
            isFinishExperiment=true;
            iterExperiment++;
            t = new Thread() 
            {
                @Override
                public void run() 
                {
                    String str="";
                    
                    if(enable_iterationExperiment)
                        str+=(iterExperiment-1)+",";
                    
                    long timeExp=T_timer.currentTime;
                    if(enable_endTimeOfExperiment)
                        str+=timeExp+" ms"+",";
                    
                    collisions = countCollisionsList;
                    if(enable_numberOfCollisions)
                        str += countCollisionsList+",";
                    controlCountCollisions = countCollisionsList;
                    
                    for(int i=0;i<countOfRobot;i++)
                    {
                        if(enable_endTimeOfRobot)
                        {
                            if(endTimeOfRobot[i]!=-1)
                                str+="R"+(i+1)+"(Finish)"+": "+endTimeOfRobot[i]+" ms,";
                            else
                                str+="R"+(i+1)+"(Finish)"+": "+"NULL"+",";
                        }
                        
                        if(enable_waitingTimeOfRobot)
                        {
                            if(endTimeOfRobot[i]!=-1)
                                str+="R"+(i+1)+"(Wait)"+": "+(timeExp-endTimeOfRobot[i])+" ms,";
                            else
                                str+="R"+(i+1)+"(Wait)"+": "+"NULL"+",";
                        }
                        
                        if(iterRobot[i]!=0)
                            endTimeOfRobot[i]=0;
                        else
                            endTimeOfRobot[i]=-1;
                    }
                    
                    JsonFile.writeFile(csvFileName,str+"\n", true);
                }
            };
            t.start();

            try {
                Thread.sleep(200);
            } catch (InterruptedException ex) {
                Logger.getLogger(StatusRobot.class.getName()).log(Level.SEVERE, null, ex);
            }

        }
        
        return true;
    }

    public static void nextExperiment()
    {
        if(finishExperiment())
        {
            if(iterExperiment<=iterationOfExperiment)
            {
//                t.stop();
                isFinishExperiment=false;
                for(int i=0;i<countOfRobot;i++)
                {
                    if(iterRobot[i]>0)
                        finish[i]=false;
                }
            }
            else if(iterExperiment>iterationOfExperiment)
            {
                stopTimer();
            }
        }
    }

    public static void stopTimer()
    {
        for(int i=0;i<countOfRobot;i++)
        {
            statusThread[i]=false;
        }
        T_timer.isStop=true;
    }
    public void anlysis(int robotID)
    {
        
        if(!finish[robotID-1])
        {
            Stoping(robotID,true);
            Driving(robotID,true);
            currentIsEndPosition(robotID,true);
        }
        
        nextExperiment();
        
    }
    
    
    
}
