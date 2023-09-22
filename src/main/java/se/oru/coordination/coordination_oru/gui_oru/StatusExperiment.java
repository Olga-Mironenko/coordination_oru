/*
 * Click nbfs://nbhost/SystemFileSystem/Templates/Licenses/license-default.txt to change this license
 * Click nbfs://nbhost/SystemFileSystem/Templates/Classes/Class.java to edit this template
 */
package se.oru.coordination.coordination_oru.gui_oru;
import se.oru.coordination.coordination_oru.util.Missions;

/**
 *
 * @author bader
 */
public class StatusExperiment {
    
    //countMission[]: لتخزين عدد مهمات كل روبوت موجود بداخل التجربة
    //iteration: لتخزين عدد مرات تكرار التجربة
    //i_iteration: يعبر عن رقم التجربة الحالية
    //stopRobot[]: تعمل هذا المصفوفة على معرفة هل جميع الروبوتات انتهت من العمل ام لأ حيث يوضع بها لكل روبوت يتم الانتهاء من عمله قيمة -1 وقيمة +1 للروبوت الذي لم ينته من عمله
    //setIterationOfExperiment(int iter): لإدخال عدد مرات تكرار التجربة
    //start(int countRobot): لبدأ وضع قيم افتراضية لجميع الروبوتات ويأخذ هذا التابع عدد الروبوتات
    //addMission(int robotID): يستخدم هذا التابع لاضافة مهمة حيث يعمل هذا التابع على انقاص عدد مهمات الروبوت المستخدمة 
    //isAddMission(int robotID): يعمل هذا التابع على التحقق من انه يوجد للروبوت مهمة لم تستخدم بعد ويرجع قيمة صحيحة اذا كان يوجد مهمة وقيمة خاطئة اذا نفذت جميع المهمات لديه وبهذه الحالة يعمل على تحديث حالة التجربةمن خلال استخدام التابعupdate
    //isCompleteExperiment(): يرجع هذا التابع قيمة صحيحة اذا كان الروبوتات قد انتهت مهماتهم في هذه التجربة وقيمة خطأ اذا كان احد الروبوتات مازال يعمل في التجربة
    
    
    
    static int countRobot=0;
    static int []countMission;  
    static int iteration=1;   
    static int i_iteration=1;
    static int [] stopRobot;
    
    public static void setIterationOfExperiment(int iter)
    {
        iteration=iter;
    }
    
    public static void start(int countRobot)
    {
        i_iteration=1;
        StatusExperiment.countRobot = countRobot;
        countMission = new int[countRobot];
        stopRobot = new int[countRobot];
        for(int i=0;i<countRobot;i++)
        {
            countMission[i] = Missions.getMissions(i+1).size();
            stopRobot[i]=1;  //لعمل اختبار للروبوتات لكي يتم الانتهاء التجربة ولم يبقى روبوت فيها يضل عمل وايضا لدخول تجربة بدون روبوتات من التجربة السابقة
        }
        
    }
    
    public static void addMission(int robotID)
    {
        if(countMission[robotID-1]!=0)
            countMission[robotID-1]--;
    }
    
    public static boolean isAddMission(int robotID)
    {
        if(countMission[robotID-1]!=0 && StatusRobot.iterRobot[robotID-1]>0)
            return true;
        
        
        update();
        
        return false;
    }

    public static void update()
    {
        if(isCompleteExperiment()==false)  //اذا لم تنتهي التجربة نخرج من التابع وإلا يستمر في الكود 
            return;

        i_iteration++;
        if(i_iteration<=iteration)
        {
            for(int i=0;i<countRobot;i++)
            {
                if(StatusRobot.iterRobot[i]>0)      //هل الروبوت يوجد له عدد مرات تكرار
                {
                    countMission[i] = Missions.getMissions(i+1).size();
                    stopRobot[i]=1;
                }
            }
        }
        else
        {
            StatusRobot.stopTimer();
        }
    }
    
    public static boolean isCompleteExperiment()
    {
        for(int i=0;i<countRobot;i++)
        if(countMission[i]!=0 || stopRobot[i]==1)
        {
            return false;
        }      
        return true;
    }
    
}
