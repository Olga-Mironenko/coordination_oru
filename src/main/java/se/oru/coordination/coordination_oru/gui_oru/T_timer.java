package se.oru.coordination.coordination_oru.gui_oru;
import java.util.Timer;
import java.util.TimerTask;
import javax.swing.JButton;
import javax.swing.JLabel;

public class T_timer {
    public static long updataTime = 1;
    public static long currentTime = 0;
    public static boolean isStop = false;
    
    public static long time_ms = 0;
    public static JButton  btn_stop;
    public static JLabel l_log;

    public static void setTimer()
    {
        // long seconds = endTime; // عدد الثواني التي تريد عمل مؤقت لها
        currentTime = 0;
        isStop = false;
        Timer timer = new Timer();
        timer.schedule(new TimerTask() 
        {
            public void run() 
            {
                if (isStop) 
                {
                    timer.cancel();
                }
                if(currentTime>time_ms)
                {
                    btn_stop.doClick();
                }
                l_log.setText("Log ("+currentTime+")");
                currentTime+=1;
                
            }
        }, 0, updataTime);
    }
}
