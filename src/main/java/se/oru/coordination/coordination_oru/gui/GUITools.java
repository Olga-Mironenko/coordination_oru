/*
 * Click nbfs://nbhost/SystemFileSystem/Templates/Licenses/license-default.txt to change this license
 * Click nbfs://nbhost/SystemFileSystem/Templates/Classes/Class.java to edit this template
 */
package se.oru.coordination.coordination_oru.gui;

import java.io.IOException;
import java.util.logging.Level;
import java.util.logging.Logger;
import javax.swing.DefaultListModel;
import javax.swing.JButton;
import javax.swing.JLabel;
import javax.swing.JList;
import javax.swing.JTextArea;
import javax.swing.JTextField;
import org.json.simple.parser.ParseException;
import se.oru.coordination.coordination_oru.simulation2D.TrajectoryEnvelopeCoordinatorSimulation;
import se.oru.coordination.coordination_oru.util.Missions;

/**
 *
 * @author bader
 */
public class GUITools {
    
    public static GUIInterface interf = null;
    public static Prog prg = new Prog();
    public static PkgRobotWithMap pkg = new PkgRobotWithMap();
    
    public static void initGUITools()
    {
        pkg.robotJson = new RobotJson[0];
        prg = new Prog();
        pkg = new PkgRobotWithMap();
    }
    
    public static void deleteRobot(int index)
    {
        pkg=pkg.pkgDeleteRobot(index);
    }
    
    public static void stop()
    {
        Timer.isStop=true;           //لايقاف مؤقت التجربة الذي يوضع في سجل الروبوتات
//        LogRobot.isStoplog = false;    //لإيقاف السجل للروبوتات
        
        removeAllMission();


        prg.tec.stopInference();
//        prg.viz.frame.dispose();
//        prg.viz.frame.removeAll();

        prg.tec = new TrajectoryEnvelopeCoordinatorSimulation();
    }
    
    public static void run(JTextArea textArea,int countIteration) {
        try {
            StatusRobot.textArea = textArea;
            StatusRobot.setIterationOfExperiment(countIteration);
            StatusExperiment.setIterationOfExperiment(countIteration);

            prg.progRun(countIteration);
        } catch (InterruptedException ex) 
        {
            Logger.getLogger(GUIInterface.class.getName()).log(Level.SEVERE, null, ex);
        } catch (IOException ex) 
        {
            Logger.getLogger(GUIInterface.class.getName()).log(Level.SEVERE, null, ex);
        } catch (ParseException ex) 
        {
            Logger.getLogger(GUIInterface.class.getName()).log(Level.SEVERE, null, ex);
        }
    }

    public static void btn_save(String fileName)
    {
        if(fileName != "")
        {
            JSONFile.writeDataToJson(fileName, pkg);
        }
    }
    
    public static void btn_Apply(boolean isNewRobot,JButton btn_newRobot,int indexOfList,JList listRobots,JTextField textBox_name,JTextField textBox_velocity,JTextField textBox_acceleration,JTextField textBox_color, JTextArea textBox_size, JTextArea textBox_path,JTextField textBox_iterationRobot2, JLabel textBox_map)
    {
        if(isNewRobot) //هنا يعني لم يتم اضافة روبوت جديد اي لم يتم الضغط على زر روبوت جديد
        {
            //هنا يتم تعديل بيانات الروبوت
            pkg.robotJson[indexOfList].name=textBox_name.getText();
            pkg.robotJson[indexOfList].velocity= Double.valueOf(textBox_velocity.getText()) ;
            pkg.robotJson[indexOfList].accleration= Double.valueOf(textBox_acceleration.getText()) ;
            pkg.robotJson[indexOfList].color=textBox_color.getText();
            pkg.robotJson[indexOfList].size= JSONFile.convertStringtoSizeArray_interface(textBox_size.getText());
            pkg.robotJson[indexOfList].indexSize = JSONFile.convertStringtoSizeArray_interface(textBox_size.getText()).length;
            pkg.robotJson[indexOfList].path= JSONFile.convertStringtoPathArray_interface(textBox_path.getText());
            pkg.robotJson[indexOfList].indexPath = JSONFile.convertStringtoPathArray_interface(textBox_path.getText()).length;

            pkg.robotJson[indexOfList].iterationPath = Integer.parseInt(textBox_iterationRobot2.getText());
        }
        else  //لحفظ معلومات الروبوت الجديد التي تم اضافته
        {
            PkgRobotWithMap newPkg = new PkgRobotWithMap();
            newPkg.robotJson =new RobotJson[1];
            newPkg.yamlFile = new String();
            newPkg.robotJson[0] = new RobotJson();
            newPkg.robotJson[0].name=textBox_name.getText();
            newPkg.robotJson[0].color=textBox_color.getText();
            newPkg.robotJson[0].velocity=Double.valueOf(textBox_velocity.getText()) ;
            newPkg.robotJson[0].accleration=Double.valueOf(textBox_acceleration.getText()) ;

            //
            int len;
            len = JSONFile.convertStringtoSizeArray_interface(textBox_size.getText()).length;
            newPkg.robotJson[0].size = new Point[len];
            for(int j=0;j<len;j++)
            {
                newPkg.robotJson[0].size[j] = new Point();
                Point [] po = JSONFile.convertStringtoSizeArray_interface(textBox_size.getText());
                newPkg.robotJson[0].size[j].setPoint(po[j].x, po[j].y);
            }

            len = JSONFile.convertStringtoSizeArray_interface(textBox_path.getText()).length;
            newPkg.robotJson[0].path = new Point[len];
            for(int j=0; j<len; j++)
            {
                newPkg.robotJson[0].path[j] = new Point();
                Point [] po = JSONFile.convertStringtoSizeArray_interface(textBox_path.getText());
                newPkg.robotJson[0].path[j].setPoint(po[j].x, po[j].y, po[j].theta);
            }
            //
            newPkg.robotJson[0].indexSize=newPkg.robotJson[0].size.length;
            newPkg.robotJson[0].indexPath=newPkg.robotJson[0].path.length;
            //
            newPkg.yamlFile = GUITools.pkg.yamlFile;
//            newPkg.robotJson[0].iteration = Integer.parseInt(textBox_iterationRobot.getText());
            newPkg.robotJson[0].iterationPath = Integer.parseInt(textBox_iterationRobot2.getText());
            //
            pkg=pkg.pkgNewItem(newPkg);
            
            //------------------------------------------------------------
            DefaultListModel lModel = new DefaultListModel();
            int i;
            for(i=0; i< GUITools.pkg.robotJson.length; i++)
            {
                //listRobots.addItem("R"+(i+1));
                lModel.addElement("R"+(i+1));
            }
            listRobots.setModel(lModel);
            //-------------------------------------------------------------

            btn_newRobot.setText("New Robot");

        }
    }
    
    public static void selectListRobots(int indexOfList,JList listRobots,JTextField textBox_name,JTextField textBox_velocity,JTextField textBox_acceleration,JTextField textBox_color, JTextArea textBox_size, JTextArea textBox_path,JTextField textBox_iterationRobot2,JLabel textBox_map)
    {
        if(listRobots.getModel().getSize()!=0)
        {
            indexOfList=listRobots.getSelectedIndex();

            textBox_name.setText(pkg.robotJson[indexOfList].name);
            textBox_velocity.setText(pkg.robotJson[indexOfList].velocity +"");
            textBox_acceleration.setText(pkg.robotJson[indexOfList].accleration+"");
            textBox_color.setText(pkg.robotJson[indexOfList].color);
            textBox_size.setText(JSONFile.dataToSize_interface(pkg.robotJson[indexOfList].size));
            textBox_path.setText(JSONFile.dataToPath_interface(pkg.robotJson[indexOfList].path));
            textBox_map.setText(pkg.yamlFile);

            textBox_iterationRobot2.setText(pkg.robotJson[indexOfList].iterationPath+"");
        }
    }

    public static void removeAllMission()
    {
        for(int i = 1;i<=prg.myRobot.length;i++)
        {
            while(Missions.getMissions(i).size()!=0)
            Missions.dequeueMission(i);
        }
    }
    
    
}
