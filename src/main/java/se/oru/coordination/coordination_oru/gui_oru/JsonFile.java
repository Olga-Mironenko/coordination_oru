package se.oru.coordination.coordination_oru.gui_oru;

import java.io.BufferedWriter;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;

import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import org.json.simple.parser.ParseException;



public class JsonFile {

    static void readDataFromJson(String fileName,PkgRobotWithMap pkg) throws FileNotFoundException, IOException, ParseException
    {        

        Object o = new JSONParser().parse(new FileReader(fileName));

        JSONObject j = (JSONObject) o;

        String yamlFile = (String) j.get("map");

        JSONArray robot_arr = (JSONArray) j.get("Robots");

        pkg.intiRobotJson(robot_arr.size());

        for(int i=0;i<robot_arr.size();i++)
        {
            JSONArray robot_i = (JSONArray) robot_arr.get(i);
            pkg.robotJson[i].name=(String) robot_i.get(0);
            pkg.robotJson[i].velocity=(Double) robot_i.get(1);
            pkg.robotJson[i].accleration=(Double) robot_i.get(2);
            pkg.robotJson[i].color=(String) robot_i.get(3);

            Point[] sizeOrPath = convertStringtoSizeArray_interface((String) robot_i.get(4));
            pkg.robotJson[i].setCountOfSize(sizeOrPath.length);
            pkg.robotJson[i].size=convertStringtoSizeArray_interface((String) robot_i.get(4));
            pkg.robotJson[i].indexSize=sizeOrPath.length;

            sizeOrPath = convertStringtoPathArray_interface((String) robot_i.get(5));
            pkg.robotJson[i].setCountOfPath(sizeOrPath.length);
            pkg.robotJson[i].path=convertStringtoPathArray_interface((String) robot_i.get(5));
            pkg.robotJson[i].indexPath=sizeOrPath.length;
            
//            pkg.robotJson[i].iteration=(int)((long) robot_i.get(6));
            pkg.robotJson[i].iterationPath=(int)((long) robot_i.get(6));


            for(int j1=0;j1<pkg.robotJson[i].path.length;j1++)
            System.out.println(pkg.robotJson[i].path[j1].x+"   "+pkg.robotJson[i].path[j1].y+"   "+pkg.robotJson[i].path[j1].theta);
        }

        pkg.yamlFile = yamlFile;
        
        JSONArray experimentArr = (JSONArray) j.get("Experiment");
        pkg.csvFileName = (String) experimentArr.get(0);
        pkg.experimentIteration = ((long) experimentArr.get(1));

        
    }

    static void writeDataToJson(String fileName,PkgRobotWithMap pkg)
    {
        JSONObject json = new JSONObject();
        
        try {
            JSONArray li = new JSONArray();
            json.put("map", pkg.yamlFile);
            
            for(int i=0;i<pkg.robotJson.length;i++)
            {
                JSONArray robot_me = new JSONArray();
                robot_me.add(pkg.robotJson[i].name);
                robot_me.add(pkg.robotJson[i].velocity);
                robot_me.add(pkg.robotJson[i].accleration);
                robot_me.add(pkg.robotJson[i].color);
                robot_me.add(dataToSize_interface(pkg.robotJson[i].size));   //==========================
                robot_me.add(dataToPath_interface(pkg.robotJson[i].path));   //==========================
//                robot_me.add(pkg.robotJson[i].iteration);
                robot_me.add(pkg.robotJson[i].iterationPath);
                
                li.add(robot_me);
                json.put("Robots",li);
            }
            
            JSONArray experiment_me = new JSONArray();
            experiment_me.add(pkg.csvFileName);
            experiment_me.add(pkg.experimentIteration);
            json.put("Experiment",experiment_me);
            

        } catch (Exception e) {
            //e.printStackTrace();
        }
 
        try (PrintWriter out = new PrintWriter(new FileWriter(fileName))) {
            out.write(json.toString());
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    //لانشاء من معلومات الروبوتات ملف المسار لكي يتم استخدامه في المشروع
    static void createPathFile(String fileName,RobotJson []robotJson)
    {
        writeFile(fileName,"",false);  //to create empty file
        writeFile(fileName,"#positions\n",true);
        boolean isContinue = true;
        
        int []countPathR = new int[robotJson.length];
        for(int i=0;i<robotJson.length;i++)
        countPathR[i]=robotJson[i].path.length;

        while(isContinue)
        {
            isContinue=false;
            for(int i=0;i<robotJson.length;i++)
            if(countPathR[i]!=0)
            {
                int indW=robotJson[i].path.length - countPathR[i];
                writeFile(fileName, "a"+(i+1)+""+(indW + 1)+"	"+robotJson[i].path[indW].x+" "+robotJson[i].path[indW].y+" "+robotJson[i].path[indW].theta+" 0.0", true);
                countPathR[i]--;
                isContinue=true;

                for(int i1=0;i1<robotJson.length;i1++)
                if(countPathR[i1]!=0)
                {
                    writeFile(fileName,"\n",true);
                    break;
                }
            }
            if(isContinue)
            writeFile(fileName,"\n",true);
        }
    }

    public static void writeFile(String fileName,String text,Boolean append)
    {
        try 
        {
            BufferedWriter f_writer = new BufferedWriter(new FileWriter(fileName,append));
            f_writer.write(text);
            f_writer.close();
        } catch (Exception e) {
        }
    }

    static String dataToSize_interface(Point[]size1)
    {   
        String sizeAsString="";
        String p;
        for(int i=0;i<size1.length;i++)
        {
            p="("+size1[i].x+","+size1[i].y+")";
            
            if(i==size1.length-1)
            sizeAsString+=p;
            else
            sizeAsString+=p+" -> ";

        }
        return sizeAsString;
    }

    static String dataToPath_interface(Point[]path1)
    {
        String pathAsString="";
        String p;
        for(int i=0;i<path1.length;i++)
        {
            p="("+path1[i].x+","+path1[i].y+","+path1[i].theta+")";
            
            if(i==path1.length-1)
            pathAsString+=p;
            else
            pathAsString+=p+" -> ";
        }
        return pathAsString;
    }

    //(1,2) -> (2,4) -> (5,6)
    //(1,2)->(2,4)->(5,6)

    static Point[] convertStringtoSizeArray_interface(String sizeAsString)
    {

        sizeAsString=sizeAsString.replaceAll(" ","");
        String []size2=sizeAsString.split("->");
        Point[] sizeArray = new Point[size2.length];
        for(int i=0;i<size2.length;i++)
        sizeArray[i]=new Point();

        double x,y;
        for(int i=0;i<size2.length;i++)
        {
            //(x,y) -> [(x , y)]
            x=Double.valueOf(size2[i].split(",")[0].split("\\(")[1]);
            y=Double.valueOf(size2[i].split(",")[1].split("\\)")[0]);
            sizeArray[i].setPoint(x, y);
        }

        return sizeArray;
    }

    //(1,2,*) -> (2,4,*) -> (5,6,*)
    //(1,2,*)->(2,4,*)->(5,6,*)

    static Point[] convertStringtoPathArray_interface(String pathAsString)
    {
        pathAsString=pathAsString.replaceAll(" ","");
        String []path2=pathAsString.split("->");
        Point[] pathArray = new Point[path2.length];
        for(int i=0;i<path2.length;i++)
        pathArray[i]=new Point();

        double x,y,theta;
        for(int i=0;i<path2.length;i++)
        {
            //(x,y,*) -> [(x , y , *)]
            x=Double.valueOf(path2[i].split(",")[0].split("\\(")[1]);
            y=Double.valueOf(path2[i].split(",")[1]);
            theta=Double.valueOf(path2[i].split(",")[2].split("\\)")[0]);
            pathArray[i].setPoint(x,y,theta);
        }

        return pathArray;
    }

}
