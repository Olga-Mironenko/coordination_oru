package se.oru.coordination.coordination_oru.gui_oru;

public class PkgRobotWithMap {
    RobotJson []robotJson;
    String yamlFile="";
    String csvFileName="";
    long experimentIteration;
    
    
    void intiRobotJson(int n)
    {
        robotJson = new RobotJson[n];
        for(int i=0;i<n;i++)
        robotJson[i]=new RobotJson();
    }
    PkgRobotWithMap(PkgRobotWithMap pkg)
    {
        for(int i=0;i<pkg.robotJson.length;i++)
        this.robotJson[i] = new RobotJson(pkg.robotJson[i]);
        this.yamlFile = pkg.yamlFile;
    }
    PkgRobotWithMap()
    {
        ;
    }
    
    //لاضافة روبوت الى داخل البكج الذي يحوي جميع الروبوتات
    PkgRobotWithMap pkgNewItem(PkgRobotWithMap pkg)
    {
        PkgRobotWithMap newPkg = new PkgRobotWithMap();
        
        newPkg.robotJson = new RobotJson[this.robotJson.length+1];
        int i;
        int len;
        for(i=0;i<this.robotJson.length;i++)
        {
            newPkg.robotJson[i] = new RobotJson();
            newPkg.robotJson[i] = this.robotJson[i].newRobotJson();
        }
            newPkg.robotJson[i] = new RobotJson();
            newPkg.robotJson[i] = pkg.robotJson[0].newRobotJson();   
            newPkg.yamlFile=pkg.yamlFile;
        return newPkg;
    }
    
    //لحذف روبوت من البكج عن طريق رقم الايندكس
    PkgRobotWithMap pkgDeleteRobot(int indexRobot)
    {
        PkgRobotWithMap pkg = new PkgRobotWithMap();
        pkg.robotJson = new RobotJson[this.robotJson.length-1];
        for(int i=0;i<this.robotJson.length-1;i++)
        {
            pkg.robotJson[i]=new RobotJson();
        }

        int j=0;
        for(int i=0;i<this.robotJson.length;i++)
            if(i!=indexRobot)
            {
                pkg.robotJson[j] = this.robotJson[i].newRobotJson();
                j++;
            }
        pkg.yamlFile = this.yamlFile;
        return pkg;
    }
    
    
}
