package se.oru.coordination.coordination_oru.gui;

public class RobotJson {
    String name = "";
    double velocity = 0;
    double accleration = 0;
    String color = "";
    Point [] size;
    Point [] path;
    int indexSize = 0;
    int indexPath = 0;
    //int iteration=0;
    int iterationPath = 1;
    
    RobotJson(RobotJson robotJson2)
    {
        this.name=robotJson2.name;
        this.velocity=robotJson2.velocity;
        this.accleration=robotJson2.accleration;
        this.color=robotJson2.color;
        this.size=robotJson2.size;
        this.path=robotJson2.path;
        this.indexSize=robotJson2.indexSize;
        this.indexPath=robotJson2.indexPath;
    }
    
    RobotJson()
    {
        ;
    }

    void setCountOfSize(int n) {
        size = new Point[n];
        for(int i=0;i<n;i++) {
            size[i] = new Point();
        }
    }
    
    void setCountOfPath(int n)
    {
        path = new Point[n];
        for(int i=0;i<n;i++)
        path[i] = new Point();
    }

    void setSize(double x,double y)
    {
        size[indexSize].x=x;
        size[indexSize].y=y;
        indexSize++;
    }

    void setPath(double x,double y,double theta)
    {
        path[indexPath].x=x;
        path[indexPath].y=y;
        path[indexPath].theta=theta;
        indexPath++;
    }
    
    RobotJson newRobotJson()
    {
        RobotJson newRobotJson = new RobotJson();
        newRobotJson.name=this.name;            
        newRobotJson.color=this.color;
        newRobotJson.velocity=this.velocity;
        newRobotJson.accleration=this.accleration;
        
        int len;
        len = this.size.length;
        newRobotJson.size = new Point[len];
        for(int j=0;j<len;j++)
        {
            newRobotJson.size[j] = new Point();
            newRobotJson.size[j].setPoint(this.size[j].x, this.size[j].y);
        }

        len = this.path.length;
        newRobotJson.path = new Point[len];
        for(int j=0;j<len;j++)
        {
            newRobotJson.path[j] = new Point();
            newRobotJson.path[j].setPoint(this.path[j].x, this.path[j].y, this.path[j].theta);
        }

        newRobotJson.indexSize=this.size.length;
        newRobotJson.indexPath=this.path.length;
        
        return newRobotJson;
    }
}
