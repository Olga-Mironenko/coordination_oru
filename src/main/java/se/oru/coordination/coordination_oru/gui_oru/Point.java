package se.oru.coordination.coordination_oru.gui_oru;

public class Point {
    double x;
    double y;
    double theta=0;

    Point()
    {
        ;
    }

    Point(double x,double y,double theta)
    {
        setPoint(x, y, theta);
    }

    Point(double x,double y)
    {
        setPoint(x, y);
    }
    
    void setPoint(double x,double y,double theta)
    {
        this.x=x;
        this.y=y;
        this.theta=theta;
    }

    void setPoint(double x,double y)
    {
        this.x=x;
        this.y=y;
    }
}
