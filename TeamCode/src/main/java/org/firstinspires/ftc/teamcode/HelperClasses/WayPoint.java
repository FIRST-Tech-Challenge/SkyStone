package org.firstinspires.ftc.teamcode.HelperClasses;

public class WayPoint {
    public WayPoint(double X, double Y, double angle, double speed, boolean passThrough){
        this.x = X;
        this.y = Y;
        this.angle = angle;
        this.speed = speed;
        this.passThrough = passThrough;
    }
    public double x;
    public double y;
    public double angle;
    public double speed;
    public boolean passThrough;
}
