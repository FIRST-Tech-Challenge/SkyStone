package org.firstinspires.ftc.teamcode;

/**
 * 2019.10.26
 * Created by Claire Zheng
 */

public class RobotPosition {
    private double x;
    private double y;
    private double heading;

    public RobotPosition(){

    }
    public RobotPosition(double x, double y, double heading){
        this.x = x;
        this.y = y;
        this.heading = heading;
    }

    public double getX() {
        return x;
    }

    public void setX(double x) {
        this.x = x;
    }

    public double getY() {
        return y;
    }

    public void setY(double y) {
        this.y = y;
    }

    public double getHeading() {
        return heading;
    }

    public void setHeading(double heading) {
        this.heading = heading;
    }

}
