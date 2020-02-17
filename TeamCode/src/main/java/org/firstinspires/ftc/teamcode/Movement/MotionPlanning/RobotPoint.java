package org.firstinspires.ftc.teamcode.Movement.MotionPlanning;

public class RobotPoint {

    public double x, y, heading;
    public double hookPosition, clampPosition;
    public double intakePower;

    public RobotPoint(double x, double y, double heading){

        this.x = x;
        this.y = y;
        this.heading = heading;

    }

    public void setActions(double hookPosition, double clampPosition, double intakePower){

        this.hookPosition = hookPosition;
        this.clampPosition = clampPosition;
        this.intakePower = intakePower;

    }


}
