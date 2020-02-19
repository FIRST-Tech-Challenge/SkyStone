package org.firstinspires.ftc.teamcode.Movement.MotionPlanning;

public class RobotPoint {

    public double x, y, heading;
    public boolean isLastPoint = false;
    public double speed, radius;
    public double hookPosition = 0, clampPosition = 0;
    public double intakePower = 0;

    public RobotPoint(double x, double y, double heading, double speed, double radius){

        this.x = x;
        this.y = y;
        this.heading = heading;
        this.speed = speed;
        this.radius = radius;

    }

    public void addActions(double hookPosition, double clampPosition, double intakePower){

        this.hookPosition = hookPosition;
        this.clampPosition = clampPosition;
        this.intakePower = intakePower;

    }

}
