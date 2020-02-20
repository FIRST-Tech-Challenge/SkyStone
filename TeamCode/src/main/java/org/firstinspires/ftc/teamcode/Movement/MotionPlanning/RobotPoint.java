package org.firstinspires.ftc.teamcode.Movement.MotionPlanning;

public class RobotPoint {

    public double x, y, heading;
    public double speed, radius;

    //Action variables
    public double hookPosition = 0, clampPosition = 0;
    public double intakePower;
    public String outtakeClampState = "Receive";
    public String outtakeFlipState = "Receive";
    public int liftPosition = 0;

    public RobotPoint(double x, double y, double heading, double speed, double radius){

        this.x = x;
        this.y = y;
        this.heading = heading;
        this.speed = speed;
        this.radius = radius;

    }

    public void setIntakeActions(double intakePower, String outtakeClampState, String outtakeFlipState, int liftPosition){
        this.intakePower = intakePower;
        this.outtakeClampState = outtakeClampState;
        this.outtakeFlipState = outtakeFlipState;
        this.liftPosition = liftPosition;

    }

    public void setHookActions(double hookPosition, double clampPosition){
        this.hookPosition = hookPosition;
        this.clampPosition = clampPosition;
    }

}
