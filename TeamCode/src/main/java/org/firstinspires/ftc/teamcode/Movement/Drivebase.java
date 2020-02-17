package org.firstinspires.ftc.teamcode.Movement;

import com.qualcomm.robotcore.hardware.DcMotor;

public class Drivebase {

    protected DcMotor rightFront, leftFront, leftBack, rightBack; //Motor Objects
    public double rf, lf, lb, rb; //Motor powers

    public void initialize(){}
    public void update(){}
    public void freeze(){}

    public void setRelativeVelocity(double velX, double velY, double velHeading){}


}
