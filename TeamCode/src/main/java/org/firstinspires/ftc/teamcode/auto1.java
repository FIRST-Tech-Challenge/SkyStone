package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/*
Autonomous ONE will do simple operation regarding the grabbing of the score depot and parking from the depot side
as opposed to the human player side.
 */

@Autonomous(name = "auto1", group = "autonomous")
public class auto1 extends LinearOpMode {

    public DcMotor TL;
    public DcMotor TR;
    public DcMotor BL;
    public DcMotor BR;

    public Servo hooker;\

    public DistanceSensor distanceSensor;
    
    private double TURNSPEED = 0.4;
    private double STRAFESPEED = 0.7;
    private double STALKSPEED = 0.3;
    double triggerDist = 10.5;

    @Override
    public void runOpMode() throws InterruptedException {

        TL = hardwareMap.get(DcMotor.class, "TL");
        TR = hardwareMap.get(DcMotor.class, "TR");
        BL = hardwareMap.get(DcMotor.class, "BL");
        BR = hardwareMap.get(DcMotor.class, "BR");

        TL.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);

        hooker = hardwareMap.get(Servo.class, "hook");

        distanceSensor = hardwareMap.get(DistanceSensor.class, "dist");

        ElapsedTime runtime = new ElapsedTime();
        ElapsedTime backgroundTime = new ElapsedTime();

        waitForStart();

        //Go forward until waffle is within proximity
        driveForward();

        while(!tripWireActive(triggerDist)){
            telemetry.addData("Closing in on target", !tripWireActive(triggerDist));
            telemetry.update();
            sleep(3000);
            }

        dropDL();

        sleep(2000);

        driveBackward();

        while()
        }





    }

    public boolean tripWireActive(double triggerDist) {
        if (distanceSensor.getDistance(DistanceUnit.CM)<triggerDist){
            return true;
        }
        else {
            return false;
        }
    }

    public void rest() {
        TL.setPower(0);
        TR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);
    }

    public void dropDL() {
        hooker.setPosition(0.1);
    }

    public void raiseDL() {
        hooker.setPosition(0.9);
    }

    public void driveForward(){
        TL.setPower(STALKSPEED);
        TR.setPower(STALKSPEED);
        BL.setPower(STALKSPEED);
        BR.setPower(STALKSPEED);
    }

    public void driveBackward() {
        TL.setPower(-STALKSPEED);
        BL.setPower(-STALKSPEED);
        TR.setPower(-STALKSPEED);
        BR.setPower(-STALKSPEED);
    }

    public void strafeLeft() {
        TL.setPower(-STRAFESPEED);
        BL.setPower(STRAFESPEED);
        BR.setPower(STRAFESPEED);
        TR.setPower(-STRAFESPEED);
    }

    public void strafeRight() {
        TL.setPower(STRAFESPEED);
        BL.setPower(-STRAFESPEED);
        BR.setPower(-STRAFESPEED);
        TR.setPower(STRAFESPEED);
    }
    
    public void turnRight() {
        TL.setPower(STALKSPEED);
        TR.setPower(-STALKSPEED);
        BL.setPower(STALKSPEED);
        BR.setPower(-STALKSPEED);
    }

    public void turnLeft() {
        TL.setPower(-STALKSPEED);
        BL.setPower(-STALKSPEED);
        TR.setPower(STALKSPEED);
        BR.setPower(STALKSPEED);
    }

}
