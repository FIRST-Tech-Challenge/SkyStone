package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/*
Autonomous ONE will do simple operation regarding the grabbing of the score depot and parking from the depot side
as opposed to the human player side.
 */

@Autonomous(name = "auto1", group = "autonomous")
@Disabled
public class auto1 extends LinearOpMode {

    public DcMotor TL;
    public DcMotor TR;
    public DcMotor BL;
    public DcMotor BR;

    public Servo hooker;
    public Servo hooker1;

    public DistanceSensor distanceSensor;
    public BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    double triggerDist = 7.2;
    double globalAngle;
    private double STRAFESPEED = 1.0;
    private double STALKSPEED = 0.2;
    private double PULLOUT = 0.3;
    private double TURNSPEED = 0.5;

    @Override
    public void runOpMode() throws InterruptedException {

        TL = hardwareMap.get(DcMotor.class, "TL");
        TR = hardwareMap.get(DcMotor.class, "TR");
        BL = hardwareMap.get(DcMotor.class, "BL");
        BR = hardwareMap.get(DcMotor.class, "BR");

        TL.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);

        hooker = hardwareMap.get(Servo.class, "hook");
        hooker1 = hardwareMap.get(Servo.class, "hooke");
        hooker1 = hardwareMap.get(Servo.class, "hooke");

        distanceSensor = hardwareMap.get(DistanceSensor.class, "dist");

        ElapsedTime runtime = new ElapsedTime();
        ElapsedTime backgroundTime = new ElapsedTime();

        hooker.setPosition(0.9);
        hooker1.setPosition(0.9);


        waitForStart();

        /*
        ------------------------------------------------------------------
        Actual Process
        _-------------------------------------------------------------------
         */

        strafeRight();

        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1)) {
            telemetry.addData("Relocating Package", true);
            telemetry.update();
        }

        /*rest();

        sleep(3000);
        driveForward();
        while (opModeIsActive() && (!tripWireActive(triggerDist))) {
            telemetry.addData("Closing in on target", !tripWireActive(triggerDist));
            telemetry.update();
        }

        rest();
        dropDL();
        sleep(2000);

        driveBackward();
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 3)) {
            telemetry.addData("Relocating Package", true);
            telemetry.update();
        }
*/

    }

    /*
    -------------------------------------------------------------------------------
    IMU Corrector
    ----------------------------------------------------------------------------------


     */


    public boolean tripWireActive(double triggerDist) {
        if (distanceSensor.getDistance(DistanceUnit.CM) < triggerDist) {
            return true;
        } else {
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
        hooker1.setPosition(0.1);
    }

    public void raiseDL() {
        hooker.setPosition(0.9);
        hooker.setPosition(0.9);
    }

    public void driveBackward() {
        TL.setPower(PULLOUT);
        TR.setPower(PULLOUT);
        BL.setPower(PULLOUT);
        BR.setPower(PULLOUT);
    }

    public void driveForward() {
        TL.setPower(-STALKSPEED);
        BL.setPower(-STALKSPEED);
        TR.setPower(-STALKSPEED);
        BR.setPower(-STALKSPEED);
    }

    public void strafeRight() {
        TL.setPower(-STRAFESPEED);
        BL.setPower(STRAFESPEED);
        BR.setPower(-STRAFESPEED);
        TR.setPower(STRAFESPEED);
    }

    public void strafeLeft() {
        TL.setPower(STRAFESPEED);
        BL.setPower(-STRAFESPEED);
        BR.setPower(STRAFESPEED);
        TR.setPower(-STRAFESPEED);
    }

    public void turnRight() {
        TL.setPower(-TURNSPEED);
        TR.setPower(TURNSPEED);
        BL.setPower(-TURNSPEED);
        BR.setPower(TURNSPEED);
    }

    public void turnLeft() {
        TL.setPower(TURNSPEED);
        BL.setPower(TURNSPEED);
        TR.setPower(-TURNSPEED);
        BR.setPower(-TURNSPEED);
    }

}
