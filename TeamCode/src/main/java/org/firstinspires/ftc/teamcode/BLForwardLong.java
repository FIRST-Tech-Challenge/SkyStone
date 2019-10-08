package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by maryjaneb  on 11/13/2016.
 *
 * nerverest ticks
 * 60 1680
 * 40 1120
 * 20 560
 */
@Autonomous(name= "BLForwardLong", group="Sky autonomous")
//@Disabled
public class BLForwardLong extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor backLeft     = null; //rear left
    private DcMotor backRight    = null; //rear right
    private DcMotor frontLeft    = null; //front left
    private DcMotor frontRight   = null; //front right



    //power from -1 to 1 range. positive is forward. negative is backward.
    //move distance in inches
    public void moveDistance(double power, int time) {
            backLeft.setPower(power);
            backRight.setPower(power);
            frontLeft.setPower(power);
            frontRight.setPower(power);
            sleep(time);
            backLeft.setPower(0);
            backRight.setPower(0);
            frontLeft.setPower(0);
            frontRight.setPower(0);
            sleep(500);

    }

    //positive is right, negative is left. inches
    public void strafe(double power, int time) {
        backLeft.setPower(-power);
        backRight.setPower(power);
        frontLeft.setPower(power);
        frontRight.setPower(-power);
        sleep(time);
        backLeft.setPower(0);
        backRight.setPower(0);
        frontLeft.setPower(0);
        frontRight.setPower(0);
        sleep(500);
    }

    //positive is right, negative is left
    public void rotate(double power, int time) {
        backRight.setPower(-power);
        frontRight.setPower(-power);
        backLeft.setPower(power);
        frontLeft.setPower(power);
        sleep(time);
        backLeft.setPower(0);
        backRight.setPower(0);
        frontLeft.setPower(0);
        frontRight.setPower(0);
        sleep(500);
    }



    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Running");
        telemetry.update();

        backLeft        = hardwareMap.dcMotor.get("left_drive");
        backRight       = hardwareMap.dcMotor.get("right_drive");
        frontLeft       = hardwareMap.dcMotor.get("front_left");
        frontRight      = hardwareMap.dcMotor.get("front_right");

        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);

        waitForStart();
        runtime.reset();

        //call movement functions
        sleep(1200);
        strafe(-0.4, 200);
        moveDistance(0.4, 1500);

    }
}