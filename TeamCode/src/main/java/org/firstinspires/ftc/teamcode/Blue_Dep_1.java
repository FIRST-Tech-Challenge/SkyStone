package org.firstinspires.ftc.teamcode;

import static java.lang.Math.PI;
import static java.lang.Math.sqrt;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous

public class Blue_Dep_1 extends LinearOpMode {

    private static final double WHEEL_DIAMETER = (double) 4;

    private static final double DRIVE_TRAIN_LENGTH = 11.5;
    private static final double DRIVE_TRAIN_WIDTH = 15.5;
    private static final double DRIVE_TRAIN_DIAGONAL = sqrt(
            DRIVE_TRAIN_WIDTH * DRIVE_TRAIN_WIDTH + DRIVE_TRAIN_LENGTH * DRIVE_TRAIN_LENGTH);

    private static final int ENCODER_TICKS = 1120;

    private static final double GEAR_RATIO = (double) 2 / 1;

    private static final double DRIVE_ADJUST = 1.1776595745;
    private static final double STRAFE_ADJUST = 1.4545454525;
    private static final double ROTATE_ADJUST = 1.75;

    private static double POWER = 0.7;

    private DcMotor fL = null;
    private DcMotor fR = null;
    private DcMotor bL = null;
    private DcMotor bR = null;

    @Override
    public void runOpMode() throws InterruptedException {
        fL = hardwareMap.dcMotor.get("fL");
        fR = hardwareMap.dcMotor.get("fR");
        bL = hardwareMap.dcMotor.get("bL");
        bR = hardwareMap.dcMotor.get("bR");

        fL.setDirection(DcMotor.Direction.REVERSE);
        fL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bL.setDirection(DcMotor.Direction.REVERSE);
        bL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        forward(POWER, 61);
        strafeRight(POWER, 11);
        backward(POWER, 23);
        strafeRight(POWER, 9);
        backward(POWER, 11);
        strafeLeft(POWER, 50);
        strafeRight(POWER, 51);
        forward(POWER, 17);
        strafeRight(POWER, 8);
        backward(POWER, 6);
        strafeRight(POWER, 16);
        backward(POWER, 12);
        strafeLeft(POWER, 75);
    }

    private void forward(double power, double distance) {
        int ticks = distanceToTicks(distance * DRIVE_ADJUST);
        driveTrain(power, ticks, ticks, ticks, ticks);
    }

    private void backward(double power, double distance) {
        int ticks = distanceToTicks(distance * DRIVE_ADJUST);
        driveTrain(power, -ticks, -ticks, -ticks, -ticks);
    }

    private void strafeLeft(double power, double distance) {
        int ticks = distanceToTicks(distance * STRAFE_ADJUST);
        driveTrain(power, -ticks, ticks, ticks, -ticks);
    }

    private void strafeRight(double power, double distance) {
        int ticks = distanceToTicks(distance * STRAFE_ADJUST);
        driveTrain(power, ticks, -ticks, -ticks, ticks);
    }

    private void rotateLeft(double power, double degree) {
        int ticks = distanceToTicks((degree / 360) * (DRIVE_TRAIN_DIAGONAL * PI) * ROTATE_ADJUST);
        driveTrain(power, -ticks, ticks, -ticks, ticks);
    }

    private void rotateRight(double power, double degree) {
        int ticks = distanceToTicks((degree / 360) * (DRIVE_TRAIN_DIAGONAL * PI) * ROTATE_ADJUST);
        driveTrain(power, ticks, -ticks, ticks, -ticks);
    }

    private void driveTrain(double power, int fLTicks, int fRTicks, int bLTicks, int bRTicks) {

        fL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        fL.setTargetPosition(fLTicks);
        fR.setTargetPosition(fRTicks);
        bL.setTargetPosition(bLTicks);
        bR.setTargetPosition(bRTicks);

        //Sign of power does not matter with RunToPosition
        fL.setPower(power);
        fR.setPower(power);
        bL.setPower(power);
        bR.setPower(power);

        while (fL.isBusy() && fR.isBusy() && bL.isBusy() && bR.isBusy()) {
        }

        //Stop and return to normal mode
        fL.setPower(0);
        fR.setPower(0);
        bL.setPower(0);
        bR.setPower(0);
    }

    private int distanceToTicks(double distance) {
        return (int) ((distance / (WHEEL_DIAMETER * Math.PI) * ENCODER_TICKS) / GEAR_RATIO + 0.5);
    }

}
