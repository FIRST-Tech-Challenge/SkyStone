package org.firstinspires.ftc.teamcode;

import static java.lang.Math.PI;
import static java.lang.Math.sqrt;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

abstract class MecanumAutoCentral extends LinearOpMode {

    private static final double WHEEL_DIAMETER = (double) 4;
    private static final double DRIVE_TRAIN_LENGTH = 11.5;
    private static final double DRIVE_TRAIN_WIDTH = 15.5;
    private static final double DRIVE_TRAIN_DIAGONAL = sqrt(
            DRIVE_TRAIN_WIDTH * DRIVE_TRAIN_WIDTH + DRIVE_TRAIN_LENGTH * DRIVE_TRAIN_LENGTH);
    private static final int ENCODER_TICKS = 1120;

    private static final double GEAR_RATIO = (double) 5/2;

    private static final double DRIVE_ADJUST = 1.1776595745;
    private static final double STRAFE_ADJUST = 1.4545454525;
    private static final double ROTATE_ADJUST = 1.75;

    DcMotor fL = null;
    DcMotor fR = null;
    DcMotor bL = null;
    DcMotor bR = null;

    void drive(double power, double distance) {
        int ticks = distanceToTicks(distance * DRIVE_ADJUST);
        driveTrain(power, ticks, ticks, ticks, ticks);
    }

    void strafeLeft(double power, double distance) {
        int ticks = distanceToTicks(distance * STRAFE_ADJUST);
        driveTrain(power, -ticks, ticks, ticks, -ticks);
    }

    void rotate(double power, double degree) {
        int ticks = distanceToTicks((degree / 360) * (DRIVE_TRAIN_DIAGONAL * PI) * ROTATE_ADJUST);
        driveTrain(power, -ticks, ticks, -ticks, ticks);
    }

    private void driveTrain(double power, int fLTicks, int fRTicks, int bLTicks, int bRTicks) {

        fL.setTargetPosition(fL.getCurrentPosition() + fLTicks);
        fR.setTargetPosition(fR.getCurrentPosition() + fRTicks);
        bL.setTargetPosition(bL.getCurrentPosition() + bLTicks);
        bR.setTargetPosition(bR.getCurrentPosition() + bRTicks);

        fL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //Sign of power does not matter with RunToPosition
        setPower(power);

        while (fL.isBusy() && fR.isBusy() && bL.isBusy() && bR.isBusy()) {
        }

        //Stop and return to normal mode
        setPower(0);
        fL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void setPower(double power) {
        fL.setPower(power);
        fR.setPower(power);
        bL.setPower(power);
        bR.setPower(power);
    }

    private int distanceToTicks(double distance) {
        return (int) ((distance / (WHEEL_DIAMETER * Math.PI) * ENCODER_TICKS) / GEAR_RATIO + 0.5);
    }

}
