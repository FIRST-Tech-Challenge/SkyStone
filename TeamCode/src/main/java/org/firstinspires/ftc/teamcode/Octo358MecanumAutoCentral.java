package org.firstinspires.ftc.teamcode;

import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.random;
import static java.lang.Math.sqrt;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

abstract public class Octo358MecanumAutoCentral extends LinearOpMode {

    private static final double WHEEL_DIAMETER = (double) 4;
    private static final double DRIVE_TRAIN_LENGTH = 11.811;
    private static final double DRIVE_TRAIN_WIDTH = 15.748;
    private static final double DRIVE_TRAIN_DIAGONAL = sqrt(
            DRIVE_TRAIN_WIDTH * DRIVE_TRAIN_WIDTH + DRIVE_TRAIN_LENGTH * DRIVE_TRAIN_LENGTH);
    private static final int ENCODER_TICKS = 1120;

    DcMotor fL = null;
    DcMotor fR = null;
    DcMotor bL = null;
    DcMotor bR = null;

    static final double POWER = 0.3;

    double gameField = 12 * 12;
    double quarryLength = 48.5;
    double sideWallToQuarry = 47;

    double boxSideLength = 22.75;
    double stoneLength = 8;
    double stoneWidth = 4;
    double stoneBoxHeight = 4;
    double stoneKnobHeight = 1;
    double chassisLength = 16.5;
    double chassisWidth = 18;
    double chassisDiff = abs(chassisWidth - chassisLength);

    void drive(double power, double distance) {
        int ticks = distanceToTicks(distance);
        driveTrain(power, ticks, ticks, ticks, ticks);
    }

    void strafeLeft(double power, double distance) {
        int ticks = distanceToTicks(distance);
        driveTrain(power, -ticks, ticks, ticks, -ticks);
    }

    void rotate(double power, double degree) {
        int ticks = distanceToTicks((degree / 360) * (DRIVE_TRAIN_DIAGONAL * PI));
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
        return (int) ((distance / (WHEEL_DIAMETER * Math.PI) * ENCODER_TICKS) + 0.5);
    }

}
