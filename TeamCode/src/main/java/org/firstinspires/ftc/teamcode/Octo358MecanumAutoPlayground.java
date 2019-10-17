package org.firstinspires.ftc.teamcode;

import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.max;
import static java.lang.Math.sqrt;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

abstract public class Octo358MecanumAutoPlayground extends LinearOpMode {

    private static final double WHEEL_DIAMETER = (double) 4;
    private static final double DRIVE_TRAIN_LENGTH = 11.811;
    private static final double DRIVE_TRAIN_WIDTH = 15.748;
    private static final double DRIVE_TRAIN_DIAGONAL = sqrt(
            DRIVE_TRAIN_WIDTH * DRIVE_TRAIN_WIDTH + DRIVE_TRAIN_LENGTH * DRIVE_TRAIN_LENGTH);
    private static final int ENCODER_TICKS = 1120;

    private static final double DRIVE_ADJUST = 0.4710638298;
    private static final double STRAFE_ADJUST = 0.5818181818;
    private static final double ROTATE_ADJUST = 0.7;
    private double SHIFT_AT = 500;

    DcMotor fL = null;
    DcMotor fR = null;
    DcMotor bL = null;
    DcMotor bR = null;

    static final double POWER = 0.3;

    /*
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
     */

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

        if (abs(fLTicks) < 1000) {
            SHIFT_AT = abs(fLTicks) * 0.3;
        }

        int fLOriginalPosition = fL.getCurrentPosition();
        int fROriginalPosition = fR.getCurrentPosition();
        int bLOriginalPosition = bL.getCurrentPosition();
        int bROriginalPosition = bR.getCurrentPosition();

        fL.setTargetPosition(fL.getCurrentPosition() + fLTicks);
        fR.setTargetPosition(fR.getCurrentPosition() + fRTicks);
        bL.setTargetPosition(bL.getCurrentPosition() + bLTicks);
        bR.setTargetPosition(bR.getCurrentPosition() + bRTicks);

        fL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        setPower(0.001);

        while (fL.isBusy() && fR.isBusy() && bL.isBusy() && bR.isBusy()) {
            if (abs(fL.getCurrentPosition() - fLOriginalPosition) < SHIFT_AT ||
                abs(fR.getCurrentPosition() - fROriginalPosition) < SHIFT_AT ||
                abs(bL.getCurrentPosition() - bLOriginalPosition) < SHIFT_AT ||
                abs(bR.getCurrentPosition() - bROriginalPosition) < SHIFT_AT) {
                setPower(((maxBeginningProgress(fLOriginalPosition, fROriginalPosition, bLOriginalPosition, bROriginalPosition) + 1) / SHIFT_AT) * power);
            }
            if (abs(fLOriginalPosition + fLTicks - fL.getCurrentPosition()) < SHIFT_AT ||
                abs(fROriginalPosition + fRTicks - fR.getCurrentPosition()) < SHIFT_AT ||
                abs(bLOriginalPosition + bLTicks - bL.getCurrentPosition()) < SHIFT_AT ||
                abs(bROriginalPosition + bRTicks - bR.getCurrentPosition()) < SHIFT_AT) {
                setPower(((maxEndProgress(fLOriginalPosition + fLTicks, fROriginalPosition + fRTicks, bLOriginalPosition + bLTicks, bROriginalPosition + bRTicks) + 1) / SHIFT_AT) * power);
            }
            else {
                setPower(power);
            }
        }

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

    private double maxBeginningProgress(int fLO, int fRO, int bLO, int bRO) {
        double fLP = abs(fL.getCurrentPosition() - fLO);
        double fRP = abs(fR.getCurrentPosition() - fRO);
        double bLP = abs(bL.getCurrentPosition() - bLO);
        double bRP = abs(bR.getCurrentPosition() - bRO);

        return max(max(fLP, fRP), max(bLP, bRP));
    }

    private double maxEndProgress(int fLO, int fRO, int bLO, int bRO) {
        double fLP = abs(fLO - fL.getCurrentPosition());
        double fRP = abs(fRO - fR.getCurrentPosition());
        double bLP = abs(bLO - bL.getCurrentPosition());
        double bRP = abs(bRO - bR.getCurrentPosition());

        return max(max(fLP, fRP), max(bLP, bRP));
    }

}
