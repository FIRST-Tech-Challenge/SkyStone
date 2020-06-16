/**
 * Copyright (c) 2020, All Rights Reserved
 *
 * 'Drivetrain' controls the motion motors of 'Trobot'. It contains the drive motor variables as
 * well as containing many utility methods.
 *
 * Key features include:
 * - driving with power inputs
 * - strafing
 * - auto-drive using encoders
 * - speed reduction
 * - checking motor statuses
 *
 * 'Drivetrain' remains relatively constant through each season since a different game theme doesn't
 * affect a robot's basic movement abilites.
 *
 * Written by Timothy (Tikki) Cui
 */

package org.firstinspires.ftc.teamcode.SourceFiles;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Drivetrain {
    private HardwareMap hardwareMap;

    private DcMotor frontLeftDrive;
    private DcMotor frontRightDrive;
    private DcMotor rearLeftDrive;
    private DcMotor rearRightDrive;

    // start with full speed
    private boolean isSpeedReduced = false;
    private String speedStatus = "Normal";

    // enum variables
    public final int LEFT = -1;
    public final int RIGHT = 1;
    public final int FORWARD = 1;
    public final int BACKWARD = -1;

    public Drivetrain(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;

        frontLeftDrive = hardwareMap.get(DcMotor.class, "front left");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front right");
        rearLeftDrive = hardwareMap.get(DcMotor.class, "rear left");
        rearRightDrive = hardwareMap.get(DcMotor.class, "rear right");

        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        rearLeftDrive.setDirection(DcMotor.Direction.REVERSE);

        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    
    // Accessor/Mutator
    public HardwareMap getHardwareMap() {return hardwareMap;}
    public DcMotor getFrontLeftDrive() {return frontLeftDrive;}
    public DcMotor getFrontRightDrive() {return frontRightDrive;}
    public DcMotor getRearLeftDrive() {return rearLeftDrive;}
    public DcMotor getRearRightDrive() {return rearRightDrive;}
    public boolean isSpeedReduced() {return isSpeedReduced;}
    public String getSpeedStatus() {return speedStatus;}

    public void setHardwareMap(HardwareMap hardwareMap) {this.hardwareMap = hardwareMap;}
    public void setFrontLeftDrive(DcMotor frontLeftDrive) {this.frontLeftDrive = frontLeftDrive;}
    public void setFrontRightDrive(DcMotor frontRightDrive) {this.frontRightDrive = frontRightDrive;}
    public void setRearLeftDrive(DcMotor rearLeftDrive) {this.rearLeftDrive = rearLeftDrive;}
    public void setRearRightDrive(DcMotor rearRightDrive) {this.rearRightDrive = rearRightDrive;}
    public void setSpeedReduced(boolean speedReduced) {isSpeedReduced = speedReduced;}
    public void setSpeedStatus(String speedStatus) {this.speedStatus = speedStatus;}

    // Utility
    public void drive(double power) {
        if (!isSpeedReduced) {
            frontLeftDrive.setPower(power);
            frontRightDrive.setPower(power);
            rearLeftDrive.setPower(power);
            rearRightDrive.setPower(power);
        } else {
            frontLeftDrive.setPower(power * 0.65);
            frontRightDrive.setPower(power * 0.65);
            rearLeftDrive.setPower(power * 0.65);
            rearRightDrive.setPower(power * 0.65);
        }
    }

    public void drive(double leftPower, double rightPower) {
        if (!isSpeedReduced) {
            frontLeftDrive.setPower(leftPower);
            frontRightDrive.setPower(rightPower);
            rearLeftDrive.setPower(leftPower);
            rearRightDrive.setPower(rightPower);
        } else {
            frontLeftDrive.setPower(leftPower * 0.65);
            frontRightDrive.setPower(rightPower * 0.65);
            rearLeftDrive.setPower(leftPower * 0.65);
            rearRightDrive.setPower(rightPower * 0.65);
        }
    }

    public void turn(int direction, double degrees) {
        if (direction == LEFT) {
            // TODO: implement function body
        } else if (direction == RIGHT) {
            // TODO: implement function body
        }
    }

    public void stop() {
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        rearLeftDrive.setPower(0);
        rearRightDrive.setPower(0);
    }

    public void switchSpeed() {
        isSpeedReduced = !isSpeedReduced;

        if (isSpeedReduced) {
            speedStatus = "Reduced";
        } else {
            speedStatus = "Normal";
        }
    }

    public void strafe(int direction, double power) {
        if (direction == LEFT) {
            frontLeftDrive.setPower(power);
            frontRightDrive.setPower(-power);
            rearLeftDrive.setPower(-power);
            rearRightDrive.setPower(power);
        } else if (direction == RIGHT) {
            frontLeftDrive.setPower(-power);
            frontRightDrive.setPower(power);
            rearLeftDrive.setPower(power);
            rearRightDrive.setPower(-power);
        }
    }

    public void autoDriveTime(double power, double time) {
        frontLeftDrive.setPower(power);
        frontRightDrive.setPower(power);
        rearLeftDrive.setPower(power);
        rearRightDrive.setPower(power);

        try {
            Thread.sleep((long)(time));
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

//    public void autoDriveDistance(double power, double distance) {
//        if (distance > 0) {
//            frontLeftDrive.setPower(power);
//            frontRightDrive.setPower(power);
//            rearLeftDrive.setPower(power);
//            rearRightDrive.setPower(power);
//        } else if (distance < 0) {
//            frontLeftDrive.setPower(-power);
//            frontRightDrive.setPower(-power);
//            rearLeftDrive.setPower(-power);
//            rearRightDrive.setPower(-power);
//        }
//
//        // Source code for 'sleep'
//        try {
//            Thread.sleep(Math.abs((int)((distance / (72.5 * power)) * 1000)));
//        } catch (InterruptedException e) {
//            Thread.currentThread().interrupt();
//        }
//    }

    public void autoDriveEncoder(double power, double distance) {
        // TODO: Confirm measurements
        double threadsPerCentimeter = ((1120 * 2) / (10 * Math.PI));

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftDrive.setTargetPosition((int)(distance * threadsPerCentimeter));
        frontRightDrive.setTargetPosition((int)(distance * threadsPerCentimeter));
        rearLeftDrive.setTargetPosition((int)(distance * threadsPerCentimeter));
        rearRightDrive.setTargetPosition((int)(distance * threadsPerCentimeter));

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rearLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rearRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeftDrive.setPower(power);
        frontRightDrive.setPower(power);
        rearLeftDrive.setPower(power);
        rearRightDrive.setPower(power);

        while (frontLeftDrive.isBusy() || rearRightDrive.isBusy()) {}

        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        rearLeftDrive.setPower(0);
        rearRightDrive.setPower(0);
    }

    public double getPosition() {
        return (frontLeftDrive.getCurrentPosition() + frontRightDrive.getCurrentPosition()
                + rearLeftDrive.getCurrentPosition() + rearRightDrive.getCurrentPosition()) / 4.0;
    }

    public void resetEncoder() {
        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public boolean isBusy() {
        return frontLeftDrive.isBusy() || frontRightDrive.isBusy() || rearLeftDrive.isBusy() || rearRightDrive.isBusy();
    }
}