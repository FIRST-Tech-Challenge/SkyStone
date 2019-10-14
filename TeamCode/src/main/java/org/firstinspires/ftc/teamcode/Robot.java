package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

// THIS IS NOT AN OPMODE

public class Robot {

    // Motors
    public DcMotor rearLeft;
    public DcMotor rearRight;
    public DcMotor frontLeft;
    public DcMotor frontRight;

    // Constants
    public double ANDYMARK_TICKS_PER_REV = 1120; // ticks / rev
    public double WHEEL_DIAMETER = 4;
    public double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI; // in / rev
    public double TICKS_PER_INCH = ANDYMARK_TICKS_PER_REV / WHEEL_CIRCUMFERENCE; // ticks / in

    HardwareMap hwMap = null;

    public Robot () {
        // Constructor
    }

    public void init (HardwareMap ahwMap) {
        /* Initializes the robot */

        hwMap = ahwMap;

        // Drive Motor instantiation
        this.rearLeft = hwMap.dcMotor.get("rearLeft");
        this.rearRight = hwMap.dcMotor.get("rearRight");

        this.frontLeft = hwMap.dcMotor.get("frontLeft");
        this.frontRight = hwMap.dcMotor.get("frontRight");

        // Drive Motor Direction
        this.rearLeft.setDirection(DcMotor.Direction.FORWARD);
        this.frontLeft.setDirection(DcMotor.Direction.FORWARD);
        this.rearRight.setDirection(DcMotor.Direction.REVERSE);
        this.frontRight.setDirection(DcMotor.Direction.REVERSE);

        // set motor powers to 0 so they don't cause problems
        this.stopDrive();
    }

    public void setDrivePower(double power) {
        /* sets all drive motors to a certain power */
        this.rearLeft.setPower(power);
        this.frontLeft.setPower(power);
        this.rearRight.setPower(power);
        this.frontRight.setPower(power);
    }

    public void setDriveMode(DcMotor.RunMode runMode) {
        /* sets all drive motors to a certain mode */
        this.rearLeft.setMode(runMode);
        this.frontLeft.setMode(runMode);
        this.rearRight.setMode(runMode);
        this.frontRight.setMode(runMode);
    }

    public void setDriveTargetPos(int targetPosition) {
        /* sets all drive motors to a target position */
        this.rearLeft.setTargetPosition(targetPosition);
        this.frontLeft.setTargetPosition(targetPosition);
        this.rearRight.setTargetPosition(targetPosition);
        this.frontRight.setTargetPosition(targetPosition);
    }

    public void stopDrive() {
        /* stops all drive motors */
        this.setDrivePower(0);
    }

    public void driveForwardDistance(double distance, double power) {
        //* drives forward a certain distance(in) using encoders *//*

        // calculate ticks
        long NUM_TICKS_LONG = StrictMath.round(this.TICKS_PER_INCH * distance);
        int NUM_TICKS = (int) NUM_TICKS_LONG;

        // reset encoders
        this.setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // set target position
        this.setDriveTargetPos(NUM_TICKS);

        // Set to RUN_TO_POSITION mode
        this.setDriveMode(DcMotor.RunMode.RUN_TO_POSITION);

        // set drive power
        this.setDrivePower(power);

        while (this.rearLeft.isBusy() && this.frontLeft.isBusy() && this.rearRight.isBusy() && this.frontRight.isBusy()) {
            // wait until target position is reached
        }



        // stop driving
        this.stopDrive();

        // set mode back to normal
        this.setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }



    public void setStrafe(double power) {
        /* strafes at certain power
        positive power goes to the right
        negative power goes to the left */
        this.rearLeft.setPower(-power);
        this.frontRight.setPower(-power);

        this.frontLeft.setPower(power);
        this.rearRight.setPower(power);
    }

    public void strafeTime(double power, long milliseconds) throws InterruptedException {
        /* strafes for a certain amount of milliseconds */
        this.setStrafe(power);
        wait(milliseconds);
        this.stopDrive();
    }
}
