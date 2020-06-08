package org.firstinspires.ftc.teamcode.SourceFiles;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Drivetrain {

    private HardwareMap hardwareMap;

    public DcMotor frontLeftDrive;
    public DcMotor frontRightDrive;
    public DcMotor rearLeftDrive;
    public DcMotor rearRightDrive;

    // start with full speed
    public boolean isSpeedReduced = false;
    public String speedStatus = "Pending";

    public Drivetrain(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;

        frontLeftDrive = hardwareMap.get(DcMotor.class, "front left");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front right");
        rearLeftDrive = hardwareMap.get(DcMotor.class, "rear left");
        rearRightDrive = hardwareMap.get(DcMotor.class, "rear right");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        rearLeftDrive.setDirection(DcMotor.Direction.REVERSE);
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

    public void autoDrive(double speed, double distance) {
        if (speed > 1) {
            speed = 1;
        }

        if (distance > 0) {
            frontLeftDrive.setPower(speed);
            frontRightDrive.setPower(speed);
            rearLeftDrive.setPower(speed);
            rearRightDrive.setPower(speed);
        } else if (distance < 0) {
            frontLeftDrive.setPower(-speed);
            frontRightDrive.setPower(-speed);
            rearLeftDrive.setPower(-speed);
            rearRightDrive.setPower(-speed);
        }

        //time = Math.abs((int)((distance/(72.5*speed))*1000));
    }

    public void turn(String direction, double degrees) {
        if (direction.toUpperCase().equals("LEFT")) {
            // TODO: implement function body
        } else if (direction.toUpperCase().equals("RIGHT")) {
            // TODO: implement function body
        }
    }

    public void strafe(String direction) {
        if (direction.toUpperCase().equals("LEFT")) {
            frontLeftDrive.setPower(1);
            frontRightDrive.setPower(-1);
            rearLeftDrive.setPower(-1);
            rearRightDrive.setPower(1);
        } else if (direction.toUpperCase().equals("RIGHT")) {
            frontLeftDrive.setPower(-1);
            frontRightDrive.setPower(1);
            rearLeftDrive.setPower(1);
            rearRightDrive.setPower(-1);
        }
    }

    public void stop() {
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        rearLeftDrive.setPower(0);
        rearRightDrive.setPower(0);
    }
}