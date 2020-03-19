package org.firstinspires.ftc.teamcode.Skystone.Modules;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Skystone.HardwareCollection;
import org.firstinspires.ftc.teamcode.Skystone.Robot;

public class DriveModule {

    public double xMovement;
    public double yMovement;
    public double turnMovement;
    public boolean isSlowDrive;

    private double fLeftPower;
    private double fRightPower;
    private double bLeftPower;
    private double bRightPower;

    private HardwareCollection hardwareCollection;
    private Robot robot;

    public StringBuilder driveData;

    public DriveModule(){
        xMovement = 0.0;
        yMovement = 0.0;
        turnMovement = 0.0;
        isSlowDrive = false;

        driveData = new StringBuilder();
        driveData.append("xMovement yMovement turnMovement isSlowDrive fLeftPower fRightPower bLeftPower bRightPower");
    }

    public synchronized void update(Robot robot, HardwareCollection hardwareCollection){

        this.hardwareCollection = hardwareCollection;
        this.robot = robot;

        if (robot.isDebug){
            driveData.append(xMovement);
            driveData.append("\n");
            driveData.append(yMovement);
            driveData.append("\n");
            driveData.append(turnMovement);
            driveData.append("\n");
            driveData.append(isSlowDrive);
            driveData.append("\n");
            driveData.append(fLeftPower);
            driveData.append("\n");
            driveData.append(fRightPower);
            driveData.append("\n");
            driveData.append(bLeftPower);
            driveData.append("\n");
            driveData.append(bRightPower);
            driveData.append("\n");
        }

        // this is applyMove

        // convert movements to motor powers
        fLeftPower = (yMovement * 1.414 + turnMovement + xMovement);
        fRightPower = (-yMovement * 1.414 - turnMovement + xMovement);
        bLeftPower = (-yMovement * 1.414 + turnMovement + xMovement);
        bRightPower = (yMovement * 1.414 - turnMovement + xMovement);

        //scale all powers to below 1
        double maxPower = Math.abs(fLeftPower);
        if (Math.abs(bLeftPower) > maxPower) {
            maxPower = Math.abs(bLeftPower);
        }
        if (Math.abs(bRightPower) > maxPower) {
            maxPower = Math.abs(bRightPower);
        }
        if (Math.abs(fRightPower) > maxPower) {
            maxPower = Math.abs(fRightPower);
        }
        double scaleDownAmount = 1.0;
        if (maxPower > 1.0) {
            scaleDownAmount = 1.0 / maxPower;
        }
        fLeftPower *= scaleDownAmount;
        fRightPower *= scaleDownAmount;
        bLeftPower *= scaleDownAmount;
        bRightPower *= scaleDownAmount;

        if (isSlowDrive){
            fLeftPower *= 0.3;
            fRightPower *= 0.3;
            bLeftPower *= 0.3;
            bRightPower *= 0.3;
        }

        setMotorPowers(fLeftPower, fRightPower, bLeftPower, bRightPower);
    }

    public void brakeRobot(){
        driveMotorsBrakeOnZero();
        setMotorPowers(0, 0, 0, 0);

        // TODO make safer
        robot.linearOpMode.sleep(250);
    }

    public void driveMotorsBrakeOnZero() {
        //sets drive motors to brake mode
        hardwareCollection.fLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hardwareCollection.bLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hardwareCollection.fRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hardwareCollection.fLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setDrivetrainMotorModes(DcMotor.RunMode runMode) {
        hardwareCollection.fLeft.setMode(runMode);
        hardwareCollection.fRight.setMode(runMode);
        hardwareCollection.bLeft.setMode(runMode);
        hardwareCollection.bRight.setMode(runMode);
    }

    private void setMotorPowers(double fLpower, double fRpower, double bLpower, double bRpower) {
        hardwareCollection.fLeft.setPower(fLpower);
        hardwareCollection.fRight.setPower(fRpower);
        hardwareCollection.bLeft.setPower(bLpower);
        hardwareCollection.bRight.setPower(bRpower);
    }
}
