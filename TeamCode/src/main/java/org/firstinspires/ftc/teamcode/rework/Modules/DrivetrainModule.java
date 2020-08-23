package org.firstinspires.ftc.teamcode.rework.Modules;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.rework.ModuleTools.Module;
import org.firstinspires.ftc.teamcode.rework.Robot;


public class DrivetrainModule implements Module {
    private Robot robot;
    private boolean isOn;
    private boolean isFileDump;

    // States
    public double yMovement;
    public double xMovement;
    public double turnMovement;

    // Constants
    private final static double POWER_SCALE_FACTOR = 0.8;
    private final static double MECANUM_POWER_SCALE_FACTOR = 1.414;

    // Motors
    private DcMotor fLeft;
    private DcMotor fRight;
    private DcMotor bLeft;
    private DcMotor bRight;

    public DrivetrainModule(Robot robot, boolean isOn, boolean isFileDump) {
        this.robot = robot;
        this.isOn = isOn;
        this.isFileDump = isFileDump;
    }

    public void init() {
        fLeft = robot.getDcMotor("fLeft");
        fRight = robot.getDcMotor("fRight");
        bLeft = robot.getDcMotor("bLeft");
        bRight = robot.getDcMotor("bRight");

        fLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        fRight.setDirection(DcMotorSimple.Direction.FORWARD);
        bLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        bRight.setDirection(DcMotorSimple.Direction.FORWARD);

        setDrivetrainZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }


    // drivetrain update method applies the powers based on y x and turn movements
    public synchronized void update() {
        double fLPower = ((-yMovement) - turnMovement - xMovement * MECANUM_POWER_SCALE_FACTOR);
        double fRPower = ((-yMovement) + turnMovement + xMovement * MECANUM_POWER_SCALE_FACTOR);
        double bLPower = ((-yMovement) - turnMovement + xMovement * MECANUM_POWER_SCALE_FACTOR);
        double bRPower = ((-yMovement) + turnMovement - xMovement * MECANUM_POWER_SCALE_FACTOR);

        double maxPower = Math.abs(fLPower);
        if (Math.abs(fRPower) > maxPower) {
            maxPower = Math.abs(fRPower);
        }
        if (Math.abs(bLPower) > maxPower) {
            maxPower = Math.abs(bLPower);
        }
        if (Math.abs(bRPower) > maxPower) {
            maxPower = Math.abs(bRPower);
        }
        double scaleDown = 1.0;
        if (maxPower > 1.0) {
            scaleDown = 1.0 / maxPower;
        }
        fLPower *= scaleDown;
        fRPower *= scaleDown;
        bLPower *= scaleDown;
        bRPower *= scaleDown;


        fLPower *= POWER_SCALE_FACTOR;
        fRPower *= POWER_SCALE_FACTOR;
        bLPower *= POWER_SCALE_FACTOR;
        bRPower *= POWER_SCALE_FACTOR;

        setMotorPowers(fLPower, fRPower, bLPower, bRPower);
    }

    public void telemetry() {
        robot.telemetryDump.addHeader("---DRIVETRAIN---");
        robot.telemetryDump.addData("xMovement: ", xMovement);
        robot.telemetryDump.addData("yMovement: ", yMovement);
        robot.telemetryDump.addData("turnMovement: ", turnMovement);
    }

    public void fileDump(){

    }

    private void setMotorPowers(double fLPower, double fRPower, double bLPower, double bRPower) {
        fLeft.setPower(fLPower);
        fRight.setPower(fRPower);
        bLeft.setPower(bLPower);
        bRight.setPower(bRPower);
    }

    private void setDrivetrainZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        fLeft.setZeroPowerBehavior(zeroPowerBehavior);
        fRight.setZeroPowerBehavior(zeroPowerBehavior);
        bLeft.setZeroPowerBehavior(zeroPowerBehavior);
        bRight.setZeroPowerBehavior(zeroPowerBehavior);
    }

    public DcMotor getfLeft() {
        return fLeft;
    }

    public DcMotor getfRight() {
        return fRight;
    }

    public DcMotor getbLeft() {
        return bLeft;
    }

    public DcMotor getbRight() {
        return bRight;
    }

    public boolean isOn(){
        return isOn;
    }

    public boolean isFileDump() {
        return isFileDump;
    }
}
