package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.hardwareutils.HardwareManager;

public class AutoCommands{
    private HardwareManager hardware;
    private DcMotor leftFrontDrive;
    private DcMotor leftRearDrive;
    private DcMotor rightFrontDrive;
    private DcMotor rightRearDrive;
    Telemetry telemetry;

    public AutoCommands(HardwareManager hardware, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.hardware = hardware;
        leftFrontDrive = hardware.leftFrontDrive;
        rightFrontDrive = hardware.rightFrontDrive;
        rightRearDrive = hardware.rightRearDrive;
    }

    public void HorizontalMove(double power) {

        telemetry.addData("Power", power);
        double leftF = 0, rightF = 0, leftB = 0, rightB = 0;
        leftF += power;
        rightF += -power;
        leftB += -power;
        rightB += power;

        telemetry.addData("HorizontalMove method status", "Running");

        leftFrontDrive.setPower(leftF);
        leftRearDrive.setPower(leftB);
        rightFrontDrive.setPower(rightF);
        rightRearDrive.setPower(rightB);
    }

    public void driveForward(double power)
    {
        leftFrontDrive.setPower(power);
        leftRearDrive.setPower(power);
        rightFrontDrive.setPower(power);
        rightRearDrive.setPower(power);
    }

    /**
     * @param gyroTarget The target heading in degrees, between 0 and 360
     * @param gyroRange The acceptable range off target in degrees, usually 1 or 2
     * @param gyroActual The current heading in degrees, between 0 and 360
     * @param minSpeed The minimum power to apply in order to turn (e.g. 0.05 when moving or 0.15 when stopped)
     * @param addSpeed The maximum additional speed to apply in order to turn (proportional component), e.g. 0.3
     * @return The number of times in a row the heading has been in the range
     */
    public int gyroCorrect(double gyroTarget, double gyroRange, double gyroActual, double minSpeed, double addSpeed) {
        int correctCount = 0;
        double delta = (gyroTarget - gyroActual + 360.0) % 360.0; //the difference between target and actual mod 360
        if (delta > 180.0) delta -= 360.0; //makes delta between -180 and 180
        if (Math.abs(delta) > gyroRange) { //checks if delta is out of range
            correctCount = 0;
            double gyroMod = delta / 45.0; //scale from -1 to 1 if delta is less than 45 degrees
            if (Math.abs(gyroMod) > 1.0) gyroMod = Math.signum(gyroMod); //set gyromod to 1 or -1 if the error is more than 45 degrees
            turn(minSpeed * Math.signum(gyroMod) + addSpeed * gyroMod);
        }
        else {
            correctCount++;
            turn(0.0);
        }
        return correctCount;
    }

    private void turn(double power)
    {
        double leftF  = power;
        double rightF = -power;
        double leftB  = power;
        double rightB = -power;
        leftFrontDrive.setPower(leftF);
        rightFrontDrive.setPower(rightF);
        leftRearDrive.setPower(leftB);
        rightRearDrive.setPower(rightB);
    }
}