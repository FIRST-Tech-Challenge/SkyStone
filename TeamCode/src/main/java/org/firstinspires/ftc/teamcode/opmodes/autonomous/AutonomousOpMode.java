package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.hardware.HardwarePushbot;
import org.firstinspires.ftc.teamcode.utilities.PIDController;

import java.util.Locale;

public abstract class AutonomousOpMode extends LinearOpMode
{
    final double TURN_POWER = 1.0;
    final double DRIVE_POWER = 1.0;
    final double KP = 0; //TODO: Replace placeholder values
    final double KI = 0;
    final double KD = 0;

    PIDController pidRotate = new PIDController(KP, KI, KD);
    HardwarePushbot robot = new HardwarePushbot();
    AutonomousState state;
    Orientation angles;

    protected void rotate(double degrees, double power)
    {
        pidRotate.reset();
        pidRotate.setSetpoint(degrees);
        pidRotate.setInputRange(0, 90);
        pidRotate.setOutputRange(.20, power);
        pidRotate.setTolerance(2);
        pidRotate.enable();
        
        final double TURN_TOLERANCE = 10;

        power = Range.clip(power, -1, 1);
        robot.resetAngle();
        double leftPower, rightPower;

        //set the initial power and directino of the motors depending on rotation
        leftPower = -power * (degrees / Math.abs(degrees));
        rightPower = power * (degrees / Math.abs(degrees));

        robot.leftDrive.setPower(leftPower);
        robot.rightDrive.setPower(rightPower);

        //rotate within TURN_TOLERANCE degrees of error before stopping, and adjust as necessary
        //TODO: Use PID for correction!
        while (Math.abs(robot.getAngle() - degrees) > TURN_TOLERANCE)
        {
            if (robot.getAngle() > degrees + TURN_TOLERANCE && leftPower < 0)
            {
                leftPower = power;
                rightPower = -power;
                robot.leftDrive.setPower(leftPower);
                robot.rightDrive.setPower(rightPower);
            }

            if (robot.getAngle() < degrees - TURN_TOLERANCE && leftPower > 0)
            {
                leftPower = -power;
                rightPower = power;
                robot.leftDrive.setPower(leftPower);
                robot.rightDrive.setPower(rightPower);
            }
            telemetry.update();
        }

        //stop the robot
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);

        robot.resetAngle();
    }

    //robot drives until proximity sensor is hitting an object
    protected void driveUntilImpact(double power)
    {
        power = Range.clip(power, -1, 1);
        robot.leftDrive.setPower(power);
        robot.rightDrive.setPower(power);

        while (Double.isNaN(robot.distanceSensor.getDistance(DistanceUnit.CM)))
        {
            telemetry.update();
            sleep(25);
        }

        //telemetry.addLine().addData("RYAN!", " I'VE HIT LINE 204");

        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);
    }

    protected String formatAngle(AngleUnit angleUnit, double angle)
    {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    protected String formatDegrees(double degrees)
    {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    protected void composeTelemetry()
    {
        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(() ->
        {
            // Acquiring the angles is relatively expensive; we don't want
            // to do that in each of the three items that need that info, as that's
            // three times the necessary expense.
            angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX,
                    AngleUnit.DEGREES);
        });

        telemetry.addLine().addData("status", () -> robot.imu.getSystemStatus().toShortString())
                 .addData("calib", () -> robot.imu.getCalibrationStatus().toString());

        //heading is firstAngle
        telemetry.addLine()
                 .addData("heading", () -> formatAngle(angles.angleUnit, angles.firstAngle))
                 .addData("roll", () -> formatAngle(angles.angleUnit, angles.secondAngle))
                 .addData("pitch", () -> formatAngle(angles.angleUnit, angles.thirdAngle));

        telemetry.addLine().addData("dist", () -> String
                .format(Locale.US, "%.02f", robot.distanceSensor.getDistance(DistanceUnit.CM)));

        telemetry.addLine().addData("state", () -> state.toString());
    }
}
