
package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import android.text.ParcelableSpan;
import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.hardware.HardwareDummybot;
import org.firstinspires.ftc.teamcode.utilities.PIDController;

import java.util.Locale;

@Autonomous(name = "Ryan-kun Uwu", group = "Test")

public class IMUTest extends LinearOpMode
{

    // State used for updating telemetry
    Orientation angles;
    Acceleration accel;

    // Set PID proportional value to start reducing power at about 50 degrees of rotation.
    PIDController pidRotate = new PIDController(.005, 0, 0);

    //robot hardware components
    HardwareDummybot robot = new HardwareDummybot();

    //timer
    ElapsedTime time = new ElapsedTime();

    //----------------------------------------------------------------------------------------------
    // Main logic
    //----------------------------------------------------------------------------------------------

    @Override
    public void runOpMode()
    {
        Log.i("FRICK","time: " + time.toString());

        robot.init(hardwareMap);

        // Set up our telemetry dashboard
        composeTelemetry();
        // Wait until we're told to go
        waitForStart();

        if (opModeIsActive())
        {
            rotate(180, 0.2);
        }

        // Loop and update the dashboard
        while (opModeIsActive())
        {
            telemetry.update();
        }

    }

    //----------------------------------------------------------------------------------------------
    // Telemetry Configuration
    //----------------------------------------------------------------------------------------------

    void composeTelemetry()
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
            accel = robot.imu.getLinearAcceleration();
        });

        telemetry.addLine().addData("status", () -> robot.imu.getSystemStatus().toShortString())
                 .addData("calib", () -> robot.imu.getCalibrationStatus().toString());

        //heading is firstAngle
        telemetry.addLine()
                 .addData("heading", () -> formatAngle(angles.angleUnit, angles.firstAngle))
                 .addData("roll", () -> formatAngle(angles.angleUnit, angles.secondAngle))
                 .addData("pitch", () -> formatAngle(angles.angleUnit, angles.thirdAngle));

        telemetry.addLine().addData("acc", () -> accel.toString())
                 .addData("xAccel", () -> String.format(Locale.getDefault(), "%.3f", accel.xAccel))
                 .addData("yAccel", () -> String.format(Locale.getDefault(), "%.3f", accel.yAccel))
                 .addData("zAccel", () -> String.format(Locale.getDefault(), "%.3f", accel.zAccel));
    }

    //----------------------------------------------------------------------------------------------
    // Formatting
    //----------------------------------------------------------------------------------------------

    String formatAngle(AngleUnit angleUnit, double angle)
    {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees)
    {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    /**
     * Rotate left or right the number of degrees. Does not support turning more than 180 degrees.
     * Author: https://stemrobotics.cs.pdx.edu/node/9156
     *
     * @param degrees Degrees to turn, + is left - is right
     * @param power   supplied to both motors between -1.0 and 1.0
     */
    private void rotate(int degrees, double power)
    {
        // restart imu angle tracking.
        robot.resetAngle();

        // start pid controller. PID controller will monitor the turn angle with respect to the
        // target angle and reduce power as we approach the target angle with a minimum of 20%.
        // This is to prevent the robots momentum from overshooting the turn after we turn off the
        // power. The PID controller reports onTarget() = true when the difference between turn
        // angle and target angle is within 2% of target (tolerance). This helps prevent overshoot.
        // The minimum power is determined by testing and must enough to prevent motor stall and
        // complete the turn. Note: if the gap between the starting power and the stall (minimum)
        // power is small, overshoot may still occur. Overshoot is dependant on the motor and
        // gearing configuration, starting power, weight of the robot and the on target tolerance.

        pidRotate.reset();
        pidRotate.setSetpoint(degrees);
        pidRotate.setInputRange(0.2, 1);
        pidRotate.setOutputRange(0, power);
        pidRotate.setTolerance(2);
        pidRotate.enable();

        // robot.getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        // rotate until turn is completed.

        if (degrees < 0)
        {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && robot.getAngle() == 0)
            {
                robot.leftDrive.setPower(-power);
                robot.rightDrive.setPower(power);
                sleep(100);
            }

            do
            {
                power = pidRotate.performPID(robot.getAngle()); // power will be - on right turn.
                robot.leftDrive.setPower(power);
                robot.rightDrive.setPower(-power);
            }
            while (opModeIsActive() && !pidRotate.onTarget());
        }
        else    // left turn.
        {
            do
            {
                power = pidRotate.performPID(robot.getAngle()); // power will be + on left turn.
                robot.leftDrive.setPower(power);
                robot.rightDrive.setPower(-power);
            }
            while (opModeIsActive() && !pidRotate.onTarget());
        }

        // turn the motors off.
        robot.rightDrive.setPower(0);
        robot.leftDrive.setPower(0);

        // wait for rotation to stop.
        sleep(500);

        // reset angle tracking on new heading.
        robot.resetAngle();
    }

}