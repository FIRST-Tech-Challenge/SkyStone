package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.util.Locale;

@TeleOp(name = "Sensor: BNO055 IMU Calibration", group = "Sensor")

public class IMUTestCalibration extends LinearOpMode
{
    //----------------------------------------------------------------------------------------------
    // State
    //----------------------------------------------------------------------------------------------

    // Our sensors, motors, and other devices go here, along with other long term state
    BNO055IMU imu;

    // State used for updating telemetry
    Orientation angles;
    Acceleration accel;
    Velocity speed;
    Position location;

    //----------------------------------------------------------------------------------------------
    // Main logic
    //----------------------------------------------------------------------------------------------

    @Override
    public void runOpMode()
    {

        telemetry.log().setCapacity(12);
        telemetry.log().add("");
        telemetry.log().add("Please refer to the calibration instructions");
        telemetry.log().add("contained in the Adafruit IMU calibration");
        telemetry.log().add("sample opmode.");
        telemetry.log().add("");
        telemetry.log().add("When sufficient calibration has been reached,");
        telemetry.log().add("press the 'A' button to write the current");
        telemetry.log().add("calibration data to a file.");
        telemetry.log().add("");

        // We are expecting the IMU to be attached to an I2C port on a Core Device Interface Module and named "imu".
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        composeTelemetry();
        telemetry.log().add("Waiting for start...");

        // Wait until we're told to go
        while (!isStarted())
        {
            telemetry.update();
            idle();
        }

        telemetry.log().add("...started...");

        while (opModeIsActive())
        {

            if (gamepad1.a)
            {

                // Get the calibration data
                BNO055IMU.CalibrationData calibrationData = imu.readCalibrationData();

                // Save the calibration data to a file. You can choose whatever file
                // name you wish here, but you'll want to indicate the same file name
                // when you initialize the IMU in an opmode in which it is used. If you
                // have more than one IMU on your robot, you'll of course want to use
                // different configuration file names for each.
                String filename = "IMUTestCalibration.json";
                File file = AppUtil.getInstance().getSettingsFile(filename);
                ReadWriteFile.writeFile(file, calibrationData.serialize());
                telemetry.log().add("saved to '%s'", filename);

                // Wait for the button to be released
                while (gamepad1.a)
                {
                    telemetry.update();
                    idle();
                }
            }

            telemetry.update();
        }
    }

    void composeTelemetry()
    {

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(() ->
        {
            // Acquiring the angles is relatively expensive; we don't want
            // to do that in each of the three items that need that info, as that's
            // three times the necessary expense.
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX,
                    AngleUnit.DEGREES);
            accel = imu.getLinearAcceleration();
            speed = imu.getVelocity();
            location = imu.getPosition();
        });

        telemetry.addLine().addData("status", () -> imu.getSystemStatus().toShortString())
                 .addData("calib", () -> imu.getCalibrationStatus().toString());

        //heading is firstAngle
        telemetry.addLine()
                 .addData("heading", () -> formatAngle(angles.angleUnit, angles.firstAngle))
                 .addData("roll", () -> formatAngle(angles.angleUnit, angles.secondAngle))
                 .addData("pitch", () -> formatAngle(angles.angleUnit, angles.thirdAngle));

        telemetry.addLine().addData("acc", () -> accel.toString())
                 .addData("xAccel", () -> String.format(Locale.getDefault(), "%.3f", accel.xAccel))
                 .addData("yAccel", () -> String.format(Locale.getDefault(), "%.3f", accel.yAccel))
                 .addData("zAccel", () -> String.format(Locale.getDefault(), "%.3f", accel.zAccel));

        telemetry.addLine().addData("vel", () -> speed.toString())
                 .addData("xVel", () -> String.format(Locale.getDefault(), "%.3f", speed.xVeloc))
                 .addData("yVel", () -> String.format(Locale.getDefault(), "%.3f", speed.yVeloc))
                 .addData("zVel", () -> String.format(Locale.getDefault(), "%.3f", speed.zVeloc));

        telemetry.addLine().addData("pos", () -> speed.toString())
                 .addData("xPos", () -> String.format(Locale.getDefault(), "%.3f", location.x))
                 .addData("yPos", () -> String.format(Locale.getDefault(), "%.3f", location.y))
                 .addData("zPos", () -> String.format(Locale.getDefault(), "%.3f", location.z));
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
}
