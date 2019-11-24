package org.firstinspires.ftc.robotlib.robot;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.BNO055IMUImpl;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Locale;

public class GyroRobot
{
    public BNO055IMUImpl imu;
    private Telemetry telemetry;

    private Orientation angles;
    private Acceleration gravity;
    private Acceleration acceleration;

    public GyroRobot(HardwareMap hwMap, Telemetry telemetry)
    {
        this.telemetry = telemetry;
        imu = hwMap.get(BNO055IMUImpl.class, "imu");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        //parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu.initialize(parameters);

        telemetry.addAction(new Runnable() { @Override public void run()
        {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            gravity = imu.getGravity();
            acceleration = imu.getAcceleration();
        }});
        telemetry.addLine()
                .addData("status", new Func<String>()
                {
                    @Override public String value()
                    {
                        return imu.getSystemStatus().toShortString();
                    }
                })
                .addData("calib", new Func<String>()
                {
                    @Override public String value()
                    {
                        return imu.getCalibrationStatus().toString();
                    }
                });
        telemetry.addLine()
                .addData("heading", new Func<String>()
                {
                    @Override public String value()
                    {
                        return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                })
                .addData("roll", new Func<String>()
                {
                    @Override public String value()
                    {
                        return formatAngle(angles.angleUnit, angles.secondAngle);
                    }
                })
                .addData("pitch", new Func<String>()
                {
                    @Override public String value()
                    {
                        return formatAngle(angles.angleUnit, angles.thirdAngle);
                    }
                });
        telemetry.addLine()
                .addData("grvty", new Func<String>()
                {
                    @Override public String value()
                    {
                        return gravity.toString();
                    }
                })
                .addData("mag", new Func<String>()
                {
                    @Override public String value()
                    {
                        return String.format(Locale.getDefault(), "%.3f",
                                Math.sqrt(gravity.xAccel*gravity.xAccel
                                        + gravity.yAccel*gravity.yAccel
                                        + gravity.zAccel*gravity.zAccel));
                    }
                });
        telemetry.addLine()
                .addData("accel", new Func<String>()
                {
                    @Override public String value()
                    {
                        return acceleration.toString();
                    }
                })
                .addData("mag", new Func<String>()
                {
                    @Override public String value()
                    {
                        return String.format(Locale.getDefault(), "%.3f",
                                Math.sqrt(acceleration.xAccel * acceleration.xAccel
                                + acceleration.yAccel * acceleration.yAccel
                                + acceleration.zAccel * acceleration.zAccel));
                    }
                });
    }

    public void informationUpdate()
    {
        telemetry.addData("> IMU", "-----");
        telemetry.addData("Is Gyro Calibrated: ", imu.isGyroCalibrated());
        telemetry.addData("Is Acceleleration Calibrated: ", imu.isAccelerometerCalibrated());
        telemetry.addData("Is Magnet Calibrated: ", imu.isMagnetometerCalibrated());
        telemetry.addData("Is System Calibrated: ", imu.isSystemCalibrated());
        telemetry.addData("Acceleration: ", imu.getAcceleration());
        telemetry.addData("Velocity: ", imu.getVelocity());
        telemetry.addData("Position: ", imu.getPosition());
        telemetry.addData("Angular Orientation: ", imu.getAngularOrientation());
        telemetry.addData("Heading: ", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
    }

    private String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    private String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
}
