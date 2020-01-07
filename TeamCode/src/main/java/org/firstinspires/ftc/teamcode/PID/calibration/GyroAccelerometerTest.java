package org.firstinspires.ftc.teamcode.PID.calibration;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import org.firstinspires.ftc.teamcode.PID.DriveConstantsPID;
import org.firstinspires.ftc.teamcode.PID.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.PID.mecanum.SampleMecanumDriveREV;

/*
 * This is a simple routine to test turning capabilities.
 */
@Config
@Autonomous(group = "drive")
public class GyroAccelerometerTest extends LinearOpMode {
    private BNO055IMU imu;
    private static String TAG = "GyroAccelerometer";
    @Override
    public void runOpMode() throws InterruptedException {
        DriveConstantsPID.updateConstantsFromProperties();
        SampleMecanumDriveBase drive = new SampleMecanumDriveREV(hardwareMap, false);
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);
        imu.startAccelerationIntegration(new Position(), new Velocity(), 100);

        waitForStart();

        if (isStopRequested()) return;
        while (opModeIsActive()) {
            Acceleration a = imu.getLinearAcceleration();
            Velocity v = imu.getVelocity();
            Position p = imu.getPosition();

            RobotLog.dd(TAG, "Acceleration: " + a.toString());
            RobotLog.dd(TAG, "Velocity: " + v.toString());
            RobotLog.dd(TAG, "Position: " + p.toString());
            Thread.sleep(200);
        }
    }
}
