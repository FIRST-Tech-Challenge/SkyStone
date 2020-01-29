package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.roadrunner.localization.Localizer;

import org.firstinspires.ftc.teamcode.PID.DriveConstantsPID;
import org.firstinspires.ftc.teamcode.PID.RobotLogger;
import org.firstinspires.ftc.teamcode.PID.localizer.IMUBufferReader;
import org.firstinspires.ftc.teamcode.PID.localizer.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.PID.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.PID.mecanum.SampleMecanumDriveREV;
import org.firstinspires.ftc.teamcode.PID.mecanum.SampleMecanumDriveREVOptimized;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import java.util.Arrays;
import java.util.List;
import java.lang.String;

import static org.firstinspires.ftc.teamcode.PID.DriveConstantsPID.RUN_USING_ENCODER;

/*
 * This is a simple routine to test turning capabilities.
 */
@Config
@Autonomous(name = "ManualParamTest", group = "drive")
public class ManualParamTest extends LinearOpMode {
    private DcMotorEx leftFront, leftRear, rightRear, rightFront;
    private List<DcMotorEx> motors;
    private final int polling_interval = 500;
    private String TAG = "ManualParamTest";
    Localizer localizer = null;
    //IMUBufferReader imu = IMUBufferReader.getSingle_instance(hardwareMap);

    @Override
    public void runOpMode() throws InterruptedException {
        DriveConstantsPID.updateConstantsFromProperties();
        SampleMecanumDriveBase drive = null;
        if (DriveConstantsPID.USING_BULK_READ == false)
            drive = new SampleMecanumDriveREV(hardwareMap, false);
        else
            drive = new SampleMecanumDriveREVOptimized(hardwareMap, false);

        leftFront = hardwareMap.get(DcMotorEx.class, "frontLeft");
        leftRear = hardwareMap.get(DcMotorEx.class, "backLeft");
        rightRear = hardwareMap.get(DcMotorEx.class, "backRight");
        rightFront = hardwareMap.get(DcMotorEx.class, "frontRight");

        motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);
        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
        localizer = drive.getLocalizer();
        waitForStart();

        while (opModeIsActive()) {
            if (DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL && (localizer!=null)) {
                StandardTrackingWheelLocalizer t = (StandardTrackingWheelLocalizer)localizer; // @TODO
                List<Double>  odo_positions = t.getWheelPositions();

                RobotLogger.dd(TAG, "odometry positions");
                drive.print_list_double(odo_positions);
            }

            List<Double> velocities = drive.getWheelVelocities();
            RobotLogger.dd(TAG, "velocities");
            drive.print_list_double(velocities);

            List<Double> positions = drive.getWheelPositions();
            RobotLogger.dd(TAG, "wheel positions");
            drive.print_list_double(positions);

            double heading = drive.getExternalHeading();
            RobotLogger.dd(TAG, "getExternalHeading: x " + heading);

            Pose2d pose = drive.getPoseEstimate();
            RobotLogger.dd(TAG, "Pose: x " + pose.getX());
            RobotLogger.dd(TAG, "Pose: y " + pose.getY());
            RobotLogger.dd(TAG, "Pose: heading " + Double.toString(pose.getHeading()));

            Pose2d error = drive.getLastError();
            RobotLogger.dd(TAG, "xError " + error.getX());
            RobotLogger.dd(TAG, "yError " + error.getY());
            RobotLogger.dd(TAG, "headingError "  + error.getHeading());
            Thread.sleep(polling_interval);
        };

    }
}
