package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.roadrunner.localization.Localizer;

import org.firstinspires.ftc.teamcode.PID.DriveConstantsPID;
import org.firstinspires.ftc.teamcode.PID.localizer.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.PID.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.PID.mecanum.SampleMecanumDriveREV;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.RobotLog;

import java.util.Arrays;
import java.util.List;
import java.lang.String;

import static org.firstinspires.ftc.teamcode.PID.DriveConstantsPID.RUN_USING_ENCODER;

/*
 * This is a simple routine to test turning capabilities.
 */
@Config
@Autonomous(group = "drive")
public class ManualParamTest extends LinearOpMode {
    private DcMotorEx leftFront, leftRear, rightRear, rightFront;
    private List<DcMotorEx> motors;
    private final int polling_interval = 200;
    private String TAG = "ManualParamTest";
    Localizer localizer = null;
    @Override
    public void runOpMode() throws InterruptedException {
        DriveConstantsPID.updateConstantsFromProperties();
        SampleMecanumDriveBase drive = new SampleMecanumDriveREV(hardwareMap);
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

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

                RobotLog.dd(TAG, "odometry positions");
                drive.print_list_double(odo_positions);
            }

            List<Double> velocities = drive.getWheelVelocities();
            RobotLog.dd(TAG, "velocities");
            drive.print_list_double(velocities);

            List<Double> positions = drive.getWheelPositions();
            RobotLog.dd(TAG, "wheel positions");
            drive.print_list_double(positions);

            List<Double> w_powers = drive.getMotorPowers(motors);
            RobotLog.dd(TAG, "wheel powers");
            drive.print_list_double(w_powers);

            double heading = drive.getExternalHeading();
            RobotLog.dd(TAG, "getExternalHeading: x " + heading);

            Pose2d pose = drive.getPoseEstimate();
            RobotLog.dd(TAG, "Pose: x " + pose.getX());
            RobotLog.dd(TAG, "Pose: y " + pose.getY());
            RobotLog.dd(TAG, "Pose: heading " + Double.toString(pose.getHeading()));

            Pose2d error = drive.getLastError();
            RobotLog.dd(TAG, "xError " + error.getX());
            RobotLog.dd(TAG, "yError " + error.getY());
            RobotLog.dd(TAG, "headingError "  + error.getHeading());
            Thread.sleep(polling_interval);
        }
    }
}
