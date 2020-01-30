package org.firstinspires.ftc.teamcode.PID.calibration;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.All.DriveConstant;
import org.firstinspires.ftc.teamcode.PID.DriveConstantsPID;
import org.firstinspires.ftc.teamcode.PID.RobotLogger;
import org.firstinspires.ftc.teamcode.PID.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.PID.mecanum.SampleMecanumDriveREV;
import org.firstinspires.ftc.teamcode.PID.mecanum.SampleMecanumDriveREVOptimized;

/*
 * Op mode for tuning follower PID coefficients (located in the drive base classes). The robot
 * drives in a DISTANCE-by-DISTANCE square indefinitely.
 */
@Config
@Autonomous(name = "FollowerPIDTunerStrafe", group = "drive")
//@Disabled
public class FollowerPIDTunerStrafe extends LinearOpMode {
    public static double DISTANCE = 0; // update later;
    private String TAG = "FollowerPIDTunerStrafe";

    @Override
    public void runOpMode() throws InterruptedException {
        DriveConstantsPID.updateConstantsFromProperties();  // Transitional PID is used in base class;;
        DISTANCE = DriveConstantsPID.TEST_DISTANCE;
        SampleMecanumDriveBase drive = null;
        if (DriveConstantsPID.USING_BULK_READ == false)
            drive = new SampleMecanumDriveREV(hardwareMap, false);
        else
            drive = new SampleMecanumDriveREVOptimized(hardwareMap, false);
        drive.setBrakeonZeroPower(DriveConstantsPID.BRAKE_ON_ZERO);
        drive.setPoseEstimate(new Pose2d(0, 0, 0));

        waitForStart();

        if (isStopRequested()) return;

        while (!isStopRequested()) {
            if (DriveConstantsPID.USING_BULK_READ == false)
                drive = new SampleMecanumDriveREV(hardwareMap, false);
            else
                drive = new SampleMecanumDriveREVOptimized(hardwareMap, false);
            drive.setBrakeonZeroPower(DriveConstantsPID.BRAKE_ON_ZERO);

            drive.setPoseEstimate(new Pose2d(0, 0, 0));

            if (DriveConstantsPID.USING_STRAFE_DIAGNAL == true) {
                if (DriveConstantsPID.RESET_FOLLOWER)
                    drive.resetFollowerWithParameters(true, false);
                drive.followTrajectorySync(
                        drive.trajectoryBuilder()
                                .strafeTo(new Vector2d(DriveConstantsPID.TEST_DISTANCE, DriveConstantsPID.TEST_DISTANCE_0))
                                .build());
            }
            else {
                if (DriveConstantsPID.RESET_FOLLOWER)
                    drive.resetFollowerWithParameters(false, false);
                drive.followTrajectorySync(
                        drive.trajectoryBuilder()
                                .lineTo(new Vector2d(DriveConstantsPID.TEST_DISTANCE, DriveConstantsPID.TEST_DISTANCE_0))
                                .build());
            }

            Pose2d currentPos = drive.getPoseEstimate();
            Pose2d error_pose = drive.follower.getLastError();
            RobotLogger.dd(TAG, "currentPos %s, errorPos %s",currentPos.toString(), error_pose.toString());
            //drive.turnSync(Math.toRadians(90));
            try{
                Thread.sleep(2000);
            } catch(Exception e){}

            if (DriveConstantsPID.USING_STRAFE_DIAGNAL == true) {
                //drive.resetFollowerWithParameters(true);
                drive.followTrajectorySync(
                        drive.trajectoryBuilder()
                                .strafeTo(new Vector2d(0, 0))
                                .build());
            }
            else {
                //drive.resetFollowerWithParameters(false);
                drive.followTrajectorySync(
                        drive.trajectoryBuilder()
                                .lineTo(new Vector2d(0, 0))
                                .build());
            }
            currentPos = drive.getPoseEstimate();
            error_pose = drive.follower.getLastError();
            RobotLogger.dd(TAG, "currentPos %s, errorPos %s",currentPos.toString(), error_pose.toString());

            try{
                Thread.sleep(2000);
            } catch(Exception e){}

        }
    }
}
