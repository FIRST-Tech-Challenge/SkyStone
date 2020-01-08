package org.firstinspires.ftc.teamcode.PID.calibration;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.PID.DriveConstantsPID;
import org.firstinspires.ftc.teamcode.PID.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.PID.mecanum.SampleMecanumDriveREV;
import org.firstinspires.ftc.teamcode.PID.mecanum.SampleMecanumDriveREVOptimized;

/*
 * Op mode for tuning follower PID coefficients (located in the drive base classes). The robot
 * drives in a DISTANCE-by-DISTANCE square indefinitely.
 */
@Config
@Autonomous(name = "FollowerPIDTunerStraight", group = "drive")
//@Disabled
public class FollowerPIDTunerStraight extends LinearOpMode {
    public static double DISTANCE = 0; // update later;
    private String TAG = "FollowerPIDTunerStraight";

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

            RobotLog.dd(TAG, "current pose: " + drive.getPoseEstimate().toString());
            RobotLog.dd(TAG, "move forward: "+Double.toString(DISTANCE));
            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .forward(DISTANCE)
                            .build()
            );
            RobotLog.dd(TAG, "current pose: " + drive.getPoseEstimate().toString());
            RobotLog.dd(TAG, "move back: "+Double.toString(DISTANCE));
            //drive.turnSync(Math.toRadians(90));
            try{
                Thread.sleep(5000);
            } catch(Exception e){}

            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .back(DISTANCE)
                            .build()
            );
            try{
                Thread.sleep(5000);
            } catch(Exception e){}

        }
    }
}
