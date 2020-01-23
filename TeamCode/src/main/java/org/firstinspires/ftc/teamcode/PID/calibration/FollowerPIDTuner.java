package org.firstinspires.ftc.teamcode.PID.calibration;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

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
@Autonomous(name = "FollowerPIDTuner", group = "drive")
//@Disabled
public class FollowerPIDTuner extends LinearOpMode {
    public static double DISTANCE = 0; // update later;
    private String TAG = "FollowerPIDTuner";

    @Override
    public void runOpMode() throws InterruptedException {
        DriveConstantsPID.updateConstantsFromProperties();  // Transitional PID is used in base class;;
        DISTANCE = DriveConstantsPID.TEST_DISTANCE;
      
        SampleMecanumDriveBase drive = null;
        if (DriveConstantsPID.USING_BULK_READ == false)
            drive = new SampleMecanumDriveREV(hardwareMap, false);
        else
            drive = new SampleMecanumDriveREVOptimized(hardwareMap, false);

        drive.setPoseEstimate(new Pose2d(DISTANCE / 2, DISTANCE / 2, 0));

        waitForStart();

        if (isStopRequested()) return;

        while (!isStopRequested()) {
            RobotLogger.dd(TAG, "move forward: "+Double.toString(DISTANCE));
            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .forward(DISTANCE)
                            .build()
            );
            RobotLogger.dd(TAG, "turn: 90");
            drive.turnSync(Math.toRadians(90));
        }
    }
}
