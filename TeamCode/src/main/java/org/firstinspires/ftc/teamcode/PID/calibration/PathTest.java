package org.firstinspires.ftc.teamcode.PID.calibration;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.BaseTrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.PID.DriveConstantsPID;
import org.firstinspires.ftc.teamcode.PID.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.PID.mecanum.SampleMecanumDriveREV;

/*
 * This is a simple routine to test turning capabilities.
 */
@Config
@Autonomous(group = "drive")
public class PathTest extends LinearOpMode {
    private Trajectory trajectory;
    private BaseTrajectoryBuilder builder;
    private Pose2d current_pose;
    @Override
    public void runOpMode() throws InterruptedException {
        DriveConstantsPID.updateConstantsFromProperties();
        SampleMecanumDriveBase drive = new SampleMecanumDriveREV(hardwareMap, false);

        waitForStart();

        if (isStopRequested()) return;

        Trajectory trajectory = drive.trajectoryBuilder()
                .strafeLeft(DriveConstantsPID.TEST_DISTANCE)
                .build();

        drive.getLocalizer().setPoseEstimate(new Pose2d(new Vector2d(drive.getPoseEstimate().getX(),
                drive.getPoseEstimate().getY()), drive.getExternalHeading()));
        drive.getLocalizer().update();
        current_pose = drive.getPoseEstimate();

        drive = new SampleMecanumDriveREV(hardwareMap, true);
        drive.setPoseEstimate(current_pose);
        RobotLog.dd("Current Position", drive.getPoseEstimate().toString());

        builder = new TrajectoryBuilder(drive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);   //-34.752, -63.936
        if(DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {

            builder = builder//.strafeTo(new Vector2d(drive.getPoseEstimate().getX(), -63.936 + 5.0))
                    /*.strafeTo(new Vector2d(drive.getPoseEstimate().getX(), -63.936 + 5.0 * 2))
                    .strafeTo(new Vector2d(drive.getPoseEstimate().getX(), -63.936 + 5.0 * 3))
                    .strafeTo(new Vector2d(drive.getPoseEstimate().getX(), -63.936 + 5.0 * 4))
                    .strafeTo(new Vector2d(drive.getPoseEstimate().getX(), -63.936 + 5.0 * 5))*/
                    .setReversed(true).lineTo(new Vector2d(-51, -39));

        } else {
            builder = builder.strafeTo(new Vector2d(drive.getPoseEstimate().getX(),-30)).setReversed(true)
                    .lineTo(new Vector2d(-50, -30));
        }
        trajectory = builder.build();   //x - 2.812, y + 7.984
        drive.followTrajectorySync(trajectory);
    }
}
