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
    private BaseTrajectoryBuilder builder, strafe_builder;
    private Pose2d current_pose;
    private String TAG = "PathTest";
    @Override
    public void runOpMode() throws InterruptedException {
        DriveConstantsPID.updateConstantsFromProperties();
        RobotLog.dd(TAG, "create non-strafe drive");
        SampleMecanumDriveBase drive = new SampleMecanumDriveREV(hardwareMap, false);
        RobotLog.dd("Current Position", drive.getPoseEstimate().toString());
        current_pose = drive.getPoseEstimate();

        RobotLog.dd(TAG, "create strafe drive");
        SampleMecanumDriveBase strafe_drive = new SampleMecanumDriveREV(hardwareMap, true);
        RobotLog.dd("Current Position", strafe_drive.getPoseEstimate().toString());

        waitForStart();

        if (isStopRequested()) return;

        RobotLog.dd(TAG, "update pose for strafe drive");
        strafe_drive.getLocalizer().setPoseEstimate(new Pose2d(new Vector2d(drive.getPoseEstimate().getX(),
                drive.getPoseEstimate().getY()), drive.getExternalHeading()));
        strafe_drive.getLocalizer().update();

        strafe_drive.setPoseEstimate(current_pose);

        strafe_builder = new TrajectoryBuilder(strafe_drive.getPoseEstimate(), DriveConstantsPID.STRAFE_BASE_CONSTRAINTS);

        strafe_builder = strafe_builder.strafeTo(new Vector2d(drive.getPoseEstimate().getX(),24));

        trajectory = strafe_builder.build();
        strafe_drive.followTrajectorySync(trajectory);

        RobotLog.dd(TAG, "update pose for drive after strafing: " + strafe_drive.getPoseEstimate().toString());
        drive.getLocalizer().setPoseEstimate(new Pose2d(new Vector2d(strafe_drive.getPoseEstimate().getX(),
                strafe_drive.getPoseEstimate().getY()), strafe_drive.getExternalHeading()));
        drive.getLocalizer().update();

        builder = new TrajectoryBuilder(drive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);
        builder.forward(72);
        trajectory = builder.build();
        drive.followTrajectorySync(trajectory);
        RobotLog.dd(TAG, "done testing, current pose: " + drive.getPoseEstimate().toString());

    }
}
