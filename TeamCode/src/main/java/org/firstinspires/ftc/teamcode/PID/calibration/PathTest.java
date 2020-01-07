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

import org.firstinspires.ftc.teamcode.Autonomous.FieldPosition;
import org.firstinspires.ftc.teamcode.PID.DriveConstantsPID;
import org.firstinspires.ftc.teamcode.PID.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.PID.mecanum.SampleMecanumDriveREV;
import org.firstinspires.ftc.teamcode.TeleOp.TeleopConstants;

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

        drive.getLocalizer().setPoseEstimate(new Pose2d(new Vector2d(drive.getPoseEstimate().getX(),
                drive.getPoseEstimate().getY()), drive.getExternalHeading()));
        drive.getLocalizer().update();
        current_pose = drive.getPoseEstimate();

        drive = new SampleMecanumDriveREV(hardwareMap, true);
        drive.setPoseEstimate(current_pose);
        RobotLog.dd("Current Position", drive.getPoseEstimate().toString());

        builder = new TrajectoryBuilder(drive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);   //-34.752, -63.936
        if(DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {

            double initStrafeDistance = 15.0;
            double yCoordMvmtPlane = -63.936 + initStrafeDistance;
            double wallSkyStoneX = -48.0;
            double furtherMostSkyStoneX = -35.0;
            double firstRegularStoneX = -44.0;
            double foundationX = 45.0;
            double strafeDistance = 10.0;
            double rightStrafeCorrection = -2.2;

            //odometryStrafe(0.2, initStrafeDistance, false);
            //DriveConstantsPID.strafeDistance(hardwareMap, initStrafeDistance, true);
            //updatePoseFromStrafe(initPos, true);

            builder = new TrajectoryBuilder(new Pose2d(new Vector2d(-34.752, -63.936), Math.toRadians(0)), DriveConstantsPID.BASE_CONSTRAINTS);   //-34.752, -63.936
            if(DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {

                builder = builder//.strafeTo(new Vector2d(drive.getPoseEstimate().getX(), -63.936 + 5.0))
                        /*.strafeTo(new Vector2d(drive.getPoseEstimate().getX(), -63.936 + 5.0 * 2))
                        .strafeTo(new Vector2d(drive.getPoseEstimate().getX(), -63.936 + 5.0 * 3))
                        .strafeTo(new Vector2d(drive.getPoseEstimate().getX(), -63.936 + 5.0 * 4))
                        .strafeTo(new Vector2d(drive.getPoseEstimate().getX(), -63.936 + 5.0 * 5))*/
                        .strafeTo(new Vector2d(drive.getPoseEstimate().getX(), yCoordMvmtPlane + strafeDistance))
                        .setReversed(false).setReversed(true).lineTo(new Vector2d(wallSkyStoneX, yCoordMvmtPlane + strafeDistance));

            } else {
                builder = builder.strafeTo(new Vector2d(drive.getPoseEstimate().getX(),-30)).setReversed(true)
                        .lineTo(new Vector2d(-50, -30));
            }
            trajectory = builder.build();   //x - 2.812, y + 7.984
            drive.followTrajectorySync(trajectory);


            drive.getLocalizer().setPoseEstimate(new Pose2d(new Vector2d(drive.getPoseEstimate().getX(),
                    drive.getPoseEstimate().getY()), drive.getExternalHeading()));
            drive.getLocalizer().update();
            builder = new TrajectoryBuilder(drive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);
            if(DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {
                builder = builder.setReversed(false).strafeTo(new Vector2d(wallSkyStoneX, yCoordMvmtPlane))
                        .lineTo(new Vector2d(foundationX, yCoordMvmtPlane));
                //-52, -39
            } else {
                builder = builder.setReversed(false).strafeTo(new Vector2d(-50, -40))
                        .lineTo(new Vector2d(50, -40)).strafeTo(new Vector2d(50, -30));
            }
            trajectory = builder.build();
            drive.followTrajectorySync(trajectory);

            drive.getLocalizer().setPoseEstimate(new Pose2d(new Vector2d(drive.getPoseEstimate().getX(),
                    drive.getPoseEstimate().getY()), drive.getExternalHeading()));
            drive.getLocalizer().update();
            builder = new TrajectoryBuilder(drive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);
            if(DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {
                builder = builder.setReversed(false).strafeTo(new Vector2d(foundationX,yCoordMvmtPlane + strafeDistance));  //-52, -39
            } else {
                builder = builder.setReversed(false).strafeTo(new Vector2d(-50, -40))
                        .lineTo(new Vector2d(50, -40)).strafeTo(new Vector2d(50, -30));
            }
            trajectory = builder.build();
            drive.followTrajectorySync(trajectory);


            //odometryStrafe(0.2, strafeDistance, true);
            //DriveConstantsPID.strafeDistance(hardwareMap, strafeDistance + rightStrafeCorrection, false);
            //updatePoseFromStrafe(initPos, false);

            drive.getLocalizer().setPoseEstimate(new Pose2d(new Vector2d(drive.getPoseEstimate().getX(),
                    drive.getPoseEstimate().getY()), drive.getExternalHeading()));
            drive.getLocalizer().update();
            builder = new TrajectoryBuilder(drive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);
            if(DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {
                builder = builder.strafeTo(new Vector2d(foundationX, yCoordMvmtPlane))
                        .setReversed(true).lineTo(new Vector2d(furtherMostSkyStoneX, yCoordMvmtPlane));
            } else {
                builder = builder.strafeTo(new Vector2d(50, -40)).setReversed(true).lineTo(new Vector2d(-20, -40))
                        .strafeTo(new Vector2d(-20, -28));
            }
            trajectory = builder.build();
            drive.followTrajectorySync(trajectory);

            drive.getLocalizer().setPoseEstimate(new Pose2d(new Vector2d(drive.getPoseEstimate().getX(),
                    drive.getPoseEstimate().getY()), drive.getExternalHeading()));
            drive.getLocalizer().update();
            builder = new TrajectoryBuilder(drive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);
            if(DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {
                builder = builder
                        .strafeTo(new Vector2d(furtherMostSkyStoneX, yCoordMvmtPlane + strafeDistance));
            } else {
                builder = builder.strafeTo(new Vector2d(50, -40)).setReversed(true).lineTo(new Vector2d(-20, -40))
                        .strafeTo(new Vector2d(-20, -28));
            }
            trajectory = builder.build();
            drive.followTrajectorySync(trajectory);

            //odometryStrafe(0.2, strafeDistance, true);
            //DriveConstantsPID.strafeDistance(hardwareMap, strafeDistance + rightStrafeCorrection, false);
            //updatePoseFromStrafe(initPos, false);

            drive.getLocalizer().setPoseEstimate(new Pose2d(new Vector2d(drive.getPoseEstimate().getX(),
                    drive.getPoseEstimate().getY()), drive.getExternalHeading()));
            drive.getLocalizer().update();
            builder = new TrajectoryBuilder(drive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);
            if(DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {
                builder = builder.setReversed(false).strafeTo(new Vector2d(furtherMostSkyStoneX, yCoordMvmtPlane))
                        .lineTo(new Vector2d(foundationX, yCoordMvmtPlane));
            } else {
                builder = builder.setReversed(false).strafeTo(new Vector2d(-20, -40)).lineTo(new Vector2d(50, -40))
                        .strafeTo(new Vector2d(50, -30));
            }
            trajectory = builder.build();
            drive.followTrajectorySync(trajectory);

            drive.getLocalizer().setPoseEstimate(new Pose2d(new Vector2d(drive.getPoseEstimate().getX(),
                    drive.getPoseEstimate().getY()), drive.getExternalHeading()));
            drive.getLocalizer().update();
            builder = new TrajectoryBuilder(drive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);
            if(DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {
                builder = builder.setReversed(false)
                        .strafeTo(new Vector2d(foundationX, yCoordMvmtPlane + strafeDistance));
            } else {
                builder = builder.setReversed(false).strafeTo(new Vector2d(-20, -40)).lineTo(new Vector2d(50, -40))
                        .strafeTo(new Vector2d(50, -30));
            }
            trajectory = builder.build();
            drive.followTrajectorySync(trajectory);


            drive.getLocalizer().setPoseEstimate(new Pose2d(new Vector2d(drive.getPoseEstimate().getX(),
                    drive.getPoseEstimate().getY()), drive.getExternalHeading()));
            drive.getLocalizer().update();
            builder = new TrajectoryBuilder(drive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);
            if(!DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {
                builder = builder.strafeTo(new Vector2d(foundationX, yCoordMvmtPlane))
                        .setReversed(true).lineTo(new Vector2d(firstRegularStoneX, yCoordMvmtPlane));
            }
            trajectory = builder.build();
            drive.followTrajectorySync(trajectory);

            drive.getLocalizer().setPoseEstimate(new Pose2d(new Vector2d(drive.getPoseEstimate().getX(),
                    drive.getPoseEstimate().getY()), drive.getExternalHeading()));
            drive.getLocalizer().update();
            builder = new TrajectoryBuilder(drive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);
            if(!DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {
                builder = builder
                        .strafeTo(new Vector2d(firstRegularStoneX, yCoordMvmtPlane + strafeDistance));
            }
            trajectory = builder.build();
            drive.followTrajectorySync(trajectory);


            drive.getLocalizer().setPoseEstimate(new Pose2d(new Vector2d(drive.getPoseEstimate().getX(),
                    drive.getPoseEstimate().getY()), drive.getExternalHeading()));
            drive.getLocalizer().update();
            builder = new TrajectoryBuilder(drive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);
            if(DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {
                builder = builder.setReversed(false).strafeTo(new Vector2d(firstRegularStoneX, yCoordMvmtPlane))
                        .lineTo(new Vector2d(foundationX, yCoordMvmtPlane));
            }
            trajectory = builder.build();
            drive.followTrajectorySync(trajectory);

            drive.getLocalizer().setPoseEstimate(new Pose2d(new Vector2d(drive.getPoseEstimate().getX(),
                    drive.getPoseEstimate().getY()), drive.getExternalHeading()));
            drive.getLocalizer().update();
            builder = new TrajectoryBuilder(drive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);
            if(DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {
                builder = builder.setReversed(false)
                        .strafeTo(new Vector2d(foundationX, yCoordMvmtPlane + strafeDistance));
            }
            trajectory = builder.build();
            drive.followTrajectorySync(trajectory);


        } else {
            builder = builder.strafeTo(new Vector2d(drive.getPoseEstimate().getX(),-30)).setReversed(true)
                    .lineTo(new Vector2d(-50, -30));
        }
        trajectory = builder.build();   //x - 2.812, y + 7.984
        drive.followTrajectorySync(trajectory);
    }
}
