package com.hfrobots.tnt.season1819.support;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.path.heading.ConstantInterpolator;
import com.acmerobotics.roadrunner.trajectory.DashboardUtil;
import com.acmerobotics.roadrunner.followers.MecanumPIDVAFollower;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumConstraints;
import com.hfrobots.tnt.corelib.Constants;
import com.hfrobots.tnt.season1819.RoadrunnerMecanumDriveAdapter;
import com.hfrobots.tnt.season1819.TntPose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Spline follow", group="Utilities")
@Disabled
public class SplineFollowOpMode extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        //FtcDashboard dashboard = FtcDashboard.getInstance();
        RoadrunnerMecanumDriveAdapter drive = new RoadrunnerMecanumDriveAdapter(hardwareMap);
        // change these constraints to something reasonable for your drive
        DriveConstraints baseConstraints = new DriveConstraints(25.0, 40.0, Math.PI / 2, Math.PI / 2);
        MecanumConstraints constraints = new MecanumConstraints(baseConstraints, drive.getTrackWidth(), drive.getWheelBase());

        //Trajectory trajectory = new TrajectoryBuilder(new Pose2d(0, 0, 0), constraints)
        //        .lineTo(new Vector2d(0, 20), new ConstantInterpolator(0))
        //        .build();

        // Relative points

        Trajectory trajectory = new TrajectoryBuilder(
                TntPose2d.toPose2d(0, 0, 0), constraints)
                .lineTo(TntPose2d.toVector2d(0, 36), new ConstantInterpolator(0))
                .turnTo(Math.toRadians(45)).build();

        Log.d(Constants.LOG_TAG, "Trajectory duration: " + trajectory.duration());

        // TODO: tune kV, kA, and kStatic in the following follower
        // then tune the PID coefficients after you verify the open loop response is roughly correct
        MecanumPIDVAFollower follower = new MecanumPIDVAFollower(
                drive,
                new PIDCoefficients(0, 0, 0),
                new PIDCoefficients(0, 0, 0),
                .01283,
                0,
                0);

        waitForStart();

        follower.followTrajectory(trajectory);
        while (opModeIsActive() && follower.isFollowing()) {
            Pose2d currentPose = drive.getPoseEstimate();

            Log.d(Constants.LOG_TAG, "Pose: " + currentPose.getX() + ", " + currentPose.getY() + ", " + currentPose.getHeading());
            Log.d(Constants.LOG_TAG, "Remaining duration: " + trajectory.duration());

            //TelemetryPacket packet = new TelemetryPacket();
            //Canvas fieldOverlay = packet.fieldOverlay();
            //fieldOverlay.setStroke("green");
            //DashboardUtil.drawSampledTrajectory(fieldOverlay, trajectory);
            //fieldOverlay.setFill("blue");
            //fieldOverlay.fillCircle(currentPose.getX(), currentPose.getY(), 3);
            //dashboard.sendTelemetryPacket(packet);

            follower.update(currentPose);
            drive.updatePoseEstimate();
        }

        trajectory = new TrajectoryBuilder(TntPose2d.toPose2d(0, 0, 0), constraints)
                .lineTo(TntPose2d.toVector2d(10, 0), new ConstantInterpolator(0))
                .lineTo(TntPose2d.toVector2d(10, -48), new ConstantInterpolator(0)).build();

        follower.followTrajectory(trajectory);
        while (opModeIsActive() && follower.isFollowing()) {
            Pose2d currentPose = drive.getPoseEstimate();

            Log.d(Constants.LOG_TAG, "Pose: " + currentPose.getX() + ", " + currentPose.getY() + ", " + currentPose.getHeading());
            Log.d(Constants.LOG_TAG, "Remaining duration: " + trajectory.duration());

            //TelemetryPacket packet = new TelemetryPacket();
            //Canvas fieldOverlay = packet.fieldOverlay();
            //fieldOverlay.setStroke("green");
            //DashboardUtil.drawSampledTrajectory(fieldOverlay, trajectory);
            //fieldOverlay.setFill("blue");
            //fieldOverlay.fillCircle(currentPose.getX(), currentPose.getY(), 3);
            //dashboard.sendTelemetryPacket(packet);

            follower.update(currentPose);
            drive.updatePoseEstimate();
        }

        trajectory = new TrajectoryBuilder(TntPose2d.toPose2d(0, 0, 0), constraints)
                .lineTo(TntPose2d.toVector2d(10, 72), new ConstantInterpolator(0)).build();

        follower.followTrajectory(trajectory);
        while (opModeIsActive() && follower.isFollowing()) {
            Pose2d currentPose = drive.getPoseEstimate();

            Log.d(Constants.LOG_TAG, "Pose: " + currentPose.getX() + ", " + currentPose.getY() + ", " + currentPose.getHeading());
            Log.d(Constants.LOG_TAG, "Remaining duration: " + trajectory.duration());

            //TelemetryPacket packet = new TelemetryPacket();
            //Canvas fieldOverlay = packet.fieldOverlay();
            //fieldOverlay.setStroke("green");
            //DashboardUtil.drawSampledTrajectory(fieldOverlay, trajectory);
            //fieldOverlay.setFill("blue");
            //fieldOverlay.fillCircle(currentPose.getX(), currentPose.getY(), 3);
            //dashboard.sendTelemetryPacket(packet);

            follower.update(currentPose);
            drive.updatePoseEstimate();
        }
    }
}
