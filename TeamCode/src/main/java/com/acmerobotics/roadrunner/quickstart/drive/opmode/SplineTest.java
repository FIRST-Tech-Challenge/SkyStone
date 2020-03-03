package com.acmerobotics.roadrunner.quickstart.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.acmerobotics.roadrunner.quickstart.drive.mecanum.SampleMecanumDriveBase;
import com.acmerobotics.roadrunner.quickstart.drive.mecanum.SampleMecanumDriveREV;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
@Disabled
public class SplineTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDriveREV drive = new SampleMecanumDriveREV(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        Trajectory traj = drive.trajectoryBuilder(new Pose2d())
                .splineTo(new Pose2d(30, 30, 0))
                .build();

        drive.followTrajectory(traj);

        sleep(2000);

        drive.followTrajectory(
                drive.trajectoryBuilder(new Pose2d(30, 30, Math.toRadians(180)), true)
                        .splineTo(new Pose2d(0, 0, Math.toRadians(180)))
                        .build()
        );
    }
}
