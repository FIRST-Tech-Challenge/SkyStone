package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.path.heading.LinearInterpolator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.drive.mecanum.SampleMecanumDriveREVOptimized;

@Autonomous(name = "Linear Interp Test")
public class LinearInterpolatorTest extends LinearOpMode {

    SampleMecanumDriveREVOptimized drive;

    public void runOpMode(){
        drive = new SampleMecanumDriveREVOptimized(hardwareMap);

        waitForStart();
        drive.setPoseEstimate(new Pose2d(0,0,90));
        drive.followTrajectorySync(drive.trajectoryBuilder().splineTo(new Pose2d(0,20,Math.toRadians(-90)), new LinearInterpolator(Math.toRadians(90),Math.toRadians(180))).build());
    }
}
