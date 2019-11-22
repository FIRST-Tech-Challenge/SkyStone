package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.auto.InformationAuto;
import org.firstinspires.ftc.teamcode.hardware.FoundationGrabber;
import org.firstinspires.ftc.teamcode.hardware.Intake;
import org.firstinspires.ftc.teamcode.hardware.drive.mecanum.SampleMecanumDriveREVOptimized;
import org.firstinspires.ftc.teamcode.paths.LoadingZoneToFarSkybridge;


@Autonomous (name = "Meet 0 Default")
@Disabled
public class Meet0Op extends LinearOpMode {

    Intake intake;
    FoundationGrabber foundationGrabber;
    SampleMecanumDriveREVOptimized drive;

    public void runOpMode(){
        intake = intake.getInstance(hardwareMap);
        foundationGrabber = new FoundationGrabber(hardwareMap);
        drive = new SampleMecanumDriveREVOptimized(hardwareMap);
        if(InformationAuto.ifRedAlliance()){
            drive.setPoseEstimate(new Pose2d(-36, -63, 90));
        } else {
            drive.setPoseEstimate(new Pose2d(-36, 63, -90));
        }

        waitForStart();
        if(!isStopRequested()){
            foundationGrabber.setCurrentPosition(FoundationGrabber.Positions.DOWN_RIGHT);
            intake.setHold();
            intake.setGrabbing();


            drive.followTrajectorySync(new LoadingZoneToFarSkybridge(InformationAuto.ifRedAlliance(),drive).toTrajectory());
            drive.followTrajectorySync(drive.trajectoryBuilder().forward(12).build());
        }
    }
}
