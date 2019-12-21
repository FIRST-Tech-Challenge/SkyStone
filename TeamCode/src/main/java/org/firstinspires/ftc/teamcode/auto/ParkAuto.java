package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.auto.InformationAuto;
import org.firstinspires.ftc.teamcode.hardware.Intake;
import org.firstinspires.ftc.teamcode.hardware.drive.mecanum.SampleMecanumDriveREVOptimized;
import org.firstinspires.ftc.teamcode.paths.LoadingZoneToFarSkybridge;

@Autonomous(name = "Park Auto")
public class ParkAuto extends LinearOpMode {

    private Intake intake;
    private SampleMecanumDriveREVOptimized drive;
    public void runOpMode(){
        drive = SampleMecanumDriveREVOptimized.getInstance(hardwareMap);
        intake = Intake.getInstance(hardwareMap);

        if(InformationAuto.ifRedAlliance()){
            drive.setPoseEstimate(new Pose2d(-36,-63,Math.toRadians(90)));
        } else {
            drive.setPoseEstimate(new Pose2d(-36,63,Math.toRadians(-90)));
        }
        waitForStart();
        intake.release();

        drive.followTrajectory(new LoadingZoneToFarSkybridge(InformationAuto.ifRedAlliance(),drive).toTrajectory());

        while(!isStopRequested()){
            drive.update();
        }

    }

}
