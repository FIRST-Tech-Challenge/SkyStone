package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.BaseTrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.PID.DriveConstantsPID;
import org.firstinspires.ftc.teamcode.PID.mecanum.SampleMecanumDriveBase;

public class Path {
    Pose2d startingPos;
    SampleMecanumDriveBase drive;
    BaseTrajectoryBuilder builder;
    Trajectory trajectory;

    public Path(SampleMecanumDriveBase drive, Pose2d startingPos){
        this.drive = drive;
        this.startingPos = startingPos;
        drive.setPoseEstimate(startingPos);
        builder = new TrajectoryBuilder(drive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);;
    }

    public void RedQuary(int[] skystonePositions){
        switch(skystonePositions[0]){
            case 1:
                builder = builder.splineTo(new Pose2d(new Vector2d(-49.728, 37.824), Math.toRadians(0)))
                        .lineTo(new Vector2d(-49.728, 24.576));
                drive.turnSync(Math.toRadians(90));
                break;
            case 2:
                break;
            case 3:
                break;
        }
        trajectory = builder.build();
        drive.followTrajectorySync(trajectory);
    }

    public void RedFoundation(){
        trajectory = builder.build();
        drive.followTrajectorySync(trajectory);
    }

    public void BlueQuary(int[] skystonePositions){
        switch(skystonePositions[0]){
            case 1:
                builder = builder.splineTo(new Pose2d(new Vector2d(-28,9), Math.toRadians(270))).splineTo(new Pose2d(new Vector2d(-28,50), Math.toRadians(0)));
                trajectory = builder.build();
                drive.followTrajectory(trajectory);

                drive.turnSync(-180);
                break;
            case 2:
                break;
            case 3:
                break;
        }
        trajectory = builder.build();
        drive.followTrajectorySync(trajectory);
    }

    public void BlueFoundation(){
        trajectory = builder.build();
        drive.followTrajectorySync(trajectory);
    }
}
