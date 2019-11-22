package org.firstinspires.ftc.teamcode.paths;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.hardware.drive.mecanum.SampleMecanumDriveREVOptimized;

public class LoadingZoneToMovedFoundation {

    private boolean redAlliance;
    private SampleMecanumDriveREVOptimized drive;

    public LoadingZoneToMovedFoundation(boolean redAlliance, SampleMecanumDriveREVOptimized drive){
        this.redAlliance=redAlliance;
        this.drive = drive;
    }

    public Trajectory toTrajectory (){
        if(redAlliance){
            return drive.trajectoryBuilder().splineTo(new Pose2d(-34, -45, Math.toRadians(0))).splineTo(new Pose2d(3,-45,0)).splineTo(new Pose2d(52,-45,Math.toRadians(0))).build();
        }
        return drive.trajectoryBuilder().splineTo(new Pose2d(-34, 45, Math.toRadians(0))).splineTo(new Pose2d(3,45,0)).splineTo(new Pose2d(52,45,Math.toRadians(0))).build();
    }

}
