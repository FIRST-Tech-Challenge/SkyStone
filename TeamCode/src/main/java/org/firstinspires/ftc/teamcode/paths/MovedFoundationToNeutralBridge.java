package org.firstinspires.ftc.teamcode.paths;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.hardware.drive.mecanum.SampleMecanumDriveREVOptimized;

public class MovedFoundationToNeutralBridge {

    private boolean redAlliance;
    private SampleMecanumDriveREVOptimized drive;

    public MovedFoundationToNeutralBridge(boolean redAlliance, SampleMecanumDriveREVOptimized drive){
        this.redAlliance=redAlliance;
        this.drive = drive;
    }

    public Trajectory toTrajectory (){
        if(redAlliance){
            return drive.trajectoryBuilder().splineTo(new Pose2d(0,0,180)).build();
        }
        return drive.trajectoryBuilder().splineTo(new Pose2d(0,0,180)).build();
    }

}
