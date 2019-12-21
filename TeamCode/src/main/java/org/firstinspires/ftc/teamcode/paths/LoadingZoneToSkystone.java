package org.firstinspires.ftc.teamcode.paths;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.vision.SkystonePosition;
import org.firstinspires.ftc.teamcode.hardware.drive.mecanum.SampleMecanumDriveREVOptimized;

public class LoadingZoneToSkystone {

    private boolean redAlliance;
    private SampleMecanumDriveREVOptimized drive;

    public LoadingZoneToSkystone(boolean redAlliance, SampleMecanumDriveREVOptimized drive){
        this.redAlliance=redAlliance;
        this.drive = drive;
    }


    public Trajectory toTrajectory (SkystonePosition.Positions skystonePosition){
        if(redAlliance){
            if(skystonePosition == SkystonePosition.Positions.LEFT){
                return drive.trajectoryBuilder().splineTo(new Pose2d(-45,-36,Math.toRadians(90))).build();
            } else if(skystonePosition == SkystonePosition.Positions.MIDDLE){
                return drive.trajectoryBuilder().splineTo(new Pose2d(-33,-36,Math.toRadians(90))).build();
            } else if(skystonePosition == SkystonePosition.Positions.RIGHT){
                return drive.trajectoryBuilder().splineTo(new Pose2d(-25,-36,Math.toRadians(90))).build();
            }

        } else {
            if(skystonePosition == SkystonePosition.Positions.RIGHT){
                return drive.trajectoryBuilder().splineTo(new Pose2d(-45,36,Math.toRadians(-90))).build();
            } else if(skystonePosition == SkystonePosition.Positions.MIDDLE){
                return drive.trajectoryBuilder().splineTo(new Pose2d(-33,36,Math.toRadians(-90))).build();
            } else if(skystonePosition == SkystonePosition.Positions.LEFT){
                return drive.trajectoryBuilder().splineTo(new Pose2d(-25,36,Math.toRadians(-90))).build();
            }
        }
        return drive.trajectoryBuilder().splineTo(new Pose2d(-45,-29,Math.toRadians(90))).build();
    }
}

