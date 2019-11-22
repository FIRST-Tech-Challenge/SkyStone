package org.firstinspires.ftc.teamcode.paths;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.path.heading.LinearInterpolator;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.hardware.drive.mecanum.SampleMecanumDriveREVOptimized;
import org.firstinspires.ftc.teamcode.vision.SkystonePosition;

public class FoundationToSecondSkystone {

    private boolean redAlliance;
    private SampleMecanumDriveREVOptimized drive;

    public FoundationToSecondSkystone(boolean redAlliance, SampleMecanumDriveREVOptimized drive){
        this.redAlliance=redAlliance;
        this.drive = drive;
    }


    public Trajectory toTrajectory (SkystonePosition.Positions skystonePosition){
        if(redAlliance){
            if(skystonePosition == SkystonePosition.Positions.RIGHT){
                return drive.trajectoryBuilder().reverse().splineTo(new Pose2d(20,-36,Math.toRadians(0))).splineTo(new Pose2d(-10, -36, 0)).splineTo(new Pose2d(-51,-50,Math.toRadians(90))).build();
            } else if(skystonePosition == SkystonePosition.Positions.MIDDLE){
                return drive.trajectoryBuilder().reverse().splineTo(new Pose2d(20,-36,Math.toRadians(0))).splineTo(new Pose2d(-10, -36, 0)).splineTo(new Pose2d(-59,-50,Math.toRadians(90))).build();
            } else if(skystonePosition == SkystonePosition.Positions.LEFT){
                return drive.trajectoryBuilder().reverse().splineTo(new Pose2d(20,-36,Math.toRadians(0))).splineTo(new Pose2d(-10, -36, 0)).splineTo(new Pose2d(-67,-50,Math.toRadians(90))).build();
            }

        } else {
            if(skystonePosition == SkystonePosition.Positions.LEFT){
                return drive.trajectoryBuilder().reverse().splineTo(new Pose2d(20,36,Math.toRadians(0))).splineTo(new Pose2d(-10, 36, 0)).splineTo(new Pose2d(-51,50,Math.toRadians(-90))).build();
            } else if(skystonePosition == SkystonePosition.Positions.MIDDLE){
                return drive.trajectoryBuilder().reverse().splineTo(new Pose2d(20,36,Math.toRadians(0))).splineTo(new Pose2d(-10, 36, 0)).splineTo(new Pose2d(-59,50,Math.toRadians(-90))).build();
            } else if(skystonePosition == SkystonePosition.Positions.RIGHT){
                return drive.trajectoryBuilder().reverse().splineTo(new Pose2d(20,36,Math.toRadians(0))).splineTo(new Pose2d(-10, 36, 0)).splineTo(new Pose2d(-67,50,Math.toRadians(-90))).build();
            }
        }
        return drive.trajectoryBuilder().splineTo(new Pose2d(-43,-24,90)).build();
    }

}
