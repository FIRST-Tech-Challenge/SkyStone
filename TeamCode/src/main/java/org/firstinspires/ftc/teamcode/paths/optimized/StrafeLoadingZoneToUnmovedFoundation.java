package org.firstinspires.ftc.teamcode.paths.optimized;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.path.heading.ConstantInterpolator;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.hardware.drive.mecanum.SampleMecanumDriveREVOptimized;
import org.firstinspires.ftc.teamcode.vision.SkystonePosition;

public class StrafeLoadingZoneToUnmovedFoundation {

    private boolean redAlliance;
    private SampleMecanumDriveREVOptimized drive;


    public StrafeLoadingZoneToUnmovedFoundation(boolean redAlliance, SampleMecanumDriveREVOptimized drive){
        this.redAlliance=redAlliance;
        this.drive = drive;
    }

    public Trajectory toTrajectory (SkystonePosition skystonePosition){
        if(redAlliance){
            if(skystonePosition.equals(SkystonePosition.Positions.LEFT)){
                return drive.trajectoryBuilder().splineTo(new Pose2d(-67,-40,90),new ConstantInterpolator(90)).splineTo(new Pose2d(50,-40,90), new ConstantInterpolator(90)).splineTo(new Pose2d(50,-30,90), new ConstantInterpolator(90)).build();
            } else if(skystonePosition.equals(SkystonePosition.Positions.MIDDLE)){
                return drive.trajectoryBuilder().splineTo(new Pose2d(-59,-40,90), new ConstantInterpolator(90)).splineTo(new Pose2d(50,-40,90), new ConstantInterpolator(90)).splineTo(new Pose2d(50,-30,90), new ConstantInterpolator(90)).build();
            } else if(skystonePosition.equals(SkystonePosition.Positions.RIGHT)){
                return drive.trajectoryBuilder().splineTo(new Pose2d(-51,-40,90), new ConstantInterpolator(90)).splineTo(new Pose2d(50,-40,90), new ConstantInterpolator(90)).splineTo(new Pose2d(50,-30,90), new ConstantInterpolator(90)).build();
            }

        } else {
            if(skystonePosition.equals(SkystonePosition.Positions.LEFT)){
                return drive.trajectoryBuilder().splineTo(new Pose2d(67,40,-90), new ConstantInterpolator(-90)).splineTo(new Pose2d(50,40,-90), new ConstantInterpolator(-90)).splineTo(new Pose2d(50,30,-90), new ConstantInterpolator(-90)).build();
            } else if(skystonePosition.equals(SkystonePosition.Positions.MIDDLE)){
                return drive.trajectoryBuilder().splineTo(new Pose2d(59,40,-90), new ConstantInterpolator(-90)).splineTo(new Pose2d(50,40,-90), new ConstantInterpolator(-90)).splineTo(new Pose2d(50,30,-90), new ConstantInterpolator(-90)).build();
            } else if(skystonePosition.equals(SkystonePosition.Positions.RIGHT)){
                return drive.trajectoryBuilder().splineTo(new Pose2d(51,40,-90), new ConstantInterpolator(-90)).splineTo(new Pose2d(50,40,-90), new ConstantInterpolator(-90)).splineTo(new Pose2d(50,30,-90), new ConstantInterpolator(-90)).build();
            }
        }
        return drive.trajectoryBuilder().splineTo(new Pose2d(-67,-30,90)).build();
    }

}
