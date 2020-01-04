package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.BaseTrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.All.DriveConstant;
import org.firstinspires.ftc.teamcode.All.HardwareMap;
import org.firstinspires.ftc.teamcode.Autonomous.Vision.Align;
import org.firstinspires.ftc.teamcode.PID.DriveConstantsPID;
import org.firstinspires.ftc.teamcode.PID.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.TeleOp.Teleop;
import org.firstinspires.ftc.teamcode.TeleOp.TeleopConstants;

import java.lang.reflect.Field;
import java.util.List;

import kotlin.Unit;
import kotlin.jvm.functions.Function0;

public class Path {
    private Pose2d startingPos;
    private SampleMecanumDriveBase drive;
    private BaseTrajectoryBuilder builder;
    private Trajectory trajectory;
    private Align align;
    private HardwareMap hwMap;
    private LinearOpMode opMode;
    private List<Recognition> tfod;

    public Path(HardwareMap hwMap, LinearOpMode opMode, SampleMecanumDriveBase drive, Pose2d startingPos) {
        this.drive = drive;
        this.startingPos = startingPos;
        this.hwMap = hwMap;
        this.opMode = opMode;
        align = new Align(hwMap, opMode, DcMotor.ZeroPowerBehavior.BRAKE);
        this.drive.getLocalizer().setPoseEstimate(startingPos);
        this.drive.getLocalizer().update();
    }

    public void RedQuary(int[] skystonePositions) {
        switch (skystonePositions[0]) {
            case 1:
                transferReset();
                initIntakeClaw();

                try{
                    Thread.sleep(500);
                } catch (Exception e){}

                hwMap.foundationLock.setPosition(TeleopConstants.foundationLockUnlock);
                hwMap.transferLock.setPosition(TeleopConstants.transferLockPosPlatform);

                builder = new TrajectoryBuilder(drive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);   //-34.752, -63.936
                if(DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {
                    builder = builder.strafeTo(new Vector2d(drive.getPoseEstimate().getX(), -63.936 + 5.0))
                            .strafeTo(new Vector2d(drive.getPoseEstimate().getX(), -63.936 + 5.0 * 2))
                            .strafeTo(new Vector2d(drive.getPoseEstimate().getX(), -63.936 + 5.0 * 3))
                            .strafeTo(new Vector2d(drive.getPoseEstimate().getX(), -63.936 + 5.0 * 4))
                            .strafeTo(new Vector2d(drive.getPoseEstimate().getX(), -63.936 + 5.0 * 5))
                            .setReversed(true).lineTo(new Vector2d(-42, -63.936 + 5.0 * 5));
                } else {
                    builder = builder.strafeTo(new Vector2d(drive.getPoseEstimate().getX(),-30)).setReversed(true)
                            .lineTo(new Vector2d(-50, -30));
                }
                trajectory = builder.build();   //x - 2.812, y + 7.984
                drive.followTrajectorySync(trajectory);

                RobotLog.dd("STATUS", "Strafe #1 Done");

                try{
                    Thread.sleep(500);
                } catch (Exception e){}

                grabStone(FieldPosition.RED_QUARY);

                drive.getLocalizer().setPoseEstimate(new Pose2d(new Vector2d(drive.getPoseEstimate().getX(),
                        drive.getPoseEstimate().getY()), drive.getPoseEstimate().getHeading()));
                drive.getLocalizer().update();
                builder = new TrajectoryBuilder(drive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);
                if(DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {
                    builder = builder.setReversed(false).strafeTo(new Vector2d(-42, -45))
                            .lineTo(new Vector2d(42, -45)).strafeTo(new Vector2d(46,-35));
                } else {
                    builder = builder.setReversed(false).strafeTo(new Vector2d(-50, -40))
                            .lineTo(new Vector2d(50, -40)).strafeTo(new Vector2d(50, -30));
                }
                trajectory = builder.build();
                drive.followTrajectorySync(trajectory);

                RobotLog.dd("STATUS", "Strafe #2 Done");

                try{
                    Thread.sleep(500);
                } catch (Exception e){}

                dropStone(FieldPosition.RED_QUARY);

                drive.getLocalizer().setPoseEstimate(new Pose2d(new Vector2d(drive.getPoseEstimate().getX(),
                        drive.getPoseEstimate().getY()), drive.getPoseEstimate().getHeading()));
                drive.getLocalizer().update();
                builder = new TrajectoryBuilder(drive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);
                if(DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {
                    builder = builder.strafeTo(new Vector2d(46, -40)).setReversed(true).lineTo(new Vector2d(-24, -40))
                            .strafeTo(new Vector2d(-24, -30));
                } else {
                    builder = builder.strafeTo(new Vector2d(50, -40)).setReversed(true).lineTo(new Vector2d(-20, -40))
                            .strafeTo(new Vector2d(-20, -28));
                }
                trajectory = builder.build();
                drive.followTrajectorySync(trajectory);

                RobotLog.dd("STATUS", "Strafe #3 Done");

                try{
                    Thread.sleep(500);
                } catch (Exception e){}

                grabStone(FieldPosition.RED_QUARY);

                drive.getLocalizer().setPoseEstimate(new Pose2d(new Vector2d(drive.getPoseEstimate().getX(),
                        drive.getPoseEstimate().getY()), drive.getPoseEstimate().getHeading()));
                drive.getLocalizer().update();
                builder = new TrajectoryBuilder(drive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);
                if(DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {
                    builder = builder.setReversed(false).strafeTo(new Vector2d(-24, -40)).lineTo(new Vector2d(42, -40))
                            .strafeTo(new Vector2d(42, -30));
                } else {
                    builder = builder.setReversed(false).strafeTo(new Vector2d(-20, -40)).lineTo(new Vector2d(50, -40))
                            .strafeTo(new Vector2d(50, -30));
                }
                trajectory = builder.build();
                drive.followTrajectorySync(trajectory);

                RobotLog.dd("STATUS", "Strafe #4 Done");

                try{
                    Thread.sleep(500);
                } catch (Exception e){}

                dropStone(FieldPosition.RED_QUARY);

                break;
            case 2:
                transferReset();
                initIntakeClaw();
                hwMap.foundationLock.setPosition(TeleopConstants.foundationLockUnlock);
                hwMap.transferLock.setPosition(TeleopConstants.transferLockPosPlatform);

                builder = new TrajectoryBuilder(drive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);

                builder = builder.lineTo(new Vector2d(drive.getLocalizer().getPoseEstimate().getX(), drive.getLocalizer().getPoseEstimate().getY() + 5))
                        .splineTo(new Pose2d(new Vector2d(-33.208, -39.672), Math.toRadians(80)));
                trajectory = builder.build();
                drive.followTrajectorySync(trajectory);

                openPlatform();

                drive.getLocalizer().setPoseEstimate(new Pose2d(new Vector2d(-33.728, -39.672), Math.toRadians(80)));
                drive.getLocalizer().update();
                builder = new TrajectoryBuilder(drive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);
                builder = builder.lineTo(new Vector2d(-27.728, -11.52));
                trajectory = builder.build();
                drive.followTrajectorySync(trajectory);

                prepStone(TeleopConstants.stoneEncoderValues[0] - 300);

                drive.getLocalizer().setPoseEstimate(new Pose2d(new Vector2d(-27.728, -11.52), Math.toRadians(80)));
                drive.getLocalizer().update();
                builder = new TrajectoryBuilder(drive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);
                builder = builder.setReversed(true).lineTo(new Vector2d(-34.728, -40.672))
                        .setReversed(false).splineTo(new Pose2d(new Vector2d(-5.568, -24.44), Math.toRadians(0)))
                        .splineTo(new Pose2d(new Vector2d(55.488, -30.296), Math.toRadians(-90)));
                trajectory = builder.build();
                drive.followTrajectorySync(trajectory);

                align.setPower(0.13, 0.25);
                align.foundation(FieldPosition.RED_QUARY);
                intake(0);
                transferReset();

                drive.getLocalizer().setPoseEstimate(new Pose2d(new Vector2d(58.144, -18.128), drive.getExternalHeading()));
                drive.getLocalizer().update();
                builder = new TrajectoryBuilder(drive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);
                builder = builder.splineTo(new Pose2d(new Vector2d(45.064, -53.72), Math.toRadians(220)))
                        .setReversed(true).lineTo(new Vector2d(71.0, -53.72)).setReversed(false);
                trajectory = builder.build();
                drive.followTrajectorySync(trajectory);

                dropStone(TeleopConstants.stoneEncoderValues[0]);
                hwMap.foundationLock.setPosition(TeleopConstants.foundationLockUnlock);
                hwMap.transferLock.setPosition(TeleopConstants.transferLockPosPlatform);

                try {
                    Thread.sleep(300);
                } catch (Exception e) {
                }
                transferReset();
                intake(1);

                drive.getLocalizer().setPoseEstimate(new Pose2d(new Vector2d(71.0, -53.72), Math.toRadians(180)));
                drive.getLocalizer().update();
                builder = new TrajectoryBuilder(drive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);
                builder = builder.setReversed(false).lineTo(new Vector2d(67.0, -53.72)).strafeTo(new Vector2d(67.0, -31.44))
                        .lineTo(new Vector2d(-8.552, -31.44)).strafeTo(new Vector2d(-8.552, -3.44))  ////
                        .lineTo(new Vector2d(-16.552, -3.44));
                trajectory = builder.build();
                drive.followTrajectorySync(trajectory);

                prepStone();

                drive.getLocalizer().setPoseEstimate(new Pose2d(new Vector2d(-16.552, -3.44), drive.getExternalHeading()));
                drive.getLocalizer().update();
                builder = new TrajectoryBuilder(drive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);
                builder = builder.setReversed(true).strafeTo(new Vector2d(-16.552, -40.44));
                        //.lineTo(new Vector2d(77.0, -48.44));
                trajectory = builder.build();
                drive.followTrajectorySync(trajectory);

                driveTime(-0.75, 1150);
                dropStone();
                try{
                    Thread.sleep(500);
                } catch (Exception e){}
               // driveTime(1, 800);

                intake(0);
                break;
            case 3:
                transferReset();
                initIntakeClaw();
                hwMap.foundationLock.setPosition(TeleopConstants.foundationLockUnlock);
                hwMap.transferLock.setPosition(TeleopConstants.transferLockPosPlatform);

                builder = new TrajectoryBuilder(drive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);

                builder = builder.lineTo(new Vector2d(drive.getLocalizer().getPoseEstimate().getX(), drive.getLocalizer().getPoseEstimate().getY() + 5))
                        .splineTo(new Pose2d(new Vector2d(-42.728, -39.672), Math.toRadians(85)));
                trajectory = builder.build();
                drive.followTrajectorySync(trajectory);

                openPlatform();

                drive.getLocalizer().setPoseEstimate(new Pose2d(new Vector2d(-42.728, -39.672), Math.toRadians(85)));
                drive.getLocalizer().update();
                builder = new TrajectoryBuilder(drive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);
                builder = builder.lineTo(new Vector2d(-37.728, -11.52));
                trajectory = builder.build();
                drive.followTrajectorySync(trajectory);

                prepStone(TeleopConstants.stoneEncoderValues[0] - 300);

                drive.getLocalizer().setPoseEstimate(new Pose2d(new Vector2d(-35.728, -11.52), Math.toRadians(85)));
                drive.getLocalizer().update();
                builder = new TrajectoryBuilder(drive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);
                builder = builder.setReversed(true).lineTo(new Vector2d(-39.728, -42.672))
                        .setReversed(false).splineTo(new Pose2d(new Vector2d(-5.568, -24.44), Math.toRadians(0)))
                        .splineTo(new Pose2d(new Vector2d(51.488, -30.296), Math.toRadians(-90)));
                trajectory = builder.build();
                drive.followTrajectorySync(trajectory);

                align.setPower(0.13, 0.25);
                align.foundation(FieldPosition.RED_QUARY);
                intake(0);
                transferReset();

                drive.getLocalizer().setPoseEstimate(new Pose2d(new Vector2d(51.144, -25.128), drive.getExternalHeading()));
                drive.getLocalizer().update();
                builder = new TrajectoryBuilder(drive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);
                builder = builder.splineTo(new Pose2d(new Vector2d(29.064, -55.72), Math.toRadians(220)))
                        .setReversed(true).lineTo(new Vector2d(68.0, -55.72)).setReversed(false);
                trajectory = builder.build();
                drive.followTrajectorySync(trajectory);

                dropStone(TeleopConstants.stoneEncoderValues[0]);
                hwMap.foundationLock.setPosition(TeleopConstants.foundationLockUnlock);
                hwMap.transferLock.setPosition(TeleopConstants.transferLockPosPlatform);

                try {
                    Thread.sleep(300);
                } catch (Exception e) {
                }
                transferReset();
                intake(1);

                drive.getLocalizer().setPoseEstimate(new Pose2d(new Vector2d(68.0, -55.72), Math.toRadians(180)));
                drive.getLocalizer().update();
                builder = new TrajectoryBuilder(drive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);
                builder = builder.setReversed(false).lineTo(new Vector2d(64.0, -55.72)).strafeTo(new Vector2d(64.0, -47.44))
                        .lineTo(new Vector2d(-18.552, -47.44)).strafeTo(new Vector2d(-18.552, -28.44))
                        .lineTo(new Vector2d(-26.552, -28.44));
                trajectory = builder.build();
                drive.followTrajectorySync(trajectory);

                prepStone();

                drive.getLocalizer().setPoseEstimate(new Pose2d(new Vector2d(-26.552, -28.44), drive.getExternalHeading()));
                drive.getLocalizer().update();
                builder = new TrajectoryBuilder(drive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);
                builder = builder.setReversed(true).strafeTo(new Vector2d(-26.552, -55.44));
                        //.lineTo(new Vector2d(77.0, -55.44));
                trajectory = builder.build();
                drive.followTrajectorySync(trajectory);

                driveTime(-0.75, 1150);
                dropStone();
                try{
                    Thread.sleep(500);
                } catch (Exception e){}
                //driveTime(1, 800);

                intake(0);
                break;
        }
    }

    public void RedFoundationPark() {
        transferReset();
        initIntakeClaw();
        try {
            Thread.sleep(5000);
        } catch (Exception e){}
        drive.getLocalizer().setPoseEstimate(startingPos);
        drive.getLocalizer().update();
        builder = new TrajectoryBuilder(drive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);
        builder = builder.strafeRight(9).setReversed(false).forward(36);
        trajectory = builder.build();
        drive.followTrajectorySync(trajectory);

        intake(0);

        try{
            Thread.sleep(5000);
        } catch (Exception e){}

        intake(0);
    }

    public void BlueQuary(int[] skystonePositions) {    // (-x, y)
        switch (skystonePositions[0]) {
            case 1:
                transferReset();
                initIntakeClaw();

                try{
                    Thread.sleep(500);
                } catch (Exception e){}

                hwMap.foundationLock.setPosition(TeleopConstants.foundationLockUnlock);
                hwMap.transferLock.setPosition(TeleopConstants.transferLockPosPlatform);

                builder = new TrajectoryBuilder(drive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);

                builder = builder.lineTo(new Vector2d(drive.getLocalizer().getPoseEstimate().getX(), drive.getLocalizer().getPoseEstimate().getY() - 5))
                        .splineTo(new Pose2d(new Vector2d(-19.832, 39.672), Math.toRadians(-100)));
                trajectory = builder.build();
                drive.followTrajectorySync(trajectory);

                openPlatform();

                drive.getLocalizer().setPoseEstimate(new Pose2d(new Vector2d(-19.832, 39.672), Math.toRadians(-100)));
                drive.getLocalizer().update();
                builder = new TrajectoryBuilder(drive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);
                builder = builder.lineTo(new Vector2d(-17.728, 22.52));
                trajectory = builder.build();
                drive.followTrajectorySync(trajectory);

                prepStone(TeleopConstants.stoneEncoderValues[0] - 300);

                drive.getLocalizer().setPoseEstimate(new Pose2d(new Vector2d(-15.728, 22.52), Math.toRadians(-100)));
                drive.getLocalizer().update();
                builder = new TrajectoryBuilder(drive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);
                builder = builder.setReversed(true).lineTo(new Vector2d(-15.728, 36.672))
                        .setReversed(false).splineTo(new Pose2d(new Vector2d(-5.568, 34.44), Math.toRadians(0)))
                        .splineTo(new Pose2d(new Vector2d(68.488, 48.296), Math.toRadians(90)));
                trajectory = builder.build();
                drive.followTrajectorySync(trajectory);

                align.setPower(0.13, 0.25);
                align.foundation(FieldPosition.BLUE_QUARY);
                intake(0);
                transferReset();

                drive.getLocalizer().setPoseEstimate(new Pose2d(new Vector2d(68.144, 16.128), drive.getExternalHeading()));
                drive.getLocalizer().update();
                builder = new TrajectoryBuilder(drive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);
                builder = builder.splineTo(new Pose2d(new Vector2d(37.064, 58.72), Math.toRadians(140)))
                        .setReversed(true).lineTo(new Vector2d(70.0, 58.72)).setReversed(false);
                trajectory = builder.build();
                drive.followTrajectorySync(trajectory);

                dropStone(TeleopConstants.stoneEncoderValues[0]);
                hwMap.foundationLock.setPosition(TeleopConstants.foundationLockUnlock);
                hwMap.transferLock.setPosition(TeleopConstants.transferLockPosPlatform);

                try {
                    Thread.sleep(300);
                } catch (Exception e) {
                }
                transferReset();
                intake(1);

                drive.getLocalizer().setPoseEstimate(new Pose2d(new Vector2d(70.0, 58.72), Math.toRadians(0)));
                drive.getLocalizer().update();
                builder = new TrajectoryBuilder(drive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);
                builder = builder.setReversed(false).lineTo(new Vector2d(66.0, 58.72)).strafeTo(new Vector2d(66.0, 32.44))
                        .lineTo(new Vector2d(20, 32.44));
                trajectory = builder.build();
                drive.followTrajectorySync(trajectory);
                /*.strafeTo(new Vector2d(-1.552, 6.44))  ////-1.552
                        .lineTo(new Vector2d(-8.552, 6.44));
                trajectory = builder.build();
                drive.followTrajectorySync(trajectory);

                prepStone();

                drive.getLocalizer().setPoseEstimate(new Pose2d(new Vector2d(-8.552, 6.44), drive.getExternalHeading()));
                drive.getLocalizer().update();
                builder = new TrajectoryBuilder(drive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);
                builder = builder.setReversed(true).strafeTo(new Vector2d(-8.552, 43.44));
                        //.lineTo(new Vector2d(77.0, 43.44));
                trajectory = builder.build();
                drive.followTrajectorySync(trajectory);

                //driveTime(-1, 1000);
                driveTime(-0.75, 1000);
                dropStone();
                try{
                    Thread.sleep(500);
                } catch (Exception e){}
                //driveTime(1, 800);*/

                intake(0);
                break;
            case 2:
                transferReset();
                initIntakeClaw();

                try{
                    Thread.sleep(500);
                } catch (Exception e){}

                hwMap.foundationLock.setPosition(TeleopConstants.foundationLockUnlock);
                hwMap.transferLock.setPosition(TeleopConstants.transferLockPosPlatform);

                builder = new TrajectoryBuilder(drive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);

                builder = builder.lineTo(new Vector2d(drive.getLocalizer().getPoseEstimate().getX(), drive.getLocalizer().getPoseEstimate().getY() - 5))
                        .splineTo(new Pose2d(new Vector2d(-32.832, 39.672), Math.toRadians(-100)));
                trajectory = builder.build();
                drive.followTrajectorySync(trajectory);

                openPlatform();

                drive.getLocalizer().setPoseEstimate(new Pose2d(new Vector2d(-32.832, 39.672), Math.toRadians(-100)));
                drive.getLocalizer().update();
                builder = new TrajectoryBuilder(drive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);
                builder = builder.lineTo(new Vector2d(-28.728, 18.52));
                trajectory = builder.build();
                drive.followTrajectorySync(trajectory);

                prepStone(TeleopConstants.stoneEncoderValues[0] - 300);

                drive.getLocalizer().setPoseEstimate(new Pose2d(new Vector2d(-28.728, 22.52), Math.toRadians(-100)));
                drive.getLocalizer().update();
                builder = new TrajectoryBuilder(drive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);
                builder = builder.setReversed(true).lineTo(new Vector2d(-28.728, 36.672))
                        .setReversed(false).splineTo(new Pose2d(new Vector2d(-5.568, 34.44), Math.toRadians(0)))
                        .splineTo(new Pose2d(new Vector2d(67.488, 46.296), Math.toRadians(90)));
                trajectory = builder.build();
                drive.followTrajectorySync(trajectory);

                align.setPower(0.13, 0.25);
                align.foundation(FieldPosition.BLUE_QUARY);
                intake(0);
                transferReset();

                drive.getLocalizer().setPoseEstimate(new Pose2d(new Vector2d(67.144, 16.128), drive.getExternalHeading()));
                drive.getLocalizer().update();
                builder = new TrajectoryBuilder(drive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);
                builder = builder.splineTo(new Pose2d(new Vector2d(37.064, 52.72), Math.toRadians(140)))
                        .setReversed(true).lineTo(new Vector2d(68.0, 52.72)).setReversed(false);
                trajectory = builder.build();
                drive.followTrajectorySync(trajectory);

                dropStone(TeleopConstants.stoneEncoderValues[0]);
                hwMap.foundationLock.setPosition(TeleopConstants.foundationLockUnlock);
                hwMap.transferLock.setPosition(TeleopConstants.transferLockPosPlatform);

                try {
                    Thread.sleep(300);
                } catch (Exception e) {
                }
                transferReset();
                intake(1);

                drive.getLocalizer().setPoseEstimate(new Pose2d(new Vector2d(73.0, 50.72), Math.toRadians(0)));
                drive.getLocalizer().update();
                builder = new TrajectoryBuilder(drive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);
                builder = builder.setReversed(false).lineTo(new Vector2d(65.0, 50.72)).strafeTo(new Vector2d(65.0, 37.44))
                        .lineTo(new Vector2d(20.552, 37.44));
                trajectory = builder.build();
                drive.followTrajectorySync(trajectory);
                /*.strafeTo(new Vector2d(-16.552, 10.44))  ////-16.552
                        .lineTo(new Vector2d(-24.552, 10.44));
                trajectory = builder.build();
                drive.followTrajectorySync(trajectory);

                prepStone();

                drive.getLocalizer().setPoseEstimate(new Pose2d(new Vector2d(-24.552, 10.44), drive.getExternalHeading()));
                drive.getLocalizer().update();
                builder = new TrajectoryBuilder(drive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);
                builder = builder.setReversed(true).strafeTo(new Vector2d(-24.552, 53.44));
                        //.lineTo(new Vector2d(77.0, 43.44));
                trajectory = builder.build();
                drive.followTrajectorySync(trajectory);

                //driveTime(-1, 1150);
                driveTime(-0.75, 1100);
                dropStone();
                try{
                    Thread.sleep(500);
                } catch (Exception e){}*/
                //driveTime(1, 800);

                intake(0);
                break;
            case 3:
                transferReset();
                initIntakeClaw();

                hwMap.foundationLock.setPosition(TeleopConstants.foundationLockUnlock);
                hwMap.transferLock.setPosition(TeleopConstants.transferLockPosPlatform);

                builder = new TrajectoryBuilder(drive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);

                builder = builder.lineTo(new Vector2d(drive.getLocalizer().getPoseEstimate().getX(), drive.getLocalizer().getPoseEstimate().getY() - 5))
                        .splineTo(new Pose2d(new Vector2d(-48.832, 39.672), Math.toRadians(-100)));
                trajectory = builder.build();
                drive.followTrajectorySync(trajectory);

                openPlatform();

                drive.getLocalizer().setPoseEstimate(new Pose2d(new Vector2d(-48.832, 39.672), Math.toRadians(-100)));
                drive.getLocalizer().update();
                builder = new TrajectoryBuilder(drive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);
                builder = builder.lineTo(new Vector2d(-46.728, 18.52));
                trajectory = builder.build();
                drive.followTrajectorySync(trajectory);

                prepStone(TeleopConstants.stoneEncoderValues[0] - 300);

                drive.getLocalizer().setPoseEstimate(new Pose2d(new Vector2d(-48.728, 18.52), Math.toRadians(-100)));
                drive.getLocalizer().update();
                builder = new TrajectoryBuilder(drive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);
                builder = builder.setReversed(true).lineTo(new Vector2d(-49.728, 27.672))
                        .setReversed(false).splineTo(new Pose2d(new Vector2d(-5.568, 25.44), Math.toRadians(0)))
                        .splineTo(new Pose2d(new Vector2d(51.488, 42.296), Math.toRadians(90)));
                trajectory = builder.build();
                drive.followTrajectorySync(trajectory);

                align.setPower(0.13, 0.25);
                align.foundation(FieldPosition.BLUE_QUARY);
                intake(0);
                transferReset();

                drive.getLocalizer().setPoseEstimate(new Pose2d(new Vector2d(51.144, 16.128), drive.getExternalHeading()));
                drive.getLocalizer().update();
                builder = new TrajectoryBuilder(drive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);
                builder = builder.splineTo(new Pose2d(new Vector2d(38.064, 54.72), Math.toRadians(140)))
                        .setReversed(true).lineTo(new Vector2d(60.0, 54.72)).setReversed(false);
                trajectory = builder.build();
                drive.followTrajectorySync(trajectory);

                dropStone(TeleopConstants.stoneEncoderValues[0]);
                hwMap.foundationLock.setPosition(TeleopConstants.foundationLockUnlock);
                hwMap.transferLock.setPosition(TeleopConstants.transferLockPosOut);

                try {
                    Thread.sleep(300);
                } catch (Exception e) {
                }
                transferReset();
                intake(1);

                drive.getLocalizer().setPoseEstimate(new Pose2d(new Vector2d(60.0, 54.72), Math.toRadians(0)));
                drive.getLocalizer().update();
                builder = new TrajectoryBuilder(drive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);
                builder = builder.setReversed(false).lineTo(new Vector2d(54.0, 54.72)).strafeTo(new Vector2d(54.0, 35.44))
                        .lineTo(new Vector2d(17.552, 35.44));
                trajectory = builder.build();
                drive.followTrajectorySync(trajectory);/*.strafeTo(new Vector2d(-25.552, 10.44))  ////
                        .lineTo(new Vector2d(-33.552, 10.44));//.strafeTo(new Vector2d(-33.552, 43.44));
                trajectory = builder.build();
                drive.followTrajectorySync(trajectory);

                prepStone();

                drive.getLocalizer().setPoseEstimate(new Pose2d(new Vector2d(-33.552, 10.44), drive.getExternalHeading()));
                drive.getLocalizer().update();
                builder = new TrajectoryBuilder(drive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);
                builder = builder.setReversed(true).strafeTo(new Vector2d(-33.522, 43.44));
                        //.lineTo(new Vector2d(77.0, 43.44));
                trajectory = builder.build();
                drive.followTrajectorySync(trajectory);


                //driveTime(-1, 1000);
                driveTime(-0.75, 1150);
                dropStone();
                try{
                    Thread.sleep(500);
                } catch (Exception e){}
                //driveTime(1, 800);*/

                intake(0);
                break;
        }
    }

    public void BlueFoundationPark() {
        transferReset();
        initIntakeClaw();
        try {
            Thread.sleep(5000);
        } catch (Exception e){}
        drive.getLocalizer().setPoseEstimate(startingPos);
        drive.getLocalizer().update();
        builder = new TrajectoryBuilder(drive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);
        builder = builder.strafeLeft(9).setReversed(false).forward(36);
        trajectory = builder.build();
        drive.followTrajectorySync(trajectory);

        intake(0);

        try{
            Thread.sleep(5000);
        } catch (Exception e){}

        intake(0);
    }

    public void BlueFoundationDrag(){
        transferReset();
        initIntakeClaw();
        startingPos = new Pose2d(new Vector2d(20.736, 63.936), Math.toRadians(270));

        try {
            Thread.sleep(5000);
        } catch (Exception e){}
        drive.getLocalizer().setPoseEstimate(startingPos);
        drive.getLocalizer().update();
        builder = new TrajectoryBuilder(drive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);
        builder = builder.lineTo(new Vector2d(20.736, 63.936)).lineTo(new Vector2d(20.736, 48.936))
            .strafeTo(new Vector2d(68.144, 48.936));
        trajectory = builder.build();
        drive.followTrajectorySync(trajectory);

        intake(0);
        align.setPower(0.13, 0.25);
        align.foundation(FieldPosition.BLUE_QUARY);
        transferReset();

        drive.getLocalizer().setPoseEstimate(new Pose2d(new Vector2d(68.144, 16.128), drive.getExternalHeading()));
        drive.getLocalizer().update();
        builder = new TrajectoryBuilder(drive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);
        builder = builder.splineTo(new Pose2d(new Vector2d(37.064, 58.72), Math.toRadians(140)))
                .setReversed(true).lineTo(new Vector2d(70.0, 58.72)).setReversed(false);
        trajectory = builder.build();
        drive.followTrajectorySync(trajectory);

        drive.getLocalizer().setPoseEstimate(new Pose2d(new Vector2d(60.0, 54.72), Math.toRadians(0)));
        drive.getLocalizer().update();
        builder = new TrajectoryBuilder(drive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);
        builder = builder.setReversed(false).lineTo(new Vector2d(54.0, 54.72)).strafeTo(new Vector2d(54.0, 60.44))
                .lineTo(new Vector2d(17.552, 60.44));
        trajectory = builder.build();
        drive.followTrajectorySync(trajectory);
    }

    public void updateTFODData(List<Recognition> tfod) {
        this.tfod = tfod;
        updateTFOD();
    }

    public Pose2d getPoseEstimate() {
        return drive.getLocalizer().getPoseEstimate();
    }

    public double getExternalHeading() {
        return drive.getExternalHeading();
    }

    private void updateTFOD() {
        align.updateTFOD(tfod);
    }

    public void updateHeading() {
        align.updateExternalHeading(Math.toDegrees(drive.getExternalHeading()));
    }
    // TODO : servo updates are non-blocking, this is unnecessary
    private void transferReset() {
        Thread thread = new Thread() {
            public void run() {
                hwMap.transferHorn.setPosition(TeleopConstants.transferHornPosReady);
                //hwMap.innerTransfer.setPosition(TeleopConstants.innerTransferPosBlock);
            }
        };
        thread.start();
    }
    // TODO: see above
    private void openPlatform(){
        Thread thread = new Thread(){
            public void run(){
                hwMap.foundationLock.setPosition(TeleopConstants.foundationLockUnlock);
                hwMap.transferLock.setPosition(TeleopConstants.transferLockPosOut);
            }
        };
        thread.run();
    }

    private void initIntakeClaw() {
        Thread thread = new Thread() {
            public void run() {
                hwMap.clawInit.setPosition(TeleopConstants.clawInitPosCapstone);
                hwMap.clawServo1.setPosition(TeleopConstants.clawServo1PosClose);
                hwMap.clawServo2.setPosition(TeleopConstants.clawServo2PosClose);
                //resetLift(TeleopConstants.liftPower);
                hwMap.intakeInit.setPosition(TeleopConstants.intakeInitPosLeft);
                try {
                    Thread.sleep(1000);
                } catch (Exception e) {
                }
                hwMap.intakeInit.setPosition(TeleopConstants.intakeInitPosRight);
                //intake(1);

                try {
                    Thread.sleep(1000);
                } catch (Exception e) {
                }

                hwMap.intakeInit.setPosition(TeleopConstants.intakeInitPosReset);

                try {
                    Thread.sleep(500);
                } catch (Exception e) {
                }
                //hwMap.clawInit.setPosition(TeleopConstants.clawInitPosReset);
                hwMap.clawInit.setPosition(TeleopConstants.clawInitPosReset);

                try {
                    Thread.sleep(500);
                } catch (Exception e) {
                }

                hwMap.clawInit.setPosition(TeleopConstants.clawInitPosCapstone);
            }
        };
        thread.start();
    }

    private void prepStone(double sliderEncoders) {
        Thread thread = new Thread() {
            public void run() {
                hwMap.foundationLock.setPosition(TeleopConstants.foundationLockUnlock);
                hwMap.transferLock.setPosition(TeleopConstants.transferLockPosOut);
                hwMap.clawServo1.setPosition(TeleopConstants.clawServo1Block);
                hwMap.clawServo2.setPosition(TeleopConstants.clawServo2PosOpen);
                //hwMap.innerTransfer.setPosition(TeleopConstants.innerTransferPosTucked);
                hwMap.transferHorn.setPosition(TeleopConstants.transferHornPosReady);
                try {
                    sleep(800);
                } catch (Exception e) {
                }
                hwMap.transferHorn.setPosition(TeleopConstants.transferHornPosPush);
                try {
                    sleep(800);
                } catch (Exception e) {
                }
                hwMap.transferHorn.setPosition(TeleopConstants.transferHornPosReady);
                try {
                    sleep(800);
                } catch (Exception e) {
                }
                hwMap.transferHorn.setPosition(TeleopConstants.transferHornPosPush);
                /*try {
                    sleep(1000);
                } catch (Exception e) {
                }
                hwMap.innerTransfer.setPosition(TeleopConstants.innerTransferPosExtended);*/
                try {
                    sleep(1000);
                } catch (Exception e) {
                }
                hwMap.clawServo1.setPosition(TeleopConstants.clawServo1PosClose);
                hwMap.clawServo2.setPosition(TeleopConstants.clawServo2PosClose);

                //while (Math.abs(hwMap.liftOne.getCurrentPosition()) < Math.abs(sliderEncoders) && opMode.opModeIsActive()) {
                    hwMap.liftOne.setPower(TeleopConstants.liftPower);
                    hwMap.liftTwo.setPower(TeleopConstants.liftPower);
                 /*   opMode.telemetry.addData("Current Encoder Counts", -hwMap.liftOne.getCurrentPosition());
                    opMode.telemetry.addData("Target Encoder Counts", Math.abs(sliderEncoders));
                    opMode.telemetry.update();
                }*/
                 try{
                    Thread.sleep(1800);
                } catch (Exception e){}

                hwMap.liftOne.setPower(0);
                hwMap.liftTwo.setPower(0);
            }
        };
        thread.start();
    }

    private void grabStone(FieldPosition fieldPosition){
        if(fieldPosition == FieldPosition.RED_QUARY || fieldPosition == FieldPosition.RED_FOUNDATION_PARK ||
            fieldPosition == FieldPosition.RED_FOUNDATION_DRAG) {
            hwMap.redAutoClawJoint2.setPosition(TeleopConstants.autoClaw2Open);
            try{
                Thread.sleep(300);
            } catch(Exception e){}
            hwMap.redAutoClawJoint1.setPosition(TeleopConstants.autoClaw1Down);
            try{
                Thread.sleep(500);
            } catch(Exception e){}
            hwMap.redAutoClawJoint2.setPosition(TeleopConstants.autoClaw2Close);
            try{
                Thread.sleep(300);
            } catch(Exception e){}
            hwMap.redAutoClawJoint1.setPosition(TeleopConstants.autoClaw1Up);
        } else {
            hwMap.blueAutoClawJoint2.setPosition(TeleopConstants.autoClaw2Open);
            try{
                Thread.sleep(300);
            } catch(Exception e){}
            hwMap.blueAutoClawJoint1.setPosition(TeleopConstants.autoClaw1Down);
            try{
                Thread.sleep(500);
            } catch(Exception e){}
            hwMap.blueAutoClawJoint2.setPosition(TeleopConstants.autoClaw2Close);
            try{
                Thread.sleep(300);
            } catch(Exception e){}
            hwMap.redAutoClawJoint1.setPosition(TeleopConstants.autoClaw1Up);
        }
    }

    private void dropStone(FieldPosition fieldPosition){
        if(fieldPosition == FieldPosition.RED_QUARY || fieldPosition == FieldPosition.RED_FOUNDATION_PARK ||
                fieldPosition == FieldPosition.RED_FOUNDATION_DRAG) {
            hwMap.redAutoClawJoint1.setPosition(TeleopConstants.autoClaw1Down);
            hwMap.redAutoClawJoint2.setPosition(TeleopConstants.autoClaw2Open);
            try{
                Thread.sleep(300);
            } catch(Exception e){}
            hwMap.redAutoClawJoint1.setPosition(TeleopConstants.autoClaw1Up);
        } else {
            hwMap.blueAutoClawJoint1.setPosition(TeleopConstants.autoClaw1Down);
            hwMap.blueAutoClawJoint2.setPosition(TeleopConstants.autoClaw2Open);
            try{
                Thread.sleep(300);
            } catch(Exception e){}
            hwMap.blueAutoClawJoint1.setPosition(TeleopConstants.autoClaw1Up);
        }
    }

    private void prepStone() {
        Thread thread = new Thread() {
            public void run() {
                hwMap.foundationLock.setPosition(TeleopConstants.foundationLockUnlock);
                hwMap.transferLock.setPosition(TeleopConstants.transferLockPosOut);
                hwMap.clawServo1.setPosition(TeleopConstants.clawServo1PosClose);
                hwMap.clawServo2.setPosition(TeleopConstants.clawServo2PosOpen);
                hwMap.transferHorn.setPosition(TeleopConstants.transferHornPosReady);
                try {
                    sleep(800);
                } catch (Exception e) {
                }
                hwMap.transferHorn.setPosition(TeleopConstants.transferHornPosPush);
                try {
                    sleep(800);
                } catch (Exception e) {
                }
                hwMap.transferHorn.setPosition(TeleopConstants.transferHornPosReady);
                try {
                    sleep(800);
                } catch (Exception e) {
                }
                hwMap.transferHorn.setPosition(TeleopConstants.transferHornPosPush);
                try {
                    sleep(1100);
                } catch (Exception e) {
                }
                //hwMap.clawServo1.setPosition(TeleopConstants.clawServo1PosClose);
                hwMap.clawServo2.setPosition(TeleopConstants.clawServo2PosClose);
                hwMap.liftOne.setPower(0);
                hwMap.liftTwo.setPower(0);
            }
        };
        thread.start();
    }

    private void dropStone(double sliderEncoders) {
        /*while ((Math.abs(hwMap.liftOne.getCurrentPosition()) > Math.abs(sliderEncoders) || hwMap.liftReset.getState()) &&
                opMode.opModeIsActive()) {
            hwMap.liftOne.setPower(-TeleopConstants.liftPower);
            hwMap.liftTwo.setPower(-TeleopConstants.liftPower);
            opMode.telemetry.addData("Current Encoder Counts", -hwMap.liftOne.getCurrentPosition());
            opMode.telemetry.addData("Target Encoder Counts", Math.abs(sliderEncoders));
            opMode.telemetry.update();
        }*/
        hwMap.liftOne.setPower(-TeleopConstants.liftPower);
        hwMap.liftTwo.setPower(-TeleopConstants.liftPower);

        try{
            Thread.sleep(1200);
        } catch (Exception e){}

        hwMap.liftOne.setPower(0);
        hwMap.liftTwo.setPower(0);
        hwMap.clawServo1.setPosition(TeleopConstants.clawServo1PosOpen);
        hwMap.clawServo2.setPosition(TeleopConstants.clawServo2PosOpen);
    }

    private void dropStone() {
        hwMap.clawServo2.setPosition(TeleopConstants.clawServo2PosOpen);
        hwMap.clawServo1.setPosition(TeleopConstants.clawServo1PosClose);

        /*Function0<Unit> callback = new Function0<Unit>() {
            @Override
            public Unit invoke() {
                hwMap.leftIntake.setPower(1);
                hwMap.rightIntake.setPower(-1);
                return Unit.INSTANCE;
            }
        };

        drive.getLocalizer().setPoseEstimate(new Pose2d(new Vector2d(73.0, 64.72), Math.toRadians(0)));
        drive.getLocalizer().update();
        builder = new TrajectoryBuilder(drive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);
        builder = builder.setReversed(false).lineTo(new Vector2d(65.0, 64.72)).strafeTo(new Vector2d(65.0, 41.44))
                .lineTo(new Vector2d(-9.552, 41.44)).strafeTo(new Vector2d(-9.552, 10.44))  ////
                .lineTo(new Vector2d(-18.552, 10.44)).addMarker(callback);
        trajectory = builder.build();
        drive.followTrajectorySync(trajectory);*/
    }

    private void intake(double power) {
        Thread thread = new Thread() {
            public void run() {
                hwMap.leftIntake.setPower(-power);
                hwMap.rightIntake.setPower(power);
            }
        };
        thread.start();
    }

    private void driveTime(double speed, int milisecs){
        hwMap.frontRight.setPower(speed);
        hwMap.frontLeft.setPower(speed);
        hwMap.backRight.setPower(speed);
        hwMap.backLeft.setPower(speed);

        try{
            Thread.sleep(milisecs);
        } catch (Exception e){}

        hwMap.frontRight.setPower(0);
        hwMap.frontLeft.setPower(0);
        hwMap.backRight.setPower(0);
        hwMap.backLeft.setPower(0);
    }
}
