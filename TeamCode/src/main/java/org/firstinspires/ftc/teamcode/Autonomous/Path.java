package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.BaseTrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.apache.commons.math3.geometry.euclidean.twod.Line;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.All.HardwareMap;
import org.firstinspires.ftc.teamcode.Autonomous.Vision.Align;
import org.firstinspires.ftc.teamcode.PID.DriveConstantsPID;
import org.firstinspires.ftc.teamcode.PID.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.TeleOp.Teleop;
import org.firstinspires.ftc.teamcode.TeleOp.TeleopConstants;

import java.util.List;

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

                builder = new TrajectoryBuilder(drive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);

                builder = builder.lineTo(new Vector2d(drive.getLocalizer().getPoseEstimate().getX(), drive.getLocalizer().getPoseEstimate().getY() + 5))
                        .splineTo(new Pose2d(new Vector2d(-18.832, -39.672), Math.toRadians(85)))
                //.lineTo(new Vector2d(-46.728, -11.52));
                /*.lineTo(new Vector2d(-39.728, -11.52))*/;
                trajectory = builder.build();
                drive.followTrajectorySync(trajectory);

                openPlatform();

                drive.getLocalizer().setPoseEstimate(new Pose2d(new Vector2d(-18.832, -39.672), Math.toRadians(85)));
                drive.getLocalizer().update();
                builder = new TrajectoryBuilder(drive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);
                builder = builder.lineTo(new Vector2d(-13.728, -22.52));
                trajectory = builder.build();
                drive.followTrajectorySync(trajectory);

                prepStone(TeleopConstants.stoneEncoderValues[0] - 300);

                drive.getLocalizer().setPoseEstimate(new Pose2d(new Vector2d(-11.728, -22.52), Math.toRadians(85)));
                drive.getLocalizer().update();
                builder = new TrajectoryBuilder(drive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);
                builder = builder.setReversed(true).lineTo(new Vector2d(-18.728, -49.672))
                        .setReversed(false).splineTo(new Pose2d(new Vector2d(-5.568, -33.44), Math.toRadians(0)))
                        .splineTo(new Pose2d(new Vector2d(65.488, -35.296), Math.toRadians(-90)));
                trajectory = builder.build();
                drive.followTrajectorySync(trajectory);

                align.setPower(0.25, 0.25);
                align.foundation(FieldPosition.RED_QUARY);
                intake(0);
                transferReset();

                drive.getLocalizer().setPoseEstimate(new Pose2d(new Vector2d(65.144, -16.128), drive.getExternalHeading()));
                drive.getLocalizer().update();
                builder = new TrajectoryBuilder(drive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);
                builder = builder.splineTo(new Pose2d(new Vector2d(45.064, -63.72), Math.toRadians(220)))
                        .setReversed(true).lineTo(new Vector2d(73.0, -64.72)).setReversed(false);
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

                drive.getLocalizer().setPoseEstimate(new Pose2d(new Vector2d(76.0, -64.72), Math.toRadians(0)));
                drive.getLocalizer().update();
                builder = new TrajectoryBuilder(drive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);
                builder = builder.setReversed(false).lineTo(new Vector2d(65.0, -64.72)).strafeTo(new Vector2d(65.0, -40.44))
                        .lineTo(new Vector2d(10.552, -40.44)).strafeTo(new Vector2d(10.552, -28.44))  ////
                        .lineTo(new Vector2d(0.552, -28.44));
                trajectory = builder.build();
                drive.followTrajectorySync(trajectory);
                //drive.turnSync(Math.toRadians(-45));

                /*drive.getLocalizer().setPoseEstimate(new Pose2d(new Vector2d(-22.552, -40.44), drive.getExternalHeading()));
                drive.getLocalizer().update();
                builder = new TrajectoryBuilder(drive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);
                //builder = builder.setReversed(false).lineTo(new Vector2d(-28.92, -20.312));
                builder = builder.back(16);
                trajectory = builder.build();
                drive.followTrajectorySync(trajectory);

                align.setPower(0, 0.2);
                align.skystoneRed(6);*/
                //init3 = init3 - Math.toDegrees(drive.getExternalHeading());

                /*drive.getLocalizer().setPoseEstimate(new Pose2d(new Vector2d(-13.552, -26.44), drive.getExternalHeading()));
                drive.getLocalizer().update();
                builder = new TrajectoryBuilder(drive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);
                //builder = builder.setReversed(false).lineTo(new Vector2d(-28.92, -20.312));
                builder = builder.lineTo(new Vector2d(-26.552, -26.44));
                trajectory = builder.build();
                drive.followTrajectorySync(trajectory);*/

                prepStone(TeleopConstants.stoneEncoderValues[1] - 300);

                drive.getLocalizer().setPoseEstimate(new Pose2d(new Vector2d(0.552, -23.44), drive.getExternalHeading()));
                drive.getLocalizer().update();
                builder = new TrajectoryBuilder(drive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);
                builder = builder.setReversed(true).strafeTo(new Vector2d(0.552, -50.44))
                        .lineTo(new Vector2d(77.0, -50.44));
                //.lineTo(new Vector2d(22.0, -68.44));
                trajectory = builder.build();
                drive.followTrajectorySync(trajectory);

                /*drive.getLocalizer().setPoseEstimate(new Pose2d(new Vector2d(-13.552, -68.44), Math.toRadians(180)));
                drive.getLocalizer().update();
                builder = new TrajectoryBuilder(drive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);
                builder = builder.setReversed(true).lineTo(new Vector2d(68.0, -68.44));
                trajectory = builder.build();
                drive.followTrajectorySync(trajectory);*/

                dropStone(TeleopConstants.stoneEncoderValues[1]);

                /*drive.getLocalizer().setPoseEstimate(new Pose2d(new Vector2d(68.0, -36.44), Math.toRadians(180)));
                drive.getLocalizer().update();
                builder = new TrajectoryBuilder(drive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);
                builder = builder.setReversed(false).lineTo(new Vector2d(22.0, -36.44));
                trajectory = builder.build();
                drive.followTrajectorySync(trajectory);*/

                intake(0);
                break;
            case 2:
                transferReset();
                initIntakeClaw();
                hwMap.foundationLock.setPosition(TeleopConstants.foundationLockUnlock);
                hwMap.transferLock.setPosition(TeleopConstants.transferLockPosPlatform);

                builder = new TrajectoryBuilder(drive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);

                builder = builder.lineTo(new Vector2d(drive.getLocalizer().getPoseEstimate().getX(), drive.getLocalizer().getPoseEstimate().getY() + 5))
                        .splineTo(new Pose2d(new Vector2d(-28.208, -39.672), Math.toRadians(85)))
                //.lineTo(new Vector2d(-46.728, -11.52));
                /*.lineTo(new Vector2d(-39.728, -11.52))*/;
                trajectory = builder.build();
                drive.followTrajectorySync(trajectory);

                openPlatform();

                drive.getLocalizer().setPoseEstimate(new Pose2d(new Vector2d(-28.728, -39.672), Math.toRadians(85)));
                drive.getLocalizer().update();
                builder = new TrajectoryBuilder(drive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);
                builder = builder.lineTo(new Vector2d(-23.728, -11.52));
                trajectory = builder.build();
                drive.followTrajectorySync(trajectory);

                prepStone(TeleopConstants.stoneEncoderValues[0] - 300);

                drive.getLocalizer().setPoseEstimate(new Pose2d(new Vector2d(-21.728, -11.52), Math.toRadians(85)));
                drive.getLocalizer().update();
                builder = new TrajectoryBuilder(drive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);
                builder = builder.setReversed(true).lineTo(new Vector2d(-28.728, -42.672))
                        .setReversed(false).splineTo(new Pose2d(new Vector2d(-5.568, -23.44), Math.toRadians(0)))
                        .splineTo(new Pose2d(new Vector2d(58.488, -25.296), Math.toRadians(-90)));
                trajectory = builder.build();
                drive.followTrajectorySync(trajectory);

                align.setPower(0.25, 0.25);
                align.foundation(FieldPosition.RED_QUARY);
                intake(0);
                transferReset();

                drive.getLocalizer().setPoseEstimate(new Pose2d(new Vector2d(58.144, -16.128), drive.getExternalHeading()));
                drive.getLocalizer().update();
                builder = new TrajectoryBuilder(drive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);
                builder = builder.splineTo(new Pose2d(new Vector2d(35.064, -58.72), Math.toRadians(220)))
                        .setReversed(true).lineTo(new Vector2d(68.0, -59.72)).setReversed(false);
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

                drive.getLocalizer().setPoseEstimate(new Pose2d(new Vector2d(68.0, -64.72), Math.toRadians(0)));
                drive.getLocalizer().update();
                builder = new TrajectoryBuilder(drive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);
                builder = builder.setReversed(false).lineTo(new Vector2d(55.0, -64.72)).strafeTo(new Vector2d(55.0, -55.44))
                        .lineTo(new Vector2d(-10.552, -55.44)).strafeTo(new Vector2d(-10.552, -23.44))  ////
                        .lineTo(new Vector2d(-20.552, -23.44));
                trajectory = builder.build();
                drive.followTrajectorySync(trajectory);
                //drive.turnSync(Math.toRadians(-45));

                /*drive.getLocalizer().setPoseEstimate(new Pose2d(new Vector2d(-22.552, -40.44), drive.getExternalHeading()));
                drive.getLocalizer().update();
                builder = new TrajectoryBuilder(drive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);
                //builder = builder.setReversed(false).lineTo(new Vector2d(-28.92, -20.312));
                builder = builder.back(16);
                trajectory = builder.build();
                drive.followTrajectorySync(trajectory);

                align.setPower(0, 0.2);
                align.skystoneRed(6);*/
                //init3 = init3 - Math.toDegrees(drive.getExternalHeading());

                /*drive.getLocalizer().setPoseEstimate(new Pose2d(new Vector2d(-13.552, -26.44), drive.getExternalHeading()));
                drive.getLocalizer().update();
                builder = new TrajectoryBuilder(drive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);
                //builder = builder.setReversed(false).lineTo(new Vector2d(-28.92, -20.312));
                builder = builder.lineTo(new Vector2d(-26.552, -26.44));
                trajectory = builder.build();
                drive.followTrajectorySync(trajectory);*/

                prepStone(TeleopConstants.stoneEncoderValues[1] - 300);

                drive.getLocalizer().setPoseEstimate(new Pose2d(new Vector2d(-20.552, -23.44), drive.getExternalHeading()));
                drive.getLocalizer().update();
                builder = new TrajectoryBuilder(drive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);
                builder = builder.setReversed(true).strafeTo(new Vector2d(-20.552, -55.44))
                        .lineTo(new Vector2d(77.0, -55.44));
                //.lineTo(new Vector2d(22.0, -68.44));
                trajectory = builder.build();
                drive.followTrajectorySync(trajectory);

                /*drive.getLocalizer().setPoseEstimate(new Pose2d(new Vector2d(-13.552, -68.44), Math.toRadians(180)));
                drive.getLocalizer().update();
                builder = new TrajectoryBuilder(drive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);
                builder = builder.setReversed(true).lineTo(new Vector2d(68.0, -68.44));
                trajectory = builder.build();
                drive.followTrajectorySync(trajectory);*/

                dropStone(TeleopConstants.stoneEncoderValues[1]);

                /*drive.getLocalizer().setPoseEstimate(new Pose2d(new Vector2d(68.0, -36.44), Math.toRadians(180)));
                drive.getLocalizer().update();
                builder = new TrajectoryBuilder(drive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);
                builder = builder.setReversed(false).lineTo(new Vector2d(22.0, -36.44));
                trajectory = builder.build();
                drive.followTrajectorySync(trajectory);*/

                intake(0);
                break;
            case 3:
                transferReset();
                initIntakeClaw();
                hwMap.foundationLock.setPosition(TeleopConstants.foundationLockUnlock);
                hwMap.transferLock.setPosition(TeleopConstants.transferLockPosPlatform);

                builder = new TrajectoryBuilder(drive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);

                builder = builder.lineTo(new Vector2d(drive.getLocalizer().getPoseEstimate().getX(), drive.getLocalizer().getPoseEstimate().getY() + 5))
                        .splineTo(new Pose2d(new Vector2d(-42.728, -39.672), Math.toRadians(85)))
                        //.lineTo(new Vector2d(-46.728, -11.52));
                        /*.lineTo(new Vector2d(-39.728, -11.52))*/;
                trajectory = builder.build();
                drive.followTrajectorySync(trajectory);

                //intake(1);
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
                        .setReversed(false).splineTo(new Pose2d(new Vector2d(-5.568, -23.44), Math.toRadians(0)))
                        .splineTo(new Pose2d(new Vector2d(47.488, -25.296), Math.toRadians(-90)));
                trajectory = builder.build();
                drive.followTrajectorySync(trajectory);

                align.setPower(0.25, 0.25);
                align.foundation(FieldPosition.RED_QUARY);
                intake(0);
                transferReset();

                drive.getLocalizer().setPoseEstimate(new Pose2d(new Vector2d(47.144, -16.128), drive.getExternalHeading()));
                drive.getLocalizer().update();
                builder = new TrajectoryBuilder(drive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);
                builder = builder.splineTo(new Pose2d(new Vector2d(29.064, -58.72), Math.toRadians(220)))
                        .setReversed(true).lineTo(new Vector2d(68.0, -59.72)).setReversed(false);
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

                drive.getLocalizer().setPoseEstimate(new Pose2d(new Vector2d(68.0, -64.72), Math.toRadians(0)));
                drive.getLocalizer().update();
                builder = new TrajectoryBuilder(drive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);
                builder = builder.setReversed(false).lineTo(new Vector2d(55.0, -64.72)).strafeTo(new Vector2d(55.0, -52.44))
                        .lineTo(new Vector2d(-17.552, -52.44)).strafeTo(new Vector2d(-17.552, -23.44))
                        .lineTo(new Vector2d(-27.552, -23.44));
                trajectory = builder.build();
                drive.followTrajectorySync(trajectory);
                //drive.turnSync(Math.toRadians(-45));

                /*drive.getLocalizer().setPoseEstimate(new Pose2d(new Vector2d(-22.552, -40.44), drive.getExternalHeading()));
                drive.getLocalizer().update();
                builder = new TrajectoryBuilder(drive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);
                //builder = builder.setReversed(false).lineTo(new Vector2d(-28.92, -20.312));
                builder = builder.back(16);
                trajectory = builder.build();
                drive.followTrajectorySync(trajectory);

                align.setPower(0, 0.2);
                align.skystoneRed(6);*/
                //init3 = init3 - Math.toDegrees(drive.getExternalHeading());

                /*drive.getLocalizer().setPoseEstimate(new Pose2d(new Vector2d(-13.552, -26.44), drive.getExternalHeading()));
                drive.getLocalizer().update();
                builder = new TrajectoryBuilder(drive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);
                //builder = builder.setReversed(false).lineTo(new Vector2d(-28.92, -20.312));
                builder = builder.lineTo(new Vector2d(-26.552, -26.44));
                trajectory = builder.build();
                drive.followTrajectorySync(trajectory);*/

                prepStone(TeleopConstants.stoneEncoderValues[1] - 300);

                drive.getLocalizer().setPoseEstimate(new Pose2d(new Vector2d(-27.552, -23.44), drive.getExternalHeading()));
                drive.getLocalizer().update();
                builder = new TrajectoryBuilder(drive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);
                builder = builder.setReversed(true).strafeTo(new Vector2d(-27.552, -55.44))
                        .lineTo(new Vector2d(77.0, -55.44));
                        //.lineTo(new Vector2d(22.0, -68.44));
                trajectory = builder.build();
                drive.followTrajectorySync(trajectory);

                /*drive.getLocalizer().setPoseEstimate(new Pose2d(new Vector2d(-13.552, -68.44), Math.toRadians(180)));
                drive.getLocalizer().update();
                builder = new TrajectoryBuilder(drive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);
                builder = builder.setReversed(true).lineTo(new Vector2d(68.0, -68.44));
                trajectory = builder.build();
                drive.followTrajectorySync(trajectory);*/

                dropStone(TeleopConstants.stoneEncoderValues[1]);

                /*drive.getLocalizer().setPoseEstimate(new Pose2d(new Vector2d(68.0, -36.44), Math.toRadians(180)));
                drive.getLocalizer().update();
                builder = new TrajectoryBuilder(drive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);
                builder = builder.setReversed(false).lineTo(new Vector2d(22.0, -36.44));
                trajectory = builder.build();
                drive.followTrajectorySync(trajectory);*/

                intake(0);
                break;
        }
    }

    public void RedFoundation() {
        try {
            Thread.sleep(25000);
        } catch (Exception e){}
        drive.getLocalizer().setPoseEstimate(startingPos);
        drive.getLocalizer().update();
        builder = new TrajectoryBuilder(drive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);
        builder = builder.strafeRight(8).setReversed(false).forward(32);
        trajectory = builder.build();
        drive.followTrajectorySync(trajectory);
    }

    public void BlueQuary(int[] skystonePositions) {    // (-x, y)
        switch (skystonePositions[0]) {
            case 1:
                break;
            case 2:
                transferReset();
                initIntakeClaw();
                hwMap.foundationLock.setPosition(TeleopConstants.foundationLockUnlock);
                hwMap.transferLock.setPosition(TeleopConstants.transferLockPosPlatform);

                builder = new TrajectoryBuilder(drive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);

                builder = builder.lineTo(new Vector2d(drive.getLocalizer().getPoseEstimate().getX(), drive.getLocalizer().getPoseEstimate().getY() + 5))
                        .splineTo(new Pose2d(new Vector2d(-28.208, 39.672), Math.toRadians(85)))
                //.lineTo(new Vector2d(-46.728, -11.52));
                /*.lineTo(new Vector2d(-39.728, -11.52))*/;
                trajectory = builder.build();
                drive.followTrajectorySync(trajectory);

                //intake(1);
                openPlatform();

                drive.getLocalizer().setPoseEstimate(new Pose2d(new Vector2d(-28.728, 39.672), Math.toRadians(85)));
                drive.getLocalizer().update();
                builder = new TrajectoryBuilder(drive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);
                builder = builder.lineTo(new Vector2d(-28.728, 11.52));
                trajectory = builder.build();
                drive.followTrajectorySync(trajectory);

                prepStone(TeleopConstants.stoneEncoderValues[0] - 300);

                drive.getLocalizer().setPoseEstimate(new Pose2d(new Vector2d(-28.728, 11.52), Math.toRadians(85)));
                drive.getLocalizer().update();
                builder = new TrajectoryBuilder(drive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);
                builder = builder.setReversed(true).lineTo(new Vector2d(-28.728, 42.672))
                        .setReversed(false).splineTo(new Pose2d(new Vector2d(-5.568, 23.44), Math.toRadians(0)))
                        .splineTo(new Pose2d(new Vector2d(54.488, 25.296), Math.toRadians(-90)));
                trajectory = builder.build();
                drive.followTrajectorySync(trajectory);

                align.setPower(0.25, 0.35);
                align.foundation(FieldPosition.RED_QUARY);
                intake(0);
                transferReset();

                drive.getLocalizer().setPoseEstimate(new Pose2d(new Vector2d(54.144, 16.128), drive.getExternalHeading()));
                drive.getLocalizer().update();
                builder = new TrajectoryBuilder(drive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);
                builder = builder.splineTo(new Pose2d(new Vector2d(29.064, 58.72), Math.toRadians(220)))
                        .setReversed(true).lineTo(new Vector2d(68.0, 59.72)).setReversed(false);
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

                drive.getLocalizer().setPoseEstimate(new Pose2d(new Vector2d(68.0, 64.72), Math.toRadians(0)));
                drive.getLocalizer().update();
                builder = new TrajectoryBuilder(drive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);
                builder = builder.setReversed(false).lineTo(new Vector2d(55.0, 64.72)).strafeTo(new Vector2d(55.0, 55.44))
                        .lineTo(new Vector2d(-7.552, 55.44)).strafeTo(new Vector2d(-7.552, 23.44))  ////
                        .lineTo(new Vector2d(-26.552, 23.44));
                trajectory = builder.build();
                drive.followTrajectorySync(trajectory);
                //drive.turnSync(Math.toRadians(-45));

                /*drive.getLocalizer().setPoseEstimate(new Pose2d(new Vector2d(-22.552, -40.44), drive.getExternalHeading()));
                drive.getLocalizer().update();
                builder = new TrajectoryBuilder(drive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);
                //builder = builder.setReversed(false).lineTo(new Vector2d(-28.92, -20.312));
                builder = builder.back(16);
                trajectory = builder.build();
                drive.followTrajectorySync(trajectory);

                align.setPower(0, 0.2);
                align.skystoneRed(6);*/
                //init3 = init3 - Math.toDegrees(drive.getExternalHeading());

                /*drive.getLocalizer().setPoseEstimate(new Pose2d(new Vector2d(-13.552, -26.44), drive.getExternalHeading()));
                drive.getLocalizer().update();
                builder = new TrajectoryBuilder(drive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);
                //builder = builder.setReversed(false).lineTo(new Vector2d(-28.92, -20.312));
                builder = builder.lineTo(new Vector2d(-26.552, -26.44));
                trajectory = builder.build();
                drive.followTrajectorySync(trajectory);*/

                prepStone(TeleopConstants.stoneEncoderValues[1] - 300);

                drive.getLocalizer().setPoseEstimate(new Pose2d(new Vector2d(-26.552, 23.44), drive.getExternalHeading()));
                drive.getLocalizer().update();
                builder = new TrajectoryBuilder(drive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);
                builder = builder.setReversed(true).strafeTo(new Vector2d(-26.552, 68.44))
                        .lineTo(new Vector2d(68.0, 68.44));
                //.lineTo(new Vector2d(22.0, -68.44));
                trajectory = builder.build();
                drive.followTrajectorySync(trajectory);

                /*drive.getLocalizer().setPoseEstimate(new Pose2d(new Vector2d(-13.552, -68.44), Math.toRadians(180)));
                drive.getLocalizer().update();
                builder = new TrajectoryBuilder(drive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);
                builder = builder.setReversed(true).lineTo(new Vector2d(68.0, -68.44));
                trajectory = builder.build();
                drive.followTrajectorySync(trajectory);*/

                dropStone(TeleopConstants.stoneEncoderValues[1]);

                /*drive.getLocalizer().setPoseEstimate(new Pose2d(new Vector2d(68.0, -36.44), Math.toRadians(180)));
                drive.getLocalizer().update();
                builder = new TrajectoryBuilder(drive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);
                builder = builder.setReversed(false).lineTo(new Vector2d(22.0, -36.44));
                trajectory = builder.build();
                drive.followTrajectorySync(trajectory);*/

                intake(0);
                break;
            case 3:
                transferReset();
                initIntakeClaw();
                hwMap.foundationLock.setPosition(TeleopConstants.foundationLockUnlock);
                hwMap.transferLock.setPosition(TeleopConstants.transferLockPosPlatform);

                builder = new TrajectoryBuilder(drive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);

                builder = builder.lineTo(new Vector2d(drive.getLocalizer().getPoseEstimate().getX(), drive.getLocalizer().getPoseEstimate().getY() + 5))
                        .splineTo(new Pose2d(new Vector2d(-42.728, 39.672), Math.toRadians(85)))
                //.lineTo(new Vector2d(-46.728, -11.52));
                /*.lineTo(new Vector2d(-39.728, -11.52))*/;
                trajectory = builder.build();
                drive.followTrajectorySync(trajectory);

                //intake(1);
                openPlatform();

                drive.getLocalizer().setPoseEstimate(new Pose2d(new Vector2d(-42.728, 39.672), Math.toRadians(85)));
                drive.getLocalizer().update();
                builder = new TrajectoryBuilder(drive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);
                builder = builder.lineTo(new Vector2d(-42.728, 11.52));
                trajectory = builder.build();
                drive.followTrajectorySync(trajectory);

                prepStone(TeleopConstants.stoneEncoderValues[0] - 300);

                drive.getLocalizer().setPoseEstimate(new Pose2d(new Vector2d(-42.728, 11.52), Math.toRadians(85)));
                drive.getLocalizer().update();
                builder = new TrajectoryBuilder(drive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);
                builder = builder.setReversed(true).lineTo(new Vector2d(-39.728, 42.672))
                        .setReversed(false).splineTo(new Pose2d(new Vector2d(-5.568, 23.44), Math.toRadians(0)))
                        .splineTo(new Pose2d(new Vector2d(50.488, 25.296), Math.toRadians(-90)));
                trajectory = builder.build();
                drive.followTrajectorySync(trajectory);

                align.setPower(0.25, 0.35);
                align.foundation(FieldPosition.RED_QUARY);
                intake(0);
                transferReset();

                drive.getLocalizer().setPoseEstimate(new Pose2d(new Vector2d(50.144, 16.128), drive.getExternalHeading()));
                drive.getLocalizer().update();
                builder = new TrajectoryBuilder(drive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);
                builder = builder.splineTo(new Pose2d(new Vector2d(29.064, 58.72), Math.toRadians(220)))
                        .setReversed(true).lineTo(new Vector2d(68.0, 59.72)).setReversed(false);
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

                drive.getLocalizer().setPoseEstimate(new Pose2d(new Vector2d(68.0, 64.72), Math.toRadians(0)));
                drive.getLocalizer().update();
                builder = new TrajectoryBuilder(drive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);
                builder = builder.setReversed(false).lineTo(new Vector2d(55.0, 64.72)).strafeTo(new Vector2d(55.0, 55.44))
                        .lineTo(new Vector2d(-16.552, 55.44)).strafeTo(new Vector2d(-16.552, 23.44))
                        .lineTo(new Vector2d(-26.552, 23.44));
                trajectory = builder.build();
                drive.followTrajectorySync(trajectory);
                //drive.turnSync(Math.toRadians(-45));

                /*drive.getLocalizer().setPoseEstimate(new Pose2d(new Vector2d(-22.552, -40.44), drive.getExternalHeading()));
                drive.getLocalizer().update();
                builder = new TrajectoryBuilder(drive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);
                //builder = builder.setReversed(false).lineTo(new Vector2d(-28.92, -20.312));
                builder = builder.back(16);
                trajectory = builder.build();
                drive.followTrajectorySync(trajectory);

                align.setPower(0, 0.2);
                align.skystoneRed(6);*/
                //init3 = init3 - Math.toDegrees(drive.getExternalHeading());

                /*drive.getLocalizer().setPoseEstimate(new Pose2d(new Vector2d(-13.552, -26.44), drive.getExternalHeading()));
                drive.getLocalizer().update();
                builder = new TrajectoryBuilder(drive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);
                //builder = builder.setReversed(false).lineTo(new Vector2d(-28.92, -20.312));
                builder = builder.lineTo(new Vector2d(-26.552, -26.44));
                trajectory = builder.build();
                drive.followTrajectorySync(trajectory);*/

                prepStone(TeleopConstants.stoneEncoderValues[1] - 300);

                drive.getLocalizer().setPoseEstimate(new Pose2d(new Vector2d(-26.552, 23.44), drive.getExternalHeading()));
                drive.getLocalizer().update();
                builder = new TrajectoryBuilder(drive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);
                builder = builder.setReversed(true).strafeTo(new Vector2d(-26.552, 68.44))
                        .lineTo(new Vector2d(68.0, 68.44));
                //.lineTo(new Vector2d(22.0, -68.44));
                trajectory = builder.build();
                drive.followTrajectorySync(trajectory);

                /*drive.getLocalizer().setPoseEstimate(new Pose2d(new Vector2d(-13.552, -68.44), Math.toRadians(180)));
                drive.getLocalizer().update();
                builder = new TrajectoryBuilder(drive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);
                builder = builder.setReversed(true).lineTo(new Vector2d(68.0, -68.44));
                trajectory = builder.build();
                drive.followTrajectorySync(trajectory);*/

                dropStone(TeleopConstants.stoneEncoderValues[1]);

                /*drive.getLocalizer().setPoseEstimate(new Pose2d(new Vector2d(68.0, -36.44), Math.toRadians(180)));
                drive.getLocalizer().update();
                builder = new TrajectoryBuilder(drive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);
                builder = builder.setReversed(false).lineTo(new Vector2d(22.0, -36.44));
                trajectory = builder.build();
                drive.followTrajectorySync(trajectory);*/

                intake(0);
                break;
        }
    }

    public void BlueFoundation() {
        try {
            Thread.sleep(25000);
        } catch (Exception e){}
        drive.getLocalizer().setPoseEstimate(startingPos);
        drive.getLocalizer().update();
        builder = new TrajectoryBuilder(drive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);
        builder = builder.strafeLeft(8).setReversed(false).forward(32);
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

    private void transferReset() {
        Thread thread = new Thread() {
            public void run() {
                hwMap.transferHorn.setPosition(TeleopConstants.transferHornPosReady);
                //hwMap.innerTransfer.setPosition(TeleopConstants.innerTransferPosBlock);
            }
        };
        thread.start();
    }

    private void openPlatform(){
        Thread thread = new Thread(){
            public void run(){
                hwMap.foundationLock.setPosition(TeleopConstants.foundationLockUnlock);
                hwMap.transferLock.setPosition(TeleopConstants.transferLockPosOut);
            }
        };
        thread.start();
    }

    public void resetLift(double power) {
        while (hwMap.liftReset.getState()) { //@TODO Find lift motor directions for up/down
            hwMap.liftOne.setPower(-power);
            hwMap.liftTwo.setPower(-power);
        }
    }

    private void initIntakeClaw() {
        Thread thread = new Thread() {
            public void run() {
                hwMap.clawInit.setPosition(TeleopConstants.clawInitPosReset);
                hwMap.clawServo1.setPosition(TeleopConstants.clawServo1PosClose);
                hwMap.clawServo2.setPosition(TeleopConstants.clawServo2PosClose);
                //resetLift(TeleopConstants.liftPower);
                hwMap.intakeInit.setPosition(TeleopConstants.intakeInitPosLeft);
                try {
                    Thread.sleep(1000);
                } catch (Exception e) {
                }
                hwMap.intakeInit.setPosition(TeleopConstants.intakeInitPosRight);
                intake(1);
                try {
                    Thread.sleep(3000);
                } catch (Exception e) {
                }
                hwMap.intakeInit.setPosition(TeleopConstants.intakeInitPosReset);
                //hwMap.clawInit.setPosition(TeleopConstants.clawInitPosReset);
                hwMap.clawInit.setPosition(TeleopConstants.clawInitPosCapstone);

                try {
                    Thread.sleep(500);
                } catch (Exception e) {
                }

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
                hwMap.clawServo1.setPosition(TeleopConstants.clawServo1PosClose);
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
                    sleep(1500);
                } catch (Exception e) {
                }
                hwMap.clawServo1.setPosition(TeleopConstants.clawServo1PosClose);
                hwMap.clawServo2.setPosition(TeleopConstants.clawServo2PosClose);

                /*while (Math.abs(hwMap.liftOne.getCurrentPosition()) < Math.abs(sliderEncoders) && opMode.opModeIsActive()) {
                    hwMap.liftOne.setPower(TeleopConstants.liftPower);
                    hwMap.liftTwo.setPower(TeleopConstants.liftPower);
                    opMode.telemetry.addData("Current Encoder Counts", -hwMap.liftOne.getCurrentPosition());
                    opMode.telemetry.addData("Target Encoder Counts", Math.abs(sliderEncoders));
                    opMode.telemetry.update();
                }*/
                hwMap.liftOne.setPower(0);
                hwMap.liftTwo.setPower(0);
            }
        };
        thread.start();
    }

    private void dropStone(double sliderEncoders) {
        /*while (Math.abs(hwMap.liftOne.getCurrentPosition()) > Math.abs(sliderEncoders) && hwMap.liftReset.getState() &&
                opMode.opModeIsActive()) {
            hwMap.liftOne.setPower(-TeleopConstants.liftPower);
            hwMap.liftTwo.setPower(-TeleopConstants.liftPower);
            opMode.telemetry.addData("Current Encoder Counts", -hwMap.liftOne.getCurrentPosition());
            opMode.telemetry.addData("Target Encoder Counts", Math.abs(sliderEncoders));
            opMode.telemetry.update();
        }*/
        hwMap.liftOne.setPower(0);
        hwMap.liftTwo.setPower(0);
        hwMap.clawServo1.setPosition(TeleopConstants.clawServo1PosOpen);
        hwMap.clawServo2.setPosition(TeleopConstants.clawServo2PosOpen);
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
}
