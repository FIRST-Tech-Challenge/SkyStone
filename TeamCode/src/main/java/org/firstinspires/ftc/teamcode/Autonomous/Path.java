package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.BaseTrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.All.HardwareMap;
import org.firstinspires.ftc.teamcode.Autonomous.Vision.Align;
import org.firstinspires.ftc.teamcode.PID.DriveConstantsPID;
import org.firstinspires.ftc.teamcode.PID.localizer.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.PID.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.PID.mecanum.SampleMecanumDriveREV;
import org.firstinspires.ftc.teamcode.TeleOp.TeleopConstants;

import java.lang.reflect.Field;
import java.util.List;

import kotlin.Unit;
import kotlin.jvm.functions.Function0;

import static java.lang.Math.PI;
import static org.firstinspires.ftc.teamcode.PID.DriveConstantsPID.rear_ratio;

public class Path {
    private Pose2d startingPos;
    private SampleMecanumDriveBase straightDrive;
    private SampleMecanumDriveBase strafeDrive;
    private BaseTrajectoryBuilder builder;
    private Trajectory trajectory;
    private Align align;
    private HardwareMap hwMap;
    private LinearOpMode opMode;
    private List<Recognition> tfod;
    private com.qualcomm.robotcore.hardware.HardwareMap hardwareMap;
    private static String TAG = "AutonomousPath";
    private Pose2d currentPos;
    private BNO055IMU imu;

    public Path(HardwareMap hwMap, LinearOpMode opMode, SampleMecanumDriveBase straightDrive,
                SampleMecanumDriveBase strafeDrive, Pose2d startingPos,
                com.qualcomm.robotcore.hardware.HardwareMap hardwareMap, BNO055IMU imu) {
        this.straightDrive = straightDrive;
        this.strafeDrive = strafeDrive;
        this.startingPos = startingPos;
        this.hwMap = hwMap;
        this.opMode = opMode;
        this.hardwareMap = hardwareMap;
        align = new Align(hwMap, opMode, DcMotor.ZeroPowerBehavior.BRAKE);
        this.straightDrive.getLocalizer().setPoseEstimate(startingPos);
        this.straightDrive.getLocalizer().update();
        this.strafeDrive.getLocalizer().setPoseEstimate(startingPos);
        this.strafeDrive.getLocalizer().update();
        this.imu = imu;
    }
    /*
    input: last pose from previous move;
    return: drive instance;
     */
    public SampleMecanumDriveBase DriveBuilderReset(Pose2d lastPose, boolean isStrafe, boolean init_imu)
    {
        SampleMecanumDriveBase _drive;
        _drive = new SampleMecanumDriveREV(hardwareMap, isStrafe, init_imu);
        _drive.getLocalizer().setPoseEstimate(lastPose);
        _drive.getLocalizer().update();
        if (!isStrafe) {
            builder = new TrajectoryBuilder(_drive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);
        }
        else {
            builder = new TrajectoryBuilder(_drive.getPoseEstimate(), DriveConstantsPID.STRAFE_BASE_CONSTRAINTS);
        }
        RobotLog.dd(TAG, "drive and builder created, initialized with pose: " + _drive.getPoseEstimate().toString());
        return _drive;
    }
    public void RedQuary(int[] skystonePositions) {
        DriveConstantsPID.updateConstantsFromProperties();
        switch (skystonePositions[0]) {
            case 1:
                double initStrafeDistance;
                double yCoordMvmtPlane = -46.8;
                double wallSkyStoneX = -49.0;
                double furtherMostSkyStoneX = -26.0;
                double firstRegularStoneX = -34.0;
                double foundationX = 46.0;
                double strafeDistance = 9.0;
                double rightStrafeCorrection;

                transferReset();
                initIntakeClaw();

                hwMap.foundationLock.setPosition(TeleopConstants.foundationLockUnlock);
                hwMap.transferLock.setPosition(TeleopConstants.transferLockPosPlatform);

                builder = new TrajectoryBuilder(strafeDrive.getPoseEstimate(), DriveConstantsPID.STRAFE_BASE_CONSTRAINTS);   //-34.752, -63.936
                if(DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {

                    builder = builder
                            .strafeTo(new Vector2d(strafeDrive.getPoseEstimate().getX(), yCoordMvmtPlane + strafeDistance));

                } else {
                    builder = builder.strafeTo(new Vector2d(strafeDrive.getPoseEstimate().getX(),-30)).setReversed(true)
                            .lineTo(new Vector2d(-50, -30));
                }
                trajectory = builder.build();   //x - 2.812, y + 7.984
                strafeDrive.followTrajectorySync(trajectory);

                straightDrive = new SampleMecanumDriveREV(hardwareMap, false, false);
                straightDrive.getLocalizer().setPoseEstimate(strafeDrive.getPoseEstimate());
                straightDrive.getLocalizer().update();
                builder = new TrajectoryBuilder(straightDrive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);   //-34.752, -63.936
                if(DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {

                    builder = builder
                            .setReversed(true).lineTo(new Vector2d(wallSkyStoneX, straightDrive.getPoseEstimate().getY()));

                } else {
                    builder = builder.strafeTo(new Vector2d(straightDrive.getPoseEstimate().getX(),-30)).setReversed(true)
                            .lineTo(new Vector2d(-50, -30));
                }
                trajectory = builder.build();   //x - 2.812, y + 7.984
                straightDrive.followTrajectorySync(trajectory);

                grabStone(FieldPosition.RED_QUARY);

                strafeDrive = new SampleMecanumDriveREV(hardwareMap, true, false);
                strafeDrive.getLocalizer().setPoseEstimate(straightDrive.getPoseEstimate());
                strafeDrive.getLocalizer().update();
                builder = new TrajectoryBuilder(strafeDrive.getPoseEstimate(), DriveConstantsPID.STRAFE_BASE_CONSTRAINTS);
                if(DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {
                    builder = builder.setReversed(false).strafeTo(new Vector2d(wallSkyStoneX, yCoordMvmtPlane));  //-52, -39
                } else {
                    builder = builder.setReversed(false).strafeTo(new Vector2d(-50, -40))
                            .lineTo(new Vector2d(50, -40)).strafeTo(new Vector2d(50, -30));
                }
                trajectory = builder.build();
                strafeDrive.followTrajectorySync(trajectory);

                straightDrive = new SampleMecanumDriveREV(hardwareMap, false, false);
                straightDrive.getLocalizer().setPoseEstimate(strafeDrive.getPoseEstimate());
                straightDrive.getLocalizer().update();
                builder = new TrajectoryBuilder(straightDrive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);
                if(DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {
                    builder = builder.setReversed(false)
                            .lineTo(new Vector2d(foundationX, yCoordMvmtPlane));
                    //-52, -39
                } else {
                    builder = builder.setReversed(false).strafeTo(new Vector2d(-50, -40))
                            .lineTo(new Vector2d(50, -40)).strafeTo(new Vector2d(50, -30));
                }
                trajectory = builder.build();
                straightDrive.followTrajectorySync(trajectory);

                strafeDrive = new SampleMecanumDriveREV(hardwareMap, true, false);
                strafeDrive.getLocalizer().setPoseEstimate(new Pose2d(new Vector2d(straightDrive.getPoseEstimate().getX(),
                        straightDrive.getPoseEstimate().getY()), straightDrive.getPoseEstimate().getHeading()));
                strafeDrive.getLocalizer().update();
                builder = new TrajectoryBuilder(strafeDrive.getPoseEstimate(), DriveConstantsPID.STRAFE_BASE_CONSTRAINTS);
                if(DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {
                    builder = builder.strafeTo(new Vector2d(foundationX, yCoordMvmtPlane + strafeDistance));
                } else {
                    builder = builder.strafeTo(new Vector2d(50, -40)).setReversed(true).lineTo(new Vector2d(-20, -40))
                            .strafeTo(new Vector2d(-20, -28));
                }
                trajectory = builder.build();
                strafeDrive.followTrajectorySync(trajectory);
                currentPos = strafeDrive.getPoseEstimate();

                dropStone(FieldPosition.RED_QUARY);

                strafeDrive = new SampleMecanumDriveREV(hardwareMap, true, false);
                strafeDrive.getLocalizer().setPoseEstimate(new Pose2d(currentPos.getX(), currentPos.getY(), currentPos.getHeading()));
                strafeDrive.getLocalizer().update();
                builder = new TrajectoryBuilder(strafeDrive.getPoseEstimate(), DriveConstantsPID.STRAFE_BASE_CONSTRAINTS);
                if(DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {
                    builder = builder.strafeTo(new Vector2d(foundationX, yCoordMvmtPlane));
                } else {
                    builder = builder.strafeTo(new Vector2d(50, -40)).setReversed(true).lineTo(new Vector2d(-20, -40))
                            .strafeTo(new Vector2d(-20, -28));
                }
                trajectory = builder.build();
                strafeDrive.followTrajectorySync(trajectory);

                straightDrive = new SampleMecanumDriveREV(hardwareMap, false, false);
                straightDrive.getLocalizer().setPoseEstimate(new Pose2d(new Vector2d(strafeDrive.getPoseEstimate().getX(),
                        strafeDrive.getPoseEstimate().getY()), strafeDrive.getPoseEstimate().getHeading()));
                straightDrive.getLocalizer().update();
                builder = new TrajectoryBuilder(straightDrive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);
                if(DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {
                    builder = builder
                            .setReversed(true).lineTo(new Vector2d(furtherMostSkyStoneX, yCoordMvmtPlane));
                } else {
                    builder = builder.strafeTo(new Vector2d(50, -40)).setReversed(true).lineTo(new Vector2d(-20, -40))
                            .strafeTo(new Vector2d(-20, -28));
                }
                trajectory = builder.build();
                straightDrive.followTrajectorySync(trajectory);

                strafeDrive = new SampleMecanumDriveREV(hardwareMap, true, false);
                strafeDrive.getLocalizer().setPoseEstimate(new Pose2d(new Vector2d(straightDrive.getPoseEstimate().getX(),
                        straightDrive.getPoseEstimate().getY()), strafeDrive.getPoseEstimate().getHeading()));
                strafeDrive.getLocalizer().update();
                builder = new TrajectoryBuilder(strafeDrive.getPoseEstimate(), DriveConstantsPID.STRAFE_BASE_CONSTRAINTS);
                if(DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {
                    builder = builder
                            .strafeTo(new Vector2d(furtherMostSkyStoneX, yCoordMvmtPlane + strafeDistance));
                } else {
                    builder = builder.strafeTo(new Vector2d(50, -40)).setReversed(true).lineTo(new Vector2d(-20, -40))
                            .strafeTo(new Vector2d(-20, -28));
                }
                trajectory = builder.build();
                strafeDrive.followTrajectorySync(trajectory);
                currentPos = strafeDrive.getPoseEstimate();

                grabStone(FieldPosition.RED_QUARY);

                strafeDrive = new SampleMecanumDriveREV(hardwareMap, true, false);
                strafeDrive.getLocalizer().setPoseEstimate(new Pose2d(currentPos.getX(), currentPos.getY(), currentPos.getHeading()));
                strafeDrive.getLocalizer().update();
                builder = new TrajectoryBuilder(strafeDrive.getPoseEstimate(), DriveConstantsPID.STRAFE_BASE_CONSTRAINTS);
                if(DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {
                    builder = builder.setReversed(false).strafeTo(new Vector2d(furtherMostSkyStoneX, yCoordMvmtPlane));
                } else {
                    builder = builder.setReversed(false).strafeTo(new Vector2d(-20, -40)).lineTo(new Vector2d(50, -40))
                            .strafeTo(new Vector2d(50, -30));
                }
                trajectory = builder.build();
                strafeDrive.followTrajectorySync(trajectory);

                straightDrive = new SampleMecanumDriveREV(hardwareMap, false, false);
                straightDrive.getLocalizer().setPoseEstimate(new Pose2d(new Vector2d(strafeDrive.getPoseEstimate().getX(),
                        strafeDrive.getPoseEstimate().getY()), strafeDrive.getPoseEstimate().getHeading()));
                straightDrive.getLocalizer().update();
                builder = new TrajectoryBuilder(straightDrive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);
                if(DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {
                    builder = builder.setReversed(false)
                            .lineTo(new Vector2d(foundationX, yCoordMvmtPlane));
                } else {
                    builder = builder.setReversed(false).strafeTo(new Vector2d(-20, -40)).lineTo(new Vector2d(50, -40))
                            .strafeTo(new Vector2d(50, -30));
                }
                trajectory = builder.build();
                straightDrive.followTrajectorySync(trajectory);

                strafeDrive = new SampleMecanumDriveREV(hardwareMap, true, false);
                strafeDrive.getLocalizer().setPoseEstimate(new Pose2d(new Vector2d(straightDrive.getPoseEstimate().getX(),
                        straightDrive.getPoseEstimate().getY()), straightDrive.getPoseEstimate().getHeading()));
                strafeDrive.getLocalizer().update();
                builder = new TrajectoryBuilder(strafeDrive.getPoseEstimate(), DriveConstantsPID.STRAFE_BASE_CONSTRAINTS);
                if(DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {
                    builder = builder.setReversed(false)
                            .strafeTo(new Vector2d(foundationX, yCoordMvmtPlane + strafeDistance));
                } else {
                    builder = builder.setReversed(false).strafeTo(new Vector2d(-20, -40)).lineTo(new Vector2d(50, -40))
                            .strafeTo(new Vector2d(50, -30));
                }
                trajectory = builder.build();
                strafeDrive.followTrajectorySync(trajectory);
                currentPos = strafeDrive.getPoseEstimate();

                dropStone(FieldPosition.RED_QUARY);

                strafeDrive = new SampleMecanumDriveREV(hardwareMap, true, false);
                strafeDrive.getLocalizer().setPoseEstimate(new Pose2d(currentPos.getX(), currentPos.getY(), currentPos.getHeading()));
                strafeDrive.getLocalizer().update();
                builder = new TrajectoryBuilder(strafeDrive.getPoseEstimate(), DriveConstantsPID.STRAFE_BASE_CONSTRAINTS);
                if(DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {
                    builder = builder.strafeTo(new Vector2d(foundationX, yCoordMvmtPlane));
                }
                trajectory = builder.build();
                strafeDrive.followTrajectorySync(trajectory);

                straightDrive = new SampleMecanumDriveREV(hardwareMap, false, false);
                straightDrive.getLocalizer().setPoseEstimate(new Pose2d(new Vector2d(strafeDrive.getPoseEstimate().getX(),
                        strafeDrive.getPoseEstimate().getY()), strafeDrive.getPoseEstimate().getHeading()));
                straightDrive.getLocalizer().update();
                builder = new TrajectoryBuilder(straightDrive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);
                if(DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {
                    builder = builder
                            .setReversed(true).lineTo(new Vector2d(firstRegularStoneX, yCoordMvmtPlane));
                }
                trajectory = builder.build();
                straightDrive.followTrajectorySync(trajectory);

                strafeDrive = new SampleMecanumDriveREV(hardwareMap, true, false);
                strafeDrive.getLocalizer().setPoseEstimate(new Pose2d(new Vector2d(straightDrive.getPoseEstimate().getX(),
                        straightDrive.getPoseEstimate().getY()), strafeDrive.getPoseEstimate().getHeading()));
                strafeDrive.getLocalizer().update();
                builder = new TrajectoryBuilder(strafeDrive.getPoseEstimate(), DriveConstantsPID.STRAFE_BASE_CONSTRAINTS);
                if(DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {
                    builder = builder
                            .strafeTo(new Vector2d(firstRegularStoneX, yCoordMvmtPlane + strafeDistance));
                }
                trajectory = builder.build();
                strafeDrive.followTrajectorySync(trajectory);
                currentPos = strafeDrive.getPoseEstimate();

                grabStone(FieldPosition.RED_QUARY);

                strafeDrive = new SampleMecanumDriveREV(hardwareMap, true, false);
                strafeDrive.getLocalizer().setPoseEstimate(new Pose2d(currentPos.getX(), currentPos.getY(), currentPos.getHeading()));
                strafeDrive.getLocalizer().update();
                builder = new TrajectoryBuilder(strafeDrive.getPoseEstimate(), DriveConstantsPID.STRAFE_BASE_CONSTRAINTS);
                if(DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {
                    builder = builder.setReversed(false).strafeTo(new Vector2d(firstRegularStoneX, yCoordMvmtPlane));
                }
                trajectory = builder.build();
                strafeDrive.followTrajectorySync(trajectory);

                straightDrive = new SampleMecanumDriveREV(hardwareMap, false, false);
                straightDrive.getLocalizer().setPoseEstimate(new Pose2d(new Vector2d(strafeDrive.getPoseEstimate().getX(),
                        strafeDrive.getPoseEstimate().getY()), strafeDrive.getPoseEstimate().getHeading()));
                straightDrive.getLocalizer().update();
                builder = new TrajectoryBuilder(straightDrive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);
                if(DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {
                    builder = builder.setReversed(false)
                            .lineTo(new Vector2d(foundationX, yCoordMvmtPlane));
                }
                trajectory = builder.build();
                straightDrive.followTrajectorySync(trajectory);

                strafeDrive = new SampleMecanumDriveREV(hardwareMap, true, false);
                strafeDrive.getLocalizer().setPoseEstimate(new Pose2d(new Vector2d(straightDrive.getPoseEstimate().getX(),
                        straightDrive.getPoseEstimate().getY()), strafeDrive.getPoseEstimate().getHeading()));
                strafeDrive.getLocalizer().update();
                builder = new TrajectoryBuilder(strafeDrive.getPoseEstimate(), DriveConstantsPID.STRAFE_BASE_CONSTRAINTS);
                if(DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {
                    builder = builder.setReversed(false)
                            .strafeTo(new Vector2d(foundationX, yCoordMvmtPlane + strafeDistance));
                }
                trajectory = builder.build();
                strafeDrive.followTrajectorySync(trajectory);
                currentPos = strafeDrive.getPoseEstimate();

                dropStone(FieldPosition.RED_QUARY);

                if(getIMUAngle() < Math.PI)
                    straightDrive.turnSync(-((getIMUAngle() + 2 * Math.PI) - 3 * Math.PI / 2));
                else
                    straightDrive.turnSync(-(getIMUAngle() - 3 * Math.PI / 2));

                strafeDrive = new SampleMecanumDriveREV(hardwareMap, true, false);
                strafeDrive.getLocalizer().setPoseEstimate(new Pose2d(currentPos.getX(), currentPos.getY(), currentPos.getHeading()));
                strafeDrive.getLocalizer().update();
                builder = new TrajectoryBuilder(strafeDrive.getPoseEstimate(), DriveConstantsPID.STRAFE_BASE_CONSTRAINTS);
                if(DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {
                    builder = builder.setReversed(false).strafeTo(new Vector2d(foundationX, yCoordMvmtPlane - 6));
                            //.strafeTo(new Vector2d(42, -30));
                }
                trajectory = builder.build();
                strafeDrive.followTrajectorySync(trajectory);

                straightDrive = new SampleMecanumDriveREV(hardwareMap, true, false);
                straightDrive.getLocalizer().setPoseEstimate(new Pose2d(new Vector2d(strafeDrive.getPoseEstimate().getX(),
                        strafeDrive.getPoseEstimate().getY()), strafeDrive.getPoseEstimate().getHeading()));
                straightDrive.getLocalizer().update();
                builder = new TrajectoryBuilder(straightDrive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);
                if(DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {
                    builder = builder.setReversed(false)
                            .setReversed(true).lineTo(new Vector2d(straightDrive.getPoseEstimate().getX(), -20));
                    //.strafeTo(new Vector2d(42, -30));
                }
                trajectory = builder.build();
                straightDrive.followTrajectorySync(trajectory);
                currentPos = straightDrive.getPoseEstimate();

                hwMap.foundationLock.setPosition(TeleopConstants.foundationLockLock);
                hwMap.transferLock.setPosition(TeleopConstants.transferLockPosUp);

                straightDrive = new SampleMecanumDriveREV(hardwareMap, true, false);
                straightDrive.getLocalizer().setPoseEstimate(new Pose2d(currentPos.getX(), currentPos.getY(), currentPos.getHeading()));
                straightDrive.getLocalizer().update();
                builder = new TrajectoryBuilder(straightDrive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);
                if(DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {
                    builder = builder.setReversed(false)//.strafeTo(new Vector2d(-24, -40))
                            .setReversed(false).lineTo(new Vector2d(straightDrive.getPoseEstimate().getX() - 15, -52));
                    //.strafeTo(new Vector2d(42, -30));
                }
                trajectory = builder.build();
                straightDrive.followTrajectorySync(trajectory);

                straightDrive.turnSync(-(getIMUAngle() - Math.PI) + Math.toRadians(20));
                break;
            case 2:
                initStrafeDistance = 27.0;
                yCoordMvmtPlane = -63.936 + initStrafeDistance;
                wallSkyStoneX = -61.0;
                furtherMostSkyStoneX = -33.0;
                firstRegularStoneX = -24.0;
                foundationX = 45.0;
                strafeDistance = 12.0;
                rightStrafeCorrection = -2.0;

                transferReset();
                initIntakeClaw();

                DriveConstantsPID.strafeDistance(hardwareMap, initStrafeDistance, true);
                updatePoseFromStrafe(initStrafeDistance, true);

                builder = new TrajectoryBuilder(straightDrive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);   //-34.752, -63.936
                if(DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {

                    builder = builder//.strafeTo(new Vector2d(straightDrive.getPoseEstimate().getX(), -63.936 + 5.0))
                            /*.strafeTo(new Vector2d(straightDrive.getPoseEstimate().getX(), -63.936 + 5.0 * 2))
                            .strafeTo(new Vector2d(straightDrive.getPoseEstimate().getX(), -63.936 + 5.0 * 3))
                            .strafeTo(new Vector2d(straightDrive.getPoseEstimate().getX(), -63.936 + 5.0 * 4))
                            .strafeTo(new Vector2d(straightDrive.getPoseEstimate().getX(), -63.936 + 5.0 * 5))*/
                            .setReversed(true).lineTo(new Vector2d(wallSkyStoneX, yCoordMvmtPlane));

                } else {
                    builder = builder.strafeTo(new Vector2d(straightDrive.getPoseEstimate().getX(),-30)).setReversed(true)
                            .lineTo(new Vector2d(-50, -30));
                }
                trajectory = builder.build();   //x - 2.812, y + 7.984
                straightDrive.followTrajectorySync(trajectory);
                RobotLog.dd("STATUS", "Strafe #1 Done");

                grabStone(FieldPosition.RED_QUARY);

                DriveConstantsPID.strafeDistance(hardwareMap, strafeDistance + rightStrafeCorrection, false);
                updatePoseFromStrafe(strafeDistance, false);

                straightDrive.getLocalizer().setPoseEstimate(new Pose2d(new Vector2d(straightDrive.getPoseEstimate().getX(),
                        straightDrive.getPoseEstimate().getY()), straightDrive.getPoseEstimate().getHeading()));
                straightDrive.getLocalizer().update();
                builder = new TrajectoryBuilder(straightDrive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);
                if(DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {
                    builder = builder.setReversed(false)//.strafeTo(new Vector2d(-42, -45))
                            .lineTo(new Vector2d(foundationX, yCoordMvmtPlane));//.strafeTo(new Vector2d(46,-35));  //-52, -39
                } else {
                    builder = builder.setReversed(false).strafeTo(new Vector2d(-50, -40))
                            .lineTo(new Vector2d(50, -40)).strafeTo(new Vector2d(50, -30));
                }
                trajectory = builder.build();
                straightDrive.followTrajectorySync(trajectory);

                RobotLog.dd("STATUS", "Strafe #2 Done");

                DriveConstantsPID.strafeDistance(hardwareMap, strafeDistance, true);
                updatePoseFromStrafe(strafeDistance, true);

                dropStone(FieldPosition.RED_QUARY);

                DriveConstantsPID.strafeDistance(hardwareMap, strafeDistance + rightStrafeCorrection, false);
                updatePoseFromStrafe(strafeDistance, false);

                straightDrive.getLocalizer().setPoseEstimate(new Pose2d(new Vector2d(straightDrive.getPoseEstimate().getX(),
                        straightDrive.getPoseEstimate().getY()), straightDrive.getPoseEstimate().getHeading()));
                straightDrive.getLocalizer().update();
                builder = new TrajectoryBuilder(straightDrive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);
                if(DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {
                    builder = builder//.strafeTo(new Vector2d(46, -40))
                            .setReversed(true).lineTo(new Vector2d(furtherMostSkyStoneX, yCoordMvmtPlane));
                    //.strafeTo(new Vector2d(-24, -30));
                } else {
                    builder = builder.strafeTo(new Vector2d(50, -40)).setReversed(true).lineTo(new Vector2d(-20, -40))
                            .strafeTo(new Vector2d(-20, -28));
                }
                trajectory = builder.build();
                straightDrive.followTrajectorySync(trajectory);

                RobotLog.dd("STATUS", "Strafe #3 Done");

                DriveConstantsPID.strafeDistance(hardwareMap, strafeDistance, true);
                updatePoseFromStrafe(strafeDistance, true);

                grabStone(FieldPosition.RED_QUARY);

                DriveConstantsPID.strafeDistance(hardwareMap, strafeDistance + rightStrafeCorrection, false);
                updatePoseFromStrafe(strafeDistance, false);

                straightDrive.getLocalizer().setPoseEstimate(new Pose2d(new Vector2d(straightDrive.getPoseEstimate().getX(),
                        straightDrive.getPoseEstimate().getY()), straightDrive.getPoseEstimate().getHeading()));
                straightDrive.getLocalizer().update();
                builder = new TrajectoryBuilder(straightDrive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);
                if(DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {
                    builder = builder.setReversed(false)//.strafeTo(new Vector2d(-24, -40))
                            .lineTo(new Vector2d(foundationX, yCoordMvmtPlane));
                    //.strafeTo(new Vector2d(42, -30));
                } else {
                    builder = builder.setReversed(false).strafeTo(new Vector2d(-20, -40)).lineTo(new Vector2d(50, -40))
                            .strafeTo(new Vector2d(50, -30));
                }
                trajectory = builder.build();
                straightDrive.followTrajectorySync(trajectory);

                RobotLog.dd("STATUS", "Strafe #4 Done");

                DriveConstantsPID.strafeDistance(hardwareMap, strafeDistance, true);
                updatePoseFromStrafe(strafeDistance, true);

                dropStone(FieldPosition.RED_QUARY);

                DriveConstantsPID.strafeDistance(hardwareMap, strafeDistance + rightStrafeCorrection, false);
                updatePoseFromStrafe(strafeDistance, false);

                straightDrive.getLocalizer().setPoseEstimate(new Pose2d(new Vector2d(straightDrive.getPoseEstimate().getX(),
                        straightDrive.getPoseEstimate().getY()), straightDrive.getPoseEstimate().getHeading()));
                straightDrive.getLocalizer().update();
                builder = new TrajectoryBuilder(straightDrive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);
                if(DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {
                    builder = builder//.strafeTo(new Vector2d(46, -40))
                            .setReversed(true).lineTo(new Vector2d(firstRegularStoneX, yCoordMvmtPlane));
                    //.strafeTo(new Vector2d(-24, -30));
                }
                trajectory = builder.build();
                straightDrive.followTrajectorySync(trajectory);

                RobotLog.dd("STATUS", "Strafe #3 Done");

                DriveConstantsPID.strafeDistance(hardwareMap, strafeDistance, true);
                updatePoseFromStrafe(strafeDistance, true);

                grabStone(FieldPosition.RED_QUARY);

                DriveConstantsPID.strafeDistance(hardwareMap, strafeDistance + rightStrafeCorrection, false);
                updatePoseFromStrafe(strafeDistance, false);

                straightDrive.getLocalizer().setPoseEstimate(new Pose2d(new Vector2d(straightDrive.getPoseEstimate().getX(),
                        straightDrive.getPoseEstimate().getY()), straightDrive.getPoseEstimate().getHeading()));
                straightDrive.getLocalizer().update();
                builder = new TrajectoryBuilder(straightDrive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);
                if(DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {
                    builder = builder.setReversed(false)//.strafeTo(new Vector2d(-24, -40))
                            .lineTo(new Vector2d(foundationX, yCoordMvmtPlane));
                    //.strafeTo(new Vector2d(42, -30));
                }
                trajectory = builder.build();
                straightDrive.followTrajectorySync(trajectory);

                RobotLog.dd("STATUS", "Strafe #4 Done");

                DriveConstantsPID.strafeDistance(hardwareMap, strafeDistance, true);
                updatePoseFromStrafe(strafeDistance, true);

                dropStone(FieldPosition.RED_QUARY);

                DriveConstantsPID.strafeDistance(hardwareMap, 16 + rightStrafeCorrection, false);
                updatePoseFromStrafe(16.0, false);

                straightDrive.turnSync(-Math.toRadians((straightDrive.getExternalHeading() + 2 * Math.PI) - 3 * Math.PI / 2));

                straightDrive.getLocalizer().setPoseEstimate(new Pose2d(new Vector2d(straightDrive.getPoseEstimate().getX(),
                        straightDrive.getPoseEstimate().getY()), straightDrive.getPoseEstimate().getHeading()));
                straightDrive.getLocalizer().update();
                builder = new TrajectoryBuilder(straightDrive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);
                if(DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {
                    builder = builder.setReversed(false)//.strafeTo(new Vector2d(-24, -40))
                            .setReversed(true).lineTo(new Vector2d(straightDrive.getPoseEstimate().getX(), -20));
                    //.strafeTo(new Vector2d(42, -30));
                }
                trajectory = builder.build();
                straightDrive.followTrajectorySync(trajectory);

                hwMap.foundationLock.setPosition(TeleopConstants.foundationLockLock);
                hwMap.transferLock.setPosition(TeleopConstants.transferLockPosUp);

                straightDrive.getLocalizer().setPoseEstimate(new Pose2d(new Vector2d(straightDrive.getPoseEstimate().getX(),
                        straightDrive.getPoseEstimate().getY()), straightDrive.getPoseEstimate().getHeading()));
                straightDrive.getLocalizer().update();
                builder = new TrajectoryBuilder(straightDrive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);
                if(DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {
                    builder = builder.setReversed(false)//.strafeTo(new Vector2d(-24, -40))
                            .setReversed(false).lineTo(new Vector2d(straightDrive.getPoseEstimate().getX() - 15, -53));
                    //.strafeTo(new Vector2d(42, -30));
                }
                trajectory = builder.build();
                straightDrive.followTrajectorySync(trajectory);

                straightDrive.turnSync(-Math.toRadians(straightDrive.getExternalHeading() - Math.PI));
                break;
            case 3:
                initStrafeDistance = 27.0;
                yCoordMvmtPlane = -63.936 + initStrafeDistance;
                wallSkyStoneX = -70.0;
                furtherMostSkyStoneX = -42.0;
                firstRegularStoneX = -24.0;
                foundationX = 45.0;
                strafeDistance = 12.0;
                rightStrafeCorrection = -2.0;

                transferReset();
                initIntakeClaw();

                try{
                    Thread.sleep(500);
                } catch (Exception e){}

                DriveConstantsPID.strafeDistance(hardwareMap, initStrafeDistance, true);
                updatePoseFromStrafe(initStrafeDistance, true);

                builder = new TrajectoryBuilder(straightDrive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);   //-34.752, -63.936
                if(DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {

                    builder = builder//.strafeTo(new Vector2d(straightDrive.getPoseEstimate().getX(), -63.936 + 5.0))
                            /*.strafeTo(new Vector2d(straightDrive.getPoseEstimate().getX(), -63.936 + 5.0 * 2))
                            .strafeTo(new Vector2d(straightDrive.getPoseEstimate().getX(), -63.936 + 5.0 * 3))
                            .strafeTo(new Vector2d(straightDrive.getPoseEstimate().getX(), -63.936 + 5.0 * 4))
                            .strafeTo(new Vector2d(straightDrive.getPoseEstimate().getX(), -63.936 + 5.0 * 5))*/
                            .setReversed(true).lineTo(new Vector2d(wallSkyStoneX, yCoordMvmtPlane));

                } else {
                    builder = builder.strafeTo(new Vector2d(straightDrive.getPoseEstimate().getX(),-30)).setReversed(true)
                            .lineTo(new Vector2d(-50, -30));
                }
                trajectory = builder.build();   //x - 2.812, y + 7.984
                straightDrive.followTrajectorySync(trajectory);
                RobotLog.dd("STATUS", "Strafe #1 Done");

                grabStone(FieldPosition.RED_QUARY);

                DriveConstantsPID.strafeDistance(hardwareMap, strafeDistance + rightStrafeCorrection, false);
                updatePoseFromStrafe(strafeDistance, false);

                straightDrive.getLocalizer().setPoseEstimate(new Pose2d(new Vector2d(straightDrive.getPoseEstimate().getX(),
                        straightDrive.getPoseEstimate().getY()), straightDrive.getPoseEstimate().getHeading()));
                straightDrive.getLocalizer().update();
                builder = new TrajectoryBuilder(straightDrive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);
                if(DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {
                    builder = builder.setReversed(false)//.strafeTo(new Vector2d(-42, -45))
                            .lineTo(new Vector2d(foundationX, yCoordMvmtPlane));//.strafeTo(new Vector2d(46,-35));  //-52, -39
                } else {
                    builder = builder.setReversed(false).strafeTo(new Vector2d(-50, -40))
                            .lineTo(new Vector2d(50, -40)).strafeTo(new Vector2d(50, -30));
                }
                trajectory = builder.build();
                straightDrive.followTrajectorySync(trajectory);

                RobotLog.dd("STATUS", "Strafe #2 Done");

                DriveConstantsPID.strafeDistance(hardwareMap, strafeDistance, true);
                updatePoseFromStrafe(strafeDistance, true);

                dropStone(FieldPosition.RED_QUARY);

                DriveConstantsPID.strafeDistance(hardwareMap, strafeDistance + rightStrafeCorrection, false);
                updatePoseFromStrafe(strafeDistance, false);

                straightDrive.getLocalizer().setPoseEstimate(new Pose2d(new Vector2d(straightDrive.getPoseEstimate().getX(),
                        straightDrive.getPoseEstimate().getY()), straightDrive.getPoseEstimate().getHeading()));
                straightDrive.getLocalizer().update();
                builder = new TrajectoryBuilder(straightDrive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);
                if(DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {
                    builder = builder//.strafeTo(new Vector2d(46, -40))
                            .setReversed(true).lineTo(new Vector2d(furtherMostSkyStoneX, yCoordMvmtPlane));
                    //.strafeTo(new Vector2d(-24, -30));
                } else {
                    builder = builder.strafeTo(new Vector2d(50, -40)).setReversed(true).lineTo(new Vector2d(-20, -40))
                            .strafeTo(new Vector2d(-20, -28));
                }
                trajectory = builder.build();
                straightDrive.followTrajectorySync(trajectory);

                RobotLog.dd("STATUS", "Strafe #3 Done");

                DriveConstantsPID.strafeDistance(hardwareMap, strafeDistance, true);
                updatePoseFromStrafe(strafeDistance, true);

                grabStone(FieldPosition.RED_QUARY);

                DriveConstantsPID.strafeDistance(hardwareMap, strafeDistance + rightStrafeCorrection, false);
                updatePoseFromStrafe(strafeDistance, false);

                straightDrive.getLocalizer().setPoseEstimate(new Pose2d(new Vector2d(straightDrive.getPoseEstimate().getX(),
                        straightDrive.getPoseEstimate().getY()), straightDrive.getPoseEstimate().getHeading()));
                straightDrive.getLocalizer().update();
                builder = new TrajectoryBuilder(straightDrive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);
                if(DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {
                    builder = builder.setReversed(false)//.strafeTo(new Vector2d(-24, -40))
                            .lineTo(new Vector2d(foundationX, yCoordMvmtPlane));
                    //.strafeTo(new Vector2d(42, -30));
                } else {
                    builder = builder.setReversed(false).strafeTo(new Vector2d(-20, -40)).lineTo(new Vector2d(50, -40))
                            .strafeTo(new Vector2d(50, -30));
                }
                trajectory = builder.build();
                straightDrive.followTrajectorySync(trajectory);

                RobotLog.dd("STATUS", "Strafe #4 Done");

                DriveConstantsPID.strafeDistance(hardwareMap, strafeDistance, true);
                updatePoseFromStrafe(strafeDistance, true);

                dropStone(FieldPosition.RED_QUARY);

                DriveConstantsPID.strafeDistance(hardwareMap, strafeDistance + rightStrafeCorrection, false);
                updatePoseFromStrafe(strafeDistance, false);

                straightDrive.getLocalizer().setPoseEstimate(new Pose2d(new Vector2d(straightDrive.getPoseEstimate().getX(),
                        straightDrive.getPoseEstimate().getY()), straightDrive.getPoseEstimate().getHeading()));
                straightDrive.getLocalizer().update();
                builder = new TrajectoryBuilder(straightDrive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);
                if(DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {
                    builder = builder//.strafeTo(new Vector2d(46, -40))
                            .setReversed(true).lineTo(new Vector2d(firstRegularStoneX, yCoordMvmtPlane));
                    //.strafeTo(new Vector2d(-24, -30));
                }
                trajectory = builder.build();
                straightDrive.followTrajectorySync(trajectory);

                RobotLog.dd("STATUS", "Strafe #3 Done");

                DriveConstantsPID.strafeDistance(hardwareMap, strafeDistance, true);
                updatePoseFromStrafe(strafeDistance, true);

                grabStone(FieldPosition.RED_QUARY);

                DriveConstantsPID.strafeDistance(hardwareMap, strafeDistance + rightStrafeCorrection, false);
                updatePoseFromStrafe(strafeDistance, false);

                straightDrive.getLocalizer().setPoseEstimate(new Pose2d(new Vector2d(straightDrive.getPoseEstimate().getX(),
                        straightDrive.getPoseEstimate().getY()), straightDrive.getPoseEstimate().getHeading()));
                straightDrive.getLocalizer().update();
                builder = new TrajectoryBuilder(straightDrive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);
                if(DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {
                    builder = builder.setReversed(false)//.strafeTo(new Vector2d(-24, -40))
                            .lineTo(new Vector2d(foundationX, yCoordMvmtPlane));
                    //.strafeTo(new Vector2d(42, -30));
                }
                trajectory = builder.build();
                straightDrive.followTrajectorySync(trajectory);

                RobotLog.dd("STATUS", "Strafe #4 Done");

                DriveConstantsPID.strafeDistance(hardwareMap, strafeDistance, true);
                updatePoseFromStrafe(strafeDistance, true);

                dropStone(FieldPosition.RED_QUARY);

                DriveConstantsPID.strafeDistance(hardwareMap, 16 + rightStrafeCorrection, false);
                updatePoseFromStrafe(16.0, false);

                straightDrive.turnSync(-Math.toRadians((straightDrive.getExternalHeading() + 2 * Math.PI) - 3 * Math.PI / 2));

                straightDrive.getLocalizer().setPoseEstimate(new Pose2d(new Vector2d(straightDrive.getPoseEstimate().getX(),
                        straightDrive.getPoseEstimate().getY()), straightDrive.getPoseEstimate().getHeading()));
                straightDrive.getLocalizer().update();
                builder = new TrajectoryBuilder(straightDrive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);
                if(DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {
                    builder = builder.setReversed(false)//.strafeTo(new Vector2d(-24, -40))
                            .setReversed(true).lineTo(new Vector2d(straightDrive.getPoseEstimate().getX(), -26));
                    //.strafeTo(new Vector2d(42, -30));
                }
                trajectory = builder.build();
                straightDrive.followTrajectorySync(trajectory);

                hwMap.foundationLock.setPosition(TeleopConstants.foundationLockLock);
                hwMap.transferLock.setPosition(TeleopConstants.transferLockPosUp);

                straightDrive.getLocalizer().setPoseEstimate(new Pose2d(new Vector2d(straightDrive.getPoseEstimate().getX(),
                        straightDrive.getPoseEstimate().getY()), straightDrive.getPoseEstimate().getHeading()));
                straightDrive.getLocalizer().update();
                builder = new TrajectoryBuilder(straightDrive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);
                if(DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {
                    builder = builder.setReversed(false)//.strafeTo(new Vector2d(-24, -40))
                            .setReversed(false).lineTo(new Vector2d(straightDrive.getPoseEstimate().getX() - 10, -56));
                    //.strafeTo(new Vector2d(42, -30));
                }
                trajectory = builder.build();
                straightDrive.followTrajectorySync(trajectory);

                straightDrive.turnSync(-Math.toRadians(straightDrive.getExternalHeading() - Math.PI));
                break;
        }
    }

    public void RedFoundationPark() {
        transferReset();
        initIntakeClaw();
        try {
            Thread.sleep(5000);
        } catch (Exception e){}
        straightDrive.getLocalizer().setPoseEstimate(startingPos);
        straightDrive.getLocalizer().update();
        builder = new TrajectoryBuilder(straightDrive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);
        builder = builder.strafeRight(6).setReversed(false).forward(28);
        trajectory = builder.build();
        straightDrive.followTrajectorySync(trajectory);

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

                builder = new TrajectoryBuilder(straightDrive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);

                builder = builder.lineTo(new Vector2d(straightDrive.getLocalizer().getPoseEstimate().getX(), straightDrive.getLocalizer().getPoseEstimate().getY() - 5))
                        .splineTo(new Pose2d(new Vector2d(-19.832, 39.672), Math.toRadians(-100)));
                trajectory = builder.build();
                straightDrive.followTrajectorySync(trajectory);

                openPlatform();

                straightDrive.getLocalizer().setPoseEstimate(new Pose2d(new Vector2d(-19.832, 39.672), Math.toRadians(-100)));
                straightDrive.getLocalizer().update();
                builder = new TrajectoryBuilder(straightDrive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);
                builder = builder.lineTo(new Vector2d(-17.728, 22.52));
                trajectory = builder.build();
                straightDrive.followTrajectorySync(trajectory);

                prepStone(TeleopConstants.stoneEncoderValues[0] - 300);

                straightDrive.getLocalizer().setPoseEstimate(new Pose2d(new Vector2d(-15.728, 22.52), Math.toRadians(-100)));
                straightDrive.getLocalizer().update();
                builder = new TrajectoryBuilder(straightDrive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);
                builder = builder.setReversed(true).lineTo(new Vector2d(-15.728, 36.672))
                        .setReversed(false).splineTo(new Pose2d(new Vector2d(-5.568, 34.44), Math.toRadians(0)))
                        .splineTo(new Pose2d(new Vector2d(68.488, 48.296), Math.toRadians(90)));
                trajectory = builder.build();
                straightDrive.followTrajectorySync(trajectory);

                align.setPower(0.13, 0.25);
                align.foundation(FieldPosition.BLUE_QUARY);
                intake(0);
                transferReset();

                straightDrive.getLocalizer().setPoseEstimate(new Pose2d(new Vector2d(68.144, 16.128), straightDrive.getExternalHeading()));
                straightDrive.getLocalizer().update();
                builder = new TrajectoryBuilder(straightDrive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);
                builder = builder.splineTo(new Pose2d(new Vector2d(37.064, 58.72), Math.toRadians(140)))
                        .setReversed(true).lineTo(new Vector2d(70.0, 58.72)).setReversed(false);
                trajectory = builder.build();
                straightDrive.followTrajectorySync(trajectory);

                dropStone(TeleopConstants.stoneEncoderValues[0]);
                hwMap.foundationLock.setPosition(TeleopConstants.foundationLockUnlock);
                hwMap.transferLock.setPosition(TeleopConstants.transferLockPosPlatform);

                try {
                    Thread.sleep(300);
                } catch (Exception e) {
                }
                transferReset();
                intake(1);

                straightDrive.getLocalizer().setPoseEstimate(new Pose2d(new Vector2d(70.0, 58.72), Math.toRadians(0)));
                straightDrive.getLocalizer().update();
                builder = new TrajectoryBuilder(straightDrive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);
                builder = builder.setReversed(false).lineTo(new Vector2d(66.0, 58.72)).strafeTo(new Vector2d(66.0, 32.44))
                        .lineTo(new Vector2d(20, 32.44));
                trajectory = builder.build();
                straightDrive.followTrajectorySync(trajectory);
                /*.strafeTo(new Vector2d(-1.552, 6.44))  ////-1.552
                        .lineTo(new Vector2d(-8.552, 6.44));
                trajectory = builder.build();
                straightDrive.followTrajectorySync(trajectory);

                prepStone();

                straightDrive.getLocalizer().setPoseEstimate(new Pose2d(new Vector2d(-8.552, 6.44), straightDrive.getExternalHeading()));
                straightDrive.getLocalizer().update();
                builder = new TrajectoryBuilder(straightDrive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);
                builder = builder.setReversed(true).strafeTo(new Vector2d(-8.552, 43.44));
                        //.lineTo(new Vector2d(77.0, 43.44));
                trajectory = builder.build();
                straightDrive.followTrajectorySync(trajectory);

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

                builder = new TrajectoryBuilder(straightDrive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);

                builder = builder.lineTo(new Vector2d(straightDrive.getLocalizer().getPoseEstimate().getX(), straightDrive.getLocalizer().getPoseEstimate().getY() - 5))
                        .splineTo(new Pose2d(new Vector2d(-32.832, 39.672), Math.toRadians(-100)));
                trajectory = builder.build();
                straightDrive.followTrajectorySync(trajectory);

                openPlatform();

                straightDrive.getLocalizer().setPoseEstimate(new Pose2d(new Vector2d(-32.832, 39.672), Math.toRadians(-100)));
                straightDrive.getLocalizer().update();
                builder = new TrajectoryBuilder(straightDrive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);
                builder = builder.lineTo(new Vector2d(-28.728, 18.52));
                trajectory = builder.build();
                straightDrive.followTrajectorySync(trajectory);

                prepStone(TeleopConstants.stoneEncoderValues[0] - 300);

                straightDrive.getLocalizer().setPoseEstimate(new Pose2d(new Vector2d(-28.728, 22.52), Math.toRadians(-100)));
                straightDrive.getLocalizer().update();
                builder = new TrajectoryBuilder(straightDrive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);
                builder = builder.setReversed(true).lineTo(new Vector2d(-28.728, 36.672))
                        .setReversed(false).splineTo(new Pose2d(new Vector2d(-5.568, 34.44), Math.toRadians(0)))
                        .splineTo(new Pose2d(new Vector2d(67.488, 46.296), Math.toRadians(90)));
                trajectory = builder.build();
                straightDrive.followTrajectorySync(trajectory);

                align.setPower(0.13, 0.25);
                align.foundation(FieldPosition.BLUE_QUARY);
                intake(0);
                transferReset();

                straightDrive.getLocalizer().setPoseEstimate(new Pose2d(new Vector2d(67.144, 16.128), straightDrive.getExternalHeading()));
                straightDrive.getLocalizer().update();
                builder = new TrajectoryBuilder(straightDrive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);
                builder = builder.splineTo(new Pose2d(new Vector2d(37.064, 52.72), Math.toRadians(140)))
                        .setReversed(true).lineTo(new Vector2d(68.0, 52.72)).setReversed(false);
                trajectory = builder.build();
                straightDrive.followTrajectorySync(trajectory);

                dropStone(TeleopConstants.stoneEncoderValues[0]);
                hwMap.foundationLock.setPosition(TeleopConstants.foundationLockUnlock);
                hwMap.transferLock.setPosition(TeleopConstants.transferLockPosPlatform);

                try {
                    Thread.sleep(300);
                } catch (Exception e) {
                }
                transferReset();
                intake(1);

                straightDrive.getLocalizer().setPoseEstimate(new Pose2d(new Vector2d(73.0, 50.72), Math.toRadians(0)));
                straightDrive.getLocalizer().update();
                builder = new TrajectoryBuilder(straightDrive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);
                builder = builder.setReversed(false).lineTo(new Vector2d(65.0, 50.72)).strafeTo(new Vector2d(65.0, 37.44))
                        .lineTo(new Vector2d(20.552, 37.44));
                trajectory = builder.build();
                straightDrive.followTrajectorySync(trajectory);
                /*.strafeTo(new Vector2d(-16.552, 10.44))  ////-16.552
                        .lineTo(new Vector2d(-24.552, 10.44));
                trajectory = builder.build();
                straightDrive.followTrajectorySync(trajectory);

                prepStone();

                straightDrive.getLocalizer().setPoseEstimate(new Pose2d(new Vector2d(-24.552, 10.44), straightDrive.getExternalHeading()));
                straightDrive.getLocalizer().update();
                builder = new TrajectoryBuilder(straightDrive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);
                builder = builder.setReversed(true).strafeTo(new Vector2d(-24.552, 53.44));
                        //.lineTo(new Vector2d(77.0, 43.44));
                trajectory = builder.build();
                straightDrive.followTrajectorySync(trajectory);

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

                builder = new TrajectoryBuilder(straightDrive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);

                builder = builder.lineTo(new Vector2d(straightDrive.getLocalizer().getPoseEstimate().getX(), straightDrive.getLocalizer().getPoseEstimate().getY() - 5))
                        .splineTo(new Pose2d(new Vector2d(-48.832, 39.672), Math.toRadians(-100)));
                trajectory = builder.build();
                straightDrive.followTrajectorySync(trajectory);

                openPlatform();

                straightDrive.getLocalizer().setPoseEstimate(new Pose2d(new Vector2d(-48.832, 39.672), Math.toRadians(-100)));
                straightDrive.getLocalizer().update();
                builder = new TrajectoryBuilder(straightDrive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);
                builder = builder.lineTo(new Vector2d(-46.728, 18.52));
                trajectory = builder.build();
                straightDrive.followTrajectorySync(trajectory);

                prepStone(TeleopConstants.stoneEncoderValues[0] - 300);

                straightDrive.getLocalizer().setPoseEstimate(new Pose2d(new Vector2d(-48.728, 18.52), Math.toRadians(-100)));
                straightDrive.getLocalizer().update();
                builder = new TrajectoryBuilder(straightDrive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);
                builder = builder.setReversed(true).lineTo(new Vector2d(-49.728, 27.672))
                        .setReversed(false).splineTo(new Pose2d(new Vector2d(-5.568, 25.44), Math.toRadians(0)))
                        .splineTo(new Pose2d(new Vector2d(51.488, 42.296), Math.toRadians(90)));
                trajectory = builder.build();
                straightDrive.followTrajectorySync(trajectory);

                align.setPower(0.13, 0.25);
                align.foundation(FieldPosition.BLUE_QUARY);
                intake(0);
                transferReset();

                straightDrive.getLocalizer().setPoseEstimate(new Pose2d(new Vector2d(51.144, 16.128), straightDrive.getExternalHeading()));
                straightDrive.getLocalizer().update();
                builder = new TrajectoryBuilder(straightDrive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);
                builder = builder.splineTo(new Pose2d(new Vector2d(38.064, 54.72), Math.toRadians(140)))
                        .setReversed(true).lineTo(new Vector2d(60.0, 54.72)).setReversed(false);
                trajectory = builder.build();
                straightDrive.followTrajectorySync(trajectory);

                dropStone(TeleopConstants.stoneEncoderValues[0]);
                hwMap.foundationLock.setPosition(TeleopConstants.foundationLockUnlock);
                hwMap.transferLock.setPosition(TeleopConstants.transferLockPosOut);

                try {
                    Thread.sleep(300);
                } catch (Exception e) {
                }
                transferReset();
                intake(1);

                straightDrive.getLocalizer().setPoseEstimate(new Pose2d(new Vector2d(60.0, 54.72), Math.toRadians(0)));
                straightDrive.getLocalizer().update();
                builder = new TrajectoryBuilder(straightDrive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);
                builder = builder.setReversed(false).lineTo(new Vector2d(54.0, 54.72)).strafeTo(new Vector2d(54.0, 35.44))
                        .lineTo(new Vector2d(17.552, 35.44));
                trajectory = builder.build();
                straightDrive.followTrajectorySync(trajectory);/*.strafeTo(new Vector2d(-25.552, 10.44))  ////
                        .lineTo(new Vector2d(-33.552, 10.44));//.strafeTo(new Vector2d(-33.552, 43.44));
                trajectory = builder.build();
                straightDrive.followTrajectorySync(trajectory);

                prepStone();

                straightDrive.getLocalizer().setPoseEstimate(new Pose2d(new Vector2d(-33.552, 10.44), straightDrive.getExternalHeading()));
                straightDrive.getLocalizer().update();
                builder = new TrajectoryBuilder(straightDrive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);
                builder = builder.setReversed(true).strafeTo(new Vector2d(-33.522, 43.44));
                        //.lineTo(new Vector2d(77.0, 43.44));
                trajectory = builder.build();
                straightDrive.followTrajectorySync(trajectory);


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
        straightDrive.getLocalizer().setPoseEstimate(startingPos);
        straightDrive.getLocalizer().update();
        builder = new TrajectoryBuilder(straightDrive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);
        builder = builder.strafeLeft(6).setReversed(false).forward(28);
        trajectory = builder.build();
        straightDrive.followTrajectorySync(trajectory);

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
        straightDrive.getLocalizer().setPoseEstimate(startingPos);
        straightDrive.getLocalizer().update();
        builder = new TrajectoryBuilder(straightDrive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);
        builder = builder.lineTo(new Vector2d(20.736, 63.936)).lineTo(new Vector2d(20.736, 48.936))
            .strafeTo(new Vector2d(68.144, 48.936));
        trajectory = builder.build();
        straightDrive.followTrajectorySync(trajectory);

        intake(0);
        align.setPower(0.13, 0.25);
        align.foundation(FieldPosition.BLUE_QUARY);
        transferReset();

        straightDrive.getLocalizer().setPoseEstimate(new Pose2d(new Vector2d(68.144, 16.128), straightDrive.getExternalHeading()));
        straightDrive.getLocalizer().update();
        builder = new TrajectoryBuilder(straightDrive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);
        builder = builder.splineTo(new Pose2d(new Vector2d(37.064, 58.72), Math.toRadians(140)))
                .setReversed(true).lineTo(new Vector2d(70.0, 58.72)).setReversed(false);
        trajectory = builder.build();
        straightDrive.followTrajectorySync(trajectory);

        straightDrive.getLocalizer().setPoseEstimate(new Pose2d(new Vector2d(60.0, 54.72), Math.toRadians(0)));
        straightDrive.getLocalizer().update();
        builder = new TrajectoryBuilder(straightDrive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);
        builder = builder.setReversed(false).lineTo(new Vector2d(54.0, 54.72)).strafeTo(new Vector2d(54.0, 60.44))
                .lineTo(new Vector2d(17.552, 60.44));
        trajectory = builder.build();
        straightDrive.followTrajectorySync(trajectory);
    }

    public void updateTFODData(List<Recognition> tfod) {
        this.tfod = tfod;
        updateTFOD();
    }

    public Pose2d getPoseEstimate() {
        return straightDrive.getLocalizer().getPoseEstimate();
    }

    public double getExternalHeading() {
        return straightDrive.getExternalHeading();
    }

    private void updateTFOD() {
        align.updateTFOD(tfod);
    }

    public void updateHeading() {
        align.updateExternalHeading(Math.toDegrees(straightDrive.getExternalHeading()));
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
            } catch (Exception e){}
            hwMap.redAutoClawJoint1.setPosition(TeleopConstants.autoClaw1Down);
            try{
                Thread.sleep(500);
            } catch (Exception e){}
            hwMap.redAutoClawJoint2.setPosition(TeleopConstants.autoClaw2Close);
            try{
                Thread.sleep(300);
            } catch (Exception e){}
            hwMap.redAutoClawJoint1.setPosition(TeleopConstants.autoClaw1Up);
        } else {
            hwMap.blueAutoClawJoint2.setPosition(TeleopConstants.autoClaw2Open);
            try{
                Thread.sleep(500);
            } catch (Exception e){}
            hwMap.blueAutoClawJoint1.setPosition(TeleopConstants.autoClaw1Down);
            try{
                Thread.sleep(300);
            } catch (Exception e){}
            hwMap.blueAutoClawJoint2.setPosition(TeleopConstants.autoClaw2Close);
            try{
                Thread.sleep(300);
            } catch (Exception e){}
            hwMap.redAutoClawJoint1.setPosition(TeleopConstants.autoClaw1Up);
        }
    }

    private void dropStone(FieldPosition fieldPosition){
        if(fieldPosition == FieldPosition.RED_QUARY || fieldPosition == FieldPosition.RED_FOUNDATION_PARK ||
                fieldPosition == FieldPosition.RED_FOUNDATION_DRAG) {
            hwMap.redAutoClawJoint1.setPosition(TeleopConstants.autoClaw1Down);
            try{
                Thread.sleep(300);
            } catch (Exception e){}
            hwMap.redAutoClawJoint2.setPosition(TeleopConstants.autoClaw2Open);
            try{
                Thread.sleep(300);
            } catch (Exception e){}
            hwMap.redAutoClawJoint1.setPosition(TeleopConstants.autoClaw1Up);
        } else {
            hwMap.blueAutoClawJoint1.setPosition(TeleopConstants.autoClaw1Down);
            try{
                Thread.sleep(300);
            } catch (Exception e){}
            hwMap.blueAutoClawJoint2.setPosition(TeleopConstants.autoClaw2Open);
            try{
                Thread.sleep(300);
            } catch (Exception e){}
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

        straightDrive.getLocalizer().setPoseEstimate(new Pose2d(new Vector2d(73.0, 64.72), Math.toRadians(0)));
        straightDrive.getLocalizer().update();
        builder = new TrajectoryBuilder(straightDrive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);
        builder = builder.setReversed(false).lineTo(new Vector2d(65.0, 64.72)).strafeTo(new Vector2d(65.0, 41.44))
                .lineTo(new Vector2d(-9.552, 41.44)).strafeTo(new Vector2d(-9.552, 10.44))  ////
                .lineTo(new Vector2d(-18.552, 10.44)).addMarker(callback);
        trajectory = builder.build();
        straightDrive.followTrajectorySync(trajectory);*/
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

    private void updatePoseFromStrafe(double yOdoInitPos, boolean left){
        double strafeDistance = Math.abs(hwMap.rightIntake.getCurrentPosition() - yOdoInitPos) /
                DriveConstantsPID.odoEncoderTicksPerRev * StandardTrackingWheelLocalizer.GEAR_RATIO *
                2 * Math.PI * StandardTrackingWheelLocalizer.WHEEL_RADIUS;

        RobotLog.dd("Current Position (X, Y, Heading)", straightDrive.getPoseEstimate().toString());
        RobotLog.dd("Strafe Distance", String.valueOf(strafeDistance));
        RobotLog.dd("Direction", left ? "Left" : "Right");
        if(!left)
            strafeDistance = -strafeDistance;

        double heading = straightDrive.getExternalHeading() < 0 ? Math.PI * 2 - straightDrive.getExternalHeading() : straightDrive.getExternalHeading();

        straightDrive.getLocalizer().setPoseEstimate(new Pose2d(new Vector2d(straightDrive.getPoseEstimate().getX(),
                straightDrive.getPoseEstimate().getY() + strafeDistance), heading));
        straightDrive.getLocalizer().update();
        RobotLog.dd("Updated Position (X, Y, Heading)", straightDrive.getPoseEstimate().toString());
    }

    public void odometryStrafe(double power, double inches, boolean right){
        double counts = inches / (2 * PI * 1.25) * 1550.0;
        int sidewaysStart = hwMap.rightIntake.getCurrentPosition();

        if(!right)
            power = -power;

        hwMap.frontRight.setPower(-power);
        hwMap.frontLeft.setPower(power);
        hwMap.backRight.setPower(power * rear_ratio);
        hwMap.backLeft.setPower(-power * rear_ratio);

        while (opMode.opModeIsActive()) {
            int sideways = hwMap.rightIntake.getCurrentPosition();

            int sidewaysDiff = Math.abs(sideways - sidewaysStart);

            if (opMode != null) {
                opMode.telemetry.addData("Side", sidewaysDiff);
                opMode.telemetry.addData("Target", counts);
                opMode.telemetry.update();
            }

            if (sidewaysDiff >= counts) {
                break;
            }
        }
        hwMap.frontRight.setPower(0);
        hwMap.frontLeft.setPower(0);
        hwMap.backRight.setPower(0);
        hwMap.backLeft.setPower(0);
    }

    private double getIMUAngle(){
        return imu.getAngularOrientation().firstAngle;
    }
}
