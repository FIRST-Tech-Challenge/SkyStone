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
import org.firstinspires.ftc.teamcode.PID.localizer.VuforiaCamLocalizer;
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
    private SampleMecanumDriveBase _drive;
    private int step_count = 0;
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
    //VuforiaCamLocalizer vu;

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
        _drive = strafeDrive;
        this.imu = imu;
        //vu = new VuforiaCamLocalizer(hardwareMap);
    }

    /*
    input: last pose from previous move;
    return: drive instance;
     */
    public SampleMecanumDriveBase DriveBuilderReset(boolean isStrafe, boolean init_imu, String label) {
        currentPos = _drive.getPoseEstimate();
        Pose2d error_pose = _drive.follower.getLastError();
        RobotLog.dd(TAG, "start new step: %s, count[%d], currentPos %s, errorPos %s",
                label, step_count++, currentPos.toString(), error_pose.toString());
        //RobotLog.dd(TAG, "vuforia localization info: %s", vu.getPoseEstimate().toString());

        if (DriveConstantsPID.RECREATE_DRIVE_AND_BUILDER)
            _drive = new SampleMecanumDriveREV(hardwareMap, isStrafe, init_imu);
        else
            _drive.resetFollowerWithParameters(isStrafe);

        _drive.getLocalizer().setPoseEstimate(currentPos);
        _drive.getLocalizer().update();
        if (!isStrafe) {
            builder = new TrajectoryBuilder(_drive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);
        } else {
            builder = new TrajectoryBuilder(_drive.getPoseEstimate(), DriveConstantsPID.STRAFE_BASE_CONSTRAINTS);
        }
        RobotLog.dd(TAG, "drive and builder reset, initialized with pose: " + _drive.getPoseEstimate().toString());
        return _drive;
    }

    public void RedQuary(int[] skystonePositions) {
        DriveConstantsPID.updateConstantsFromProperties();
        RobotLog.dd(TAG, "skystone positions[0]" + skystonePositions[0]);
        switch (skystonePositions[0]) {
            case 1:
                double yCoordMvmtPlane = -44.5;
                double wallSkyStoneX = -44.0;
                double furtherMostSkyStoneX = -21.0;
                double firstRegularStoneX = -34.0;
                double foundationX = 46.0;
                double strafeDistance = 9.5;

                yCoordMvmtPlane = -41.0;
                wallSkyStoneX = -41.5;
                furtherMostSkyStoneX = -20.0;
                firstRegularStoneX = -34.0;
                foundationX = 46.0;
                strafeDistance = 8.0;

                transferReset();
                initIntakeClaw();
                init();

                DriveBuilderReset(true, false, "step1, to prepareGrab");
                //straightDrive.resetFollowerWithParameters(true);

                prepGrab(FieldPosition.RED_QUARY);

                //straightDrive.getLocalizer().setPoseEstimate(straightDrive.getPoseEstimate());
                //straightDrive.getLocalizer().update();
                //builder = new TrajectoryBuilder(straightDrive.getPoseEstimate(), DriveConstantsPID.STRAFE_BASE_CONSTRAINTS);

                builder = builder
                        .setReversed(false).strafeTo(new Vector2d(wallSkyStoneX, yCoordMvmtPlane + (strafeDistance - 3.5)));
                trajectory = builder.build();   //x - 2.812, y + 7.984
                _drive.followTrajectorySync(trajectory);

                //straightDrive.resetFollowerWithParameters(false);
                grabStone(FieldPosition.RED_QUARY);

                DriveBuilderReset(true, false, "step2, after grab, to strafe back");
                //builder = new TrajectoryBuilder(straightDrive.getPoseEstimate(), DriveConstantsPID.STRAFE_BASE_CONSTRAINTS);
                builder = builder
                        .setReversed(false).strafeTo(new Vector2d(wallSkyStoneX, yCoordMvmtPlane));
                trajectory = builder.build();   //x - 2.812, y + 7.984
                _drive.followTrajectorySync(trajectory);

                //straightDrive.resetFollowerWithParameters(false);
                DriveBuilderReset(false, false, "step3, after strafe, to go straight");

                //builder = new TrajectoryBuilder(_drive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);
                builder = builder
                        .setReversed(false).lineTo(new Vector2d(foundationX, yCoordMvmtPlane));
                trajectory = builder.build();   //x - 2.812, y + 7.984
                _drive.followTrajectorySync(trajectory);

                DriveBuilderReset(true, false, "step4, to drop stone");
                //straightDrive.resetFollowerWithParameters(true);

                //builder = new TrajectoryBuilder(_drive.getPoseEstimate(), DriveConstantsPID.STRAFE_BASE_CONSTRAINTS);
                builder = builder
                        .setReversed(false).strafeTo(new Vector2d(foundationX, yCoordMvmtPlane + strafeDistance + 1));
                trajectory = builder.build();   //x - 2.812, y + 7.984
                _drive.followTrajectorySync(trajectory);

                dropStone(FieldPosition.RED_QUARY);

                DriveBuilderReset(true, false, "step5, after drop");
                //builder = new TrajectoryBuilder(straightDrive.getPoseEstimate(), DriveConstantsPID.STRAFE_BASE_CONSTRAINTS);
                builder = builder
                        .setReversed(false).strafeTo(new Vector2d(foundationX, yCoordMvmtPlane));
                trajectory = builder.build();   //x - 2.812, y + 7.984
                _drive.followTrajectorySync(trajectory);

                DriveBuilderReset(false, false, "step6, straight move back?");
                //_drive.resetFollowerWithParameters(false);

                //builder = new TrajectoryBuilder(straightDrive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);
                builder = builder
                        .setReversed(true).lineTo(new Vector2d(furtherMostSkyStoneX, yCoordMvmtPlane));
                trajectory = builder.build();   //x - 2.812, y + 7.984
                _drive.followTrajectorySync(trajectory);

                //straightDrive.resetFollowerWithParameters(true);
                DriveBuilderReset(true, false, "step7, to prepGrab");
                prepGrab(FieldPosition.RED_QUARY);

                //builder = new TrajectoryBuilder(straightDrive.getPoseEstimate(), DriveConstantsPID.STRAFE_BASE_CONSTRAINTS);
                builder = builder
                        .setReversed(false).strafeTo(new Vector2d(furtherMostSkyStoneX, yCoordMvmtPlane + strafeDistance + 2));
                trajectory = builder.build();   //x - 2.812, y + 7.984
                _drive.followTrajectorySync(trajectory);

                grabStone(FieldPosition.RED_QUARY);

                DriveBuilderReset(true, false, "step8, after grab");
                //builder = new TrajectoryBuilder(straightDrive.getPoseEstimate(), DriveConstantsPID.STRAFE_BASE_CONSTRAINTS);
                builder = builder
                        .setReversed(false).strafeTo(new Vector2d(furtherMostSkyStoneX, yCoordMvmtPlane));
                trajectory = builder.build();   //x - 2.812, y + 7.984
                _drive.followTrajectorySync(trajectory);

                //_drive.resetFollowerWithParameters(false);
                DriveBuilderReset(false, false, "step9, strafe back?");

                //builder = new TrajectoryBuilder(straightDrive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);
                builder = builder
                        .setReversed(false).lineTo(new Vector2d(foundationX + 5, yCoordMvmtPlane));
                trajectory = builder.build();   //x - 2.812, y + 7.984
                _drive.followTrajectorySync(trajectory);

                //_drive.resetFollowerWithParameters(true);
                DriveBuilderReset(true, false, "step10, straight move");

                //builder = new TrajectoryBuilder(straightDrive.getPoseEstimate(), DriveConstantsPID.STRAFE_BASE_CONSTRAINTS);
                builder = builder
                        .setReversed(false).strafeTo(new Vector2d(foundationX + 5, yCoordMvmtPlane + strafeDistance + 3));
                trajectory = builder.build();   //x - 2.812, y + 7.984
                _drive.followTrajectorySync(trajectory);

                dropStone(FieldPosition.RED_QUARY);
                //_drive.resetFollowerWithParameters(false);
                DriveBuilderReset(false, false, "step11, after drop");

                double theta = _drive.getExternalHeading() >= 0 ? _drive.getExternalHeading() :
                        _drive.getExternalHeading() + 2 * PI;

                if(theta > PI)
                    _drive.turnSync(-(_drive.getExternalHeading() - 3 * PI / 2));
                else
                    _drive.turnSync(-(_drive.getExternalHeading() + 2 * PI - 3 * PI / 2));

                hwMap.foundationLock.setPosition(TeleopConstants.foundationLockUnlock);
                hwMap.transferLock.setPosition(TeleopConstants.transferLockPosOut);

                DriveBuilderReset(false, false, "step12, after turn");
                //builder = new TrajectoryBuilder(straightDrive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);
                builder = builder.setReversed(true).lineTo(new Vector2d(_drive.getPoseEstimate().getX(),
                        _drive.getPoseEstimate().getY() + 7));
                trajectory = builder.build();   //x - 2.812, y + 7.984
                _drive.followTrajectorySync(trajectory);

                hwMap.foundationLock.setPosition(TeleopConstants.foundationLockLock);
                hwMap.transferLock.setPosition(TeleopConstants.transferLockPosPlatform);

                try{
                    Thread.sleep(200);
                } catch (Exception e){}

                DriveBuilderReset(false, false, "step13, ???");
                //builder = new TrajectoryBuilder(_drive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);
                builder = builder.setReversed(false)
                        .splineTo(new Pose2d(new Vector2d(_drive.getPoseEstimate().getX() - 15,
                                _drive.getPoseEstimate().getY() - 27), 7 * PI / 6));
                trajectory = builder.build();   //x - 2.812, y + 7.984
                _drive.followTrajectorySync(trajectory);

                hwMap.foundationLock.setPosition(TeleopConstants.foundationLockUnlock);
                hwMap.transferLock.setPosition(TeleopConstants.transferLockPosOut);

                try{
                    Thread.sleep(200);
                } catch (Exception e){}

                DriveBuilderReset(false, false, "step14, to spline");
                //builder = new TrajectoryBuilder(_drive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);
                builder = builder
                        .setReversed(false).splineTo(new Pose2d(new Vector2d(5, -48.0), PI));
                trajectory = builder.build();   //x - 2.812, y + 7.984
                _drive.followTrajectorySync(trajectory);

                /*straightDrive = DriveBuilderReset(false, false, "step1, after strafe");
                if (DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {

                    builder = builder
                            .setReversed(true).lineTo(new Vector2d(wallSkyStoneX, straightDrive.getPoseEstimate().getY()));

                } else {
                    builder = builder.strafeTo(new Vector2d(straightDrive.getPoseEstimate().getX(), -30)).setReversed(true)
                            .lineTo(new Vector2d(-50, -30));
                }
                trajectory = builder.build();   //x - 2.812, y + 7.984
                straightDrive.followTrajectorySync(trajectory);
                currentPos = straightDrive.getPoseEstimate();

                prepGrab(FieldPosition.RED_QUARY);

                strafeDrive = DriveBuilderReset(true, false, "step1, after strafe");
                builder = new TrajectoryBuilder(strafeDrive.getPoseEstimate(), DriveConstantsPID.STRAFE_BASE_CONSTRAINTS);   //-34.752, -63.936
                if (DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {

                    builder = builder
                            .strafeTo(new Vector2d(strafeDrive.getPoseEstimate().getX(), yCoordMvmtPlane + strafeDistance));

                } else {
                    builder = builder.strafeTo(new Vector2d(strafeDrive.getPoseEstimate().getX(), -30)).setReversed(true)
                            .lineTo(new Vector2d(-50, -30));
                }
                trajectory = builder.build();   //x - 2.812, y + 7.984
                strafeDrive.followTrajectorySync(trajectory);

                grabStone(FieldPosition.RED_QUARY);

                strafeDrive = DriveBuilderReset(true, false, "step2, after grab stone ");
                if (DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {
                    builder = builder.setReversed(false).strafeTo(new Vector2d(wallSkyStoneX, yCoordMvmtPlane));  //-52, -39
                } else {
                    builder = builder.setReversed(false).strafeTo(new Vector2d(-50, -40))
                            .lineTo(new Vector2d(50, -40)).strafeTo(new Vector2d(50, -30));
                }
                trajectory = builder.build();
                strafeDrive.followTrajectorySync(trajectory);
                currentPos = strafeDrive.getPoseEstimate();

                straightDrive = DriveBuilderReset(false, false, "step3");
                if (DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {
                    builder = builder.setReversed(false)
                            .lineTo(new Vector2d(foundationX, straightDrive.getPoseEstimate().getY()));
                    //-52, -39
                } else {
                    builder = builder.setReversed(false).strafeTo(new Vector2d(-50, -40))
                            .lineTo(new Vector2d(50, -40)).strafeTo(new Vector2d(50, -30));
                }
                trajectory = builder.build();
                straightDrive.followTrajectorySync(trajectory);
                currentPos = straightDrive.getPoseEstimate();


                strafeDrive = DriveBuilderReset(true, false, "step4");
                if (DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {
                    builder = builder.strafeTo(new Vector2d(foundationX, yCoordMvmtPlane + strafeDistance));
                } else {
                    builder = builder.strafeTo(new Vector2d(50, -40)).setReversed(true).lineTo(new Vector2d(-20, -40))
                            .strafeTo(new Vector2d(-20, -28));
                }
                trajectory = builder.build();
                strafeDrive.followTrajectorySync(trajectory);
                currentPos = strafeDrive.getPoseEstimate();

                dropStone(FieldPosition.RED_QUARY);

                strafeDrive = DriveBuilderReset(true, false, "step5, after drop 1st stone");
                if (DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {
                    builder = builder.strafeTo(new Vector2d(foundationX, yCoordMvmtPlane));
                } else {
                    builder = builder.strafeTo(new Vector2d(50, -40)).setReversed(true).lineTo(new Vector2d(-20, -40))
                            .strafeTo(new Vector2d(-20, -28));
                }
                trajectory = builder.build();
                strafeDrive.followTrajectorySync(trajectory);
                currentPos = strafeDrive.getPoseEstimate();

                straightDrive = DriveBuilderReset(false, false, "step6");
                if (DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {
                    builder = builder
                            .setReversed(true).lineTo(new Vector2d(furtherMostSkyStoneX, straightDrive.getPoseEstimate().getY()));
                } else {
                    builder = builder.strafeTo(new Vector2d(50, -40)).setReversed(true).lineTo(new Vector2d(-20, -40))
                            .strafeTo(new Vector2d(-20, -28));
                }
                trajectory = builder.build();
                straightDrive.followTrajectorySync(trajectory);
                currentPos = straightDrive.getPoseEstimate();

                prepGrab(FieldPosition.RED_QUARY);

                strafeDrive = DriveBuilderReset(true, false, "step7");
                if (DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {
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

                strafeDrive = DriveBuilderReset(true, false, "step8, grabbed 2nd stone");
                if (DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {
                    builder = builder.setReversed(false).strafeTo(new Vector2d(furtherMostSkyStoneX, yCoordMvmtPlane));
                } else {
                    builder = builder.setReversed(false).strafeTo(new Vector2d(-20, -40)).lineTo(new Vector2d(50, -40))
                            .strafeTo(new Vector2d(50, -30));
                }
                trajectory = builder.build();
                strafeDrive.followTrajectorySync(trajectory);
                currentPos = strafeDrive.getPoseEstimate();

                straightDrive = DriveBuilderReset(false, false, "step9");
                if (DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {
                    builder = builder.setReversed(false)
                            .lineTo(new Vector2d(foundationX, yCoordMvmtPlane));
                } else {
                    builder = builder.setReversed(false).strafeTo(new Vector2d(-20, -40)).lineTo(new Vector2d(50, -40))
                            .strafeTo(new Vector2d(50, -30));
                }
                trajectory = builder.build();
                straightDrive.followTrajectorySync(trajectory);
                currentPos = straightDrive.getPoseEstimate();

                strafeDrive = DriveBuilderReset(true, false, "step10");
                if (DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {
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

                /*strafeDrive = DriveBuilderReset(true, false, "step11, dropped 2nd stone");
                if (DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {
                    builder = builder.strafeTo(new Vector2d(foundationX, yCoordMvmtPlane));
                }
                trajectory = builder.build();
                strafeDrive.followTrajectorySync(trajectory);
                currentPos = strafeDrive.getPoseEstimate();

                straightDrive = DriveBuilderReset(false, false, "step12");
                if (DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {
                    builder = builder
                            .setReversed(true).lineTo(new Vector2d(firstRegularStoneX, yCoordMvmtPlane));
                }
                trajectory = builder.build();
                straightDrive.followTrajectorySync(trajectory);
                currentPos = straightDrive.getPoseEstimate();

                prepGrab(FieldPosition.RED_QUARY);

                strafeDrive = DriveBuilderReset(true, false, "step13");
                if (DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {
                    builder = builder
                            .strafeTo(new Vector2d(firstRegularStoneX, yCoordMvmtPlane + strafeDistance));
                }
                trajectory = builder.build();
                strafeDrive.followTrajectorySync(trajectory);
                currentPos = strafeDrive.getPoseEstimate();

                grabStone(FieldPosition.RED_QUARY);

                strafeDrive = DriveBuilderReset(true, false, "step14, grabbed 3rd stone");
                if (DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {
                    builder = builder.setReversed(false).strafeTo(new Vector2d(firstRegularStoneX, yCoordMvmtPlane));
                }
                trajectory = builder.build();
                strafeDrive.followTrajectorySync(trajectory);
                currentPos = strafeDrive.getPoseEstimate();

                straightDrive = DriveBuilderReset(false, false, "step15");

                if (DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {
                    builder = builder.setReversed(false)
                            .lineTo(new Vector2d(foundationX, yCoordMvmtPlane));
                }
                trajectory = builder.build();
                straightDrive.followTrajectorySync(trajectory);
                currentPos = straightDrive.getPoseEstimate();

                strafeDrive = DriveBuilderReset(true, false, "step16");

                if (DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {
                    builder = builder.setReversed(false)
                            .strafeTo(new Vector2d(foundationX, yCoordMvmtPlane + strafeDistance));
                }
                trajectory = builder.build();
                strafeDrive.followTrajectorySync(trajectory);
                currentPos = strafeDrive.getPoseEstimate();

                DriveConstantsPID.keep_vuforia_running = false;

                dropStone(FieldPosition.RED_QUARY);*/

                /*hwMap.foundationLock.setPosition(TeleopConstants.foundationLockUnlock);
                hwMap.transferLock.setPosition(TeleopConstants.transferLockPosOut);

                strafeDrive = DriveBuilderReset(true, false, "step17.5");

                if (DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {
                    builder = builder.setReversed(false).strafeTo(new Vector2d(foundationX, yCoordMvmtPlane - 6));
                    //.strafeTo(new Vector2d(42, -30));
                }
                trajectory = builder.build();
                strafeDrive.followTrajectorySync(trajectory);
                currentPos = strafeDrive.getPoseEstimate();

                straightDrive = DriveBuilderReset(true, false, "step17.0, dropped 3rd stones");

                if (getIMUAngle() < Math.PI)
                    straightDrive.turnSync(-((getIMUAngle() + 2 * Math.PI) - 3 * Math.PI / 2));
                else
                    straightDrive.turnSync(-(getIMUAngle() - 3 * Math.PI / 2));
                currentPos = strafeDrive.getPoseEstimate();

                straightDrive = DriveBuilderReset(false, false, "step18");

                if (DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {
                    builder = builder.setReversed(false)
                            .setReversed(true).lineTo(new Vector2d(straightDrive.getPoseEstimate().getX(), -25));
                    //.strafeTo(new Vector2d(42, -30));
                }
                trajectory = builder.build();
                straightDrive.followTrajectorySync(trajectory);
                currentPos = straightDrive.getPoseEstimate();

                hwMap.foundationLock.setPosition(TeleopConstants.foundationLockLock);
                hwMap.transferLock.setPosition(TeleopConstants.transferLockPosUp);

                try {
                    Thread.sleep(300);
                } catch (Exception e) {
                }

                straightDrive = DriveBuilderReset(false, false, "step19");

                if (DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {
                    builder = builder.setReversed(false)//.strafeTo(new Vector2d(-24, -40))
                            .setReversed(false).lineTo(new Vector2d(straightDrive.getPoseEstimate().getX() - 10, -55));
                    //.strafeTo(new Vector2d(42, -30));
                }
                trajectory = builder.build();
                straightDrive.followTrajectorySync(trajectory);
                currentPos = straightDrive.getPoseEstimate();
                RobotLog.dd(TAG, "step20, " + currentPos.toString());

                straightDrive.turnSync(-Math.toRadians(70));
                RobotLog.dd(TAG, "step21, " + straightDrive.getPoseEstimate().toString());
                currentPos = straightDrive.getPoseEstimate();

                straightDrive = DriveBuilderReset(false, false, "step18");

                hwMap.foundationLock.setPosition(TeleopConstants.foundationLockUnlock);
                hwMap.transferLock.setPosition(TeleopConstants.transferLockPosOut);
                try {
                    Thread.sleep(300);
                } catch (Exception e) {
                }

                if (DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {
                    //builder = builder.splineTo(new Pose2d(straightDrive.getPoseEstimate().getX() - 12, straightDrive.getPoseEstimate().getY() - 24, PI / 2));
                    builder = builder.forward(30);
                    //.strafeTo(new Vector2d(42, -30));
                }
                trajectory = builder.build();
                straightDrive.followTrajectorySync(trajectory);*/
                break;
            case 2:
                yCoordMvmtPlane = -44.5;
                wallSkyStoneX = -53.5;
                furtherMostSkyStoneX = -29.5;
                firstRegularStoneX = -34.0;
                foundationX = 46.0;
                strafeDistance = 9.5;

                yCoordMvmtPlane = -42.0;
                wallSkyStoneX = -51.0;
                furtherMostSkyStoneX = -28.5;
                firstRegularStoneX = -34.0;
                foundationX = 46.0;
                strafeDistance = 8.0;

                init();

                straightDrive.resetFollowerWithParameters(true);

                prepGrab(FieldPosition.RED_QUARY);

                straightDrive.getLocalizer().setPoseEstimate(straightDrive.getPoseEstimate());
                straightDrive.getLocalizer().update();
                builder = new TrajectoryBuilder(straightDrive.getPoseEstimate(), DriveConstantsPID.STRAFE_BASE_CONSTRAINTS);
                builder = builder
                        .setReversed(false).strafeTo(new Vector2d(wallSkyStoneX, yCoordMvmtPlane + (strafeDistance - 1)));
                trajectory = builder.build();   //x - 2.812, y + 7.984
                straightDrive.followTrajectorySync(trajectory);

                //straightDrive.resetFollowerWithParameters(false);
                transferReset();
                initIntakeClaw();
                grabStone(FieldPosition.RED_QUARY);
                try{
                    Thread.sleep(300);
                } catch(Exception e){}

                builder = new TrajectoryBuilder(straightDrive.getPoseEstimate(), DriveConstantsPID.STRAFE_BASE_CONSTRAINTS);
                builder = builder
                        .setReversed(false).strafeTo(new Vector2d(wallSkyStoneX, yCoordMvmtPlane));
                trajectory = builder.build();   //x - 2.812, y + 7.984
                straightDrive.followTrajectorySync(trajectory);

                straightDrive.resetFollowerWithParameters(false);

                builder = new TrajectoryBuilder(straightDrive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);
                builder = builder
                        .setReversed(false).lineTo(new Vector2d(foundationX, yCoordMvmtPlane));
                trajectory = builder.build();   //x - 2.812, y + 7.984
                straightDrive.followTrajectorySync(trajectory);

                straightDrive.resetFollowerWithParameters(true);

                builder = new TrajectoryBuilder(straightDrive.getPoseEstimate(), DriveConstantsPID.STRAFE_BASE_CONSTRAINTS);
                builder = builder
                        .setReversed(false).strafeTo(new Vector2d(foundationX, yCoordMvmtPlane + strafeDistance + 1));
                trajectory = builder.build();   //x - 2.812, y + 7.984
                straightDrive.followTrajectorySync(trajectory);

                dropStone(FieldPosition.RED_QUARY);

                builder = new TrajectoryBuilder(straightDrive.getPoseEstimate(), DriveConstantsPID.STRAFE_BASE_CONSTRAINTS);
                builder = builder
                        .setReversed(false).strafeTo(new Vector2d(foundationX, yCoordMvmtPlane));
                trajectory = builder.build();   //x - 2.812, y + 7.984
                straightDrive.followTrajectorySync(trajectory);

                straightDrive.resetFollowerWithParameters(false);

                builder = new TrajectoryBuilder(straightDrive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);
                builder = builder
                        .setReversed(true).lineTo(new Vector2d(furtherMostSkyStoneX, yCoordMvmtPlane));
                trajectory = builder.build();   //x - 2.812, y + 7.984
                straightDrive.followTrajectorySync(trajectory);

                straightDrive.resetFollowerWithParameters(true);
                prepGrab(FieldPosition.RED_QUARY);

                builder = new TrajectoryBuilder(straightDrive.getPoseEstimate(), DriveConstantsPID.STRAFE_BASE_CONSTRAINTS);
                builder = builder
                        .setReversed(false).strafeTo(new Vector2d(furtherMostSkyStoneX, yCoordMvmtPlane + strafeDistance + 2));
                trajectory = builder.build();   //x - 2.812, y + 7.984
                straightDrive.followTrajectorySync(trajectory);

                grabStone(FieldPosition.RED_QUARY);

                builder = new TrajectoryBuilder(straightDrive.getPoseEstimate(), DriveConstantsPID.STRAFE_BASE_CONSTRAINTS);
                builder = builder
                        .setReversed(false).strafeTo(new Vector2d(furtherMostSkyStoneX, yCoordMvmtPlane));
                trajectory = builder.build();   //x - 2.812, y + 7.984
                straightDrive.followTrajectorySync(trajectory);

                straightDrive.resetFollowerWithParameters(false);

                builder = new TrajectoryBuilder(straightDrive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);
                builder = builder
                        .setReversed(false).lineTo(new Vector2d(foundationX + 7, yCoordMvmtPlane));
                trajectory = builder.build();   //x - 2.812, y + 7.984
                straightDrive.followTrajectorySync(trajectory);

                straightDrive.resetFollowerWithParameters(true);

                builder = new TrajectoryBuilder(straightDrive.getPoseEstimate(), DriveConstantsPID.STRAFE_BASE_CONSTRAINTS);
                builder = builder
                        .setReversed(false).strafeTo(new Vector2d(foundationX + 7, yCoordMvmtPlane + strafeDistance + 3));
                trajectory = builder.build();   //x - 2.812, y + 7.984
                straightDrive.followTrajectorySync(trajectory);

                dropStone(FieldPosition.RED_QUARY);
                straightDrive.resetFollowerWithParameters(false);

                theta = straightDrive.getExternalHeading() >= 0 ? straightDrive.getExternalHeading() :
                        straightDrive.getExternalHeading() + 2 * PI;
                if(theta > PI)
                    straightDrive.turnSync(-(straightDrive.getExternalHeading() - 3 * PI / 2));
                else
                    straightDrive.turnSync(-(straightDrive.getExternalHeading() + 2 * PI - 3 * PI / 2));

                hwMap.foundationLock.setPosition(TeleopConstants.foundationLockUnlock);
                hwMap.transferLock.setPosition(TeleopConstants.transferLockPosOut);

                builder = new TrajectoryBuilder(straightDrive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);
                builder = builder.setReversed(true).lineTo(new Vector2d(straightDrive.getPoseEstimate().getX(),
                        straightDrive.getPoseEstimate().getY() + 6));
                trajectory = builder.build();   //x - 2.812, y + 7.984
                straightDrive.followTrajectorySync(trajectory);

                hwMap.foundationLock.setPosition(TeleopConstants.foundationLockLock);
                hwMap.transferLock.setPosition(TeleopConstants.transferLockPosPlatform);

                try{
                    Thread.sleep(200);
                } catch (Exception e){}

                builder = new TrajectoryBuilder(straightDrive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);
                builder = builder.setReversed(false)
                        .splineTo(new Pose2d(new Vector2d(straightDrive.getPoseEstimate().getX() - 10,
                                straightDrive.getPoseEstimate().getY() - 26), 7 *PI / 6));
                trajectory = builder.build();   //x - 2.812, y + 7.984
                straightDrive.followTrajectorySync(trajectory);

                hwMap.foundationLock.setPosition(TeleopConstants.foundationLockUnlock);
                hwMap.transferLock.setPosition(TeleopConstants.transferLockPosOut);

                try{
                    Thread.sleep(200);
                } catch (Exception e){}

                builder = new TrajectoryBuilder(straightDrive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);
                builder = builder
                        .setReversed(false).splineTo(new Pose2d(new Vector2d(5, -49), PI));
                trajectory = builder.build();   //x - 2.812, y + 7.984
                straightDrive.followTrajectorySync(trajectory);

                /*hwMap.parkingServo.setPosition(TeleopConstants.parkingServoPosLock);
                hwMap.foundationLock.setPosition(TeleopConstants.foundationLockUnlock);
                hwMap.transferLock.setPosition(TeleopConstants.transferLockPosPlatform);

                straightDrive = DriveBuilderReset(false, false, "step1, after strafe");
                if (DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {

                    builder = builder
                            .setReversed(true).lineTo(new Vector2d(wallSkyStoneX, straightDrive.getPoseEstimate().getY()));

                } else {
                    builder = builder.strafeTo(new Vector2d(straightDrive.getPoseEstimate().getX(), -30)).setReversed(true)
                            .lineTo(new Vector2d(-50, -30));
                }
                trajectory = builder.build();   //x - 2.812, y + 7.984
                straightDrive.followTrajectorySync(trajectory);
                currentPos = straightDrive.getPoseEstimate();

                prepGrab(FieldPosition.RED_QUARY);

                strafeDrive = DriveBuilderReset(true, false, "step1, after strafe");
                builder = new TrajectoryBuilder(strafeDrive.getPoseEstimate(), DriveConstantsPID.STRAFE_BASE_CONSTRAINTS);   //-34.752, -63.936
                if (DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {

                    builder = builder
                            .strafeTo(new Vector2d(strafeDrive.getPoseEstimate().getX(), yCoordMvmtPlane + strafeDistance));

                } else {
                    builder = builder.strafeTo(new Vector2d(strafeDrive.getPoseEstimate().getX(), -30)).setReversed(true)
                            .lineTo(new Vector2d(-50, -30));
                }
                trajectory = builder.build();   //x - 2.812, y + 7.984
                strafeDrive.followTrajectorySync(trajectory);

                grabStone(FieldPosition.RED_QUARY);
                transferReset();
                initIntakeClaw();

                strafeDrive = DriveBuilderReset(true, false, "step2, after grab stone ");
                if (DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {
                    builder = builder.setReversed(false).strafeTo(new Vector2d(wallSkyStoneX, yCoordMvmtPlane));  //-52, -39
                } else {
                    builder = builder.setReversed(false).strafeTo(new Vector2d(-50, -40))
                            .lineTo(new Vector2d(50, -40)).strafeTo(new Vector2d(50, -30));
                }
                trajectory = builder.build();
                strafeDrive.followTrajectorySync(trajectory);
                currentPos = strafeDrive.getPoseEstimate();

                straightDrive = DriveBuilderReset(false, false, "step3");
                if (DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {
                    builder = builder.setReversed(false)
                            .lineTo(new Vector2d(foundationX, straightDrive.getPoseEstimate().getY()));
                    //-52, -39
                } else {
                    builder = builder.setReversed(false).strafeTo(new Vector2d(-50, -40))
                            .lineTo(new Vector2d(50, -40)).strafeTo(new Vector2d(50, -30));
                }
                trajectory = builder.build();
                straightDrive.followTrajectorySync(trajectory);
                currentPos = straightDrive.getPoseEstimate();


                strafeDrive = DriveBuilderReset(true, false, "step4");
                if (DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {
                    builder = builder.strafeTo(new Vector2d(foundationX, yCoordMvmtPlane + strafeDistance));
                } else {
                    builder = builder.strafeTo(new Vector2d(50, -40)).setReversed(true).lineTo(new Vector2d(-20, -40))
                            .strafeTo(new Vector2d(-20, -28));
                }
                trajectory = builder.build();
                strafeDrive.followTrajectorySync(trajectory);
                currentPos = strafeDrive.getPoseEstimate();

                dropStone(FieldPosition.RED_QUARY);

                strafeDrive = DriveBuilderReset(true, false, "step5, after drop 1st stone");
                if (DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {
                    builder = builder.strafeTo(new Vector2d(foundationX, yCoordMvmtPlane));
                } else {
                    builder = builder.strafeTo(new Vector2d(50, -40)).setReversed(true).lineTo(new Vector2d(-20, -40))
                            .strafeTo(new Vector2d(-20, -28));
                }
                trajectory = builder.build();
                strafeDrive.followTrajectorySync(trajectory);
                currentPos = strafeDrive.getPoseEstimate();

                straightDrive = DriveBuilderReset(false, false, "step6");
                if (DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {
                    builder = builder
                            .setReversed(true).lineTo(new Vector2d(furtherMostSkyStoneX, straightDrive.getPoseEstimate().getY()));
                } else {
                    builder = builder.strafeTo(new Vector2d(50, -40)).setReversed(true).lineTo(new Vector2d(-20, -40))
                            .strafeTo(new Vector2d(-20, -28));
                }
                trajectory = builder.build();
                straightDrive.followTrajectorySync(trajectory);
                currentPos = straightDrive.getPoseEstimate();

                prepGrab(FieldPosition.RED_QUARY);

                strafeDrive = DriveBuilderReset(true, false, "step7");
                if (DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {
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

                strafeDrive = DriveBuilderReset(true, false, "step8, grabbed 2nd stone");
                if (DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {
                    builder = builder.setReversed(false).strafeTo(new Vector2d(furtherMostSkyStoneX, yCoordMvmtPlane));
                } else {
                    builder = builder.setReversed(false).strafeTo(new Vector2d(-20, -40)).lineTo(new Vector2d(50, -40))
                            .strafeTo(new Vector2d(50, -30));
                }
                trajectory = builder.build();
                strafeDrive.followTrajectorySync(trajectory);
                currentPos = strafeDrive.getPoseEstimate();

                straightDrive = DriveBuilderReset(false, false, "step9");
                if (DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {
                    builder = builder.setReversed(false)
                            .lineTo(new Vector2d(foundationX, yCoordMvmtPlane));
                } else {
                    builder = builder.setReversed(false).strafeTo(new Vector2d(-20, -40)).lineTo(new Vector2d(50, -40))
                            .strafeTo(new Vector2d(50, -30));
                }
                trajectory = builder.build();
                straightDrive.followTrajectorySync(trajectory);
                currentPos = straightDrive.getPoseEstimate();

                strafeDrive = DriveBuilderReset(true, false, "step10");
                if (DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {
                    builder = builder.setReversed(false)
                            .strafeTo(new Vector2d(foundationX, yCoordMvmtPlane + strafeDistance));
                } else {
                    builder = builder.setReversed(false).strafeTo(new Vector2d(-20, -40)).lineTo(new Vector2d(50, -40))
                            .strafeTo(new Vector2d(50, -30));
                }
                trajectory = builder.build();
                strafeDrive.followTrajectorySync(trajectory);
                currentPos = strafeDrive.getPoseEstimate();

                dropStone(FieldPosition.RED_QUARY);*/

                /*strafeDrive = DriveBuilderReset(true, false, "step11, dropped 2nd stone");
                if (DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {
                    builder = builder.strafeTo(new Vector2d(foundationX, yCoordMvmtPlane));
                }
                trajectory = builder.build();
                strafeDrive.followTrajectorySync(trajectory);
                currentPos = strafeDrive.getPoseEstimate();

                straightDrive = DriveBuilderReset(false, false, "step12");
                if (DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {
                    builder = builder
                            .setReversed(true).lineTo(new Vector2d(firstRegularStoneX, yCoordMvmtPlane));
                }
                trajectory = builder.build();
                straightDrive.followTrajectorySync(trajectory);
                currentPos = straightDrive.getPoseEstimate();

                prepGrab(FieldPosition.RED_QUARY);

                strafeDrive = DriveBuilderReset(true, false, "step13");
                if (DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {
                    builder = builder
                            .strafeTo(new Vector2d(firstRegularStoneX, yCoordMvmtPlane + strafeDistance));
                }
                trajectory = builder.build();
                strafeDrive.followTrajectorySync(trajectory);
                currentPos = strafeDrive.getPoseEstimate();

                grabStone(FieldPosition.RED_QUARY);

                strafeDrive = DriveBuilderReset(true, false, "step14, grabbed 3rd stone");
                if (DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {
                    builder = builder.setReversed(false).strafeTo(new Vector2d(firstRegularStoneX, yCoordMvmtPlane));
                }
                trajectory = builder.build();
                strafeDrive.followTrajectorySync(trajectory);
                currentPos = strafeDrive.getPoseEstimate();

                straightDrive = DriveBuilderReset(false, false, "step15");

                if (DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {
                    builder = builder.setReversed(false)
                            .lineTo(new Vector2d(foundationX, yCoordMvmtPlane));
                }
                trajectory = builder.build();
                straightDrive.followTrajectorySync(trajectory);
                currentPos = straightDrive.getPoseEstimate();

                strafeDrive = DriveBuilderReset(true, false, "step16");

                if (DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {
                    builder = builder.setReversed(false)
                            .strafeTo(new Vector2d(foundationX, yCoordMvmtPlane + strafeDistance));
                }
                trajectory = builder.build();
                strafeDrive.followTrajectorySync(trajectory);
                currentPos = strafeDrive.getPoseEstimate();

                DriveConstantsPID.keep_vuforia_running = false;

                dropStone(FieldPosition.RED_QUARY);*/

                /*hwMap.parkingServo.setPosition(TeleopConstants.parkingServoPosLock);
                hwMap.foundationLock.setPosition(TeleopConstants.foundationLockUnlock);
                hwMap.transferLock.setPosition(TeleopConstants.transferLockPosOut);

                strafeDrive = DriveBuilderReset(true, false, "step17.5");

                if (DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {
                    builder = builder.setReversed(false).strafeTo(new Vector2d(foundationX, yCoordMvmtPlane - 6));
                    //.strafeTo(new Vector2d(42, -30));
                }
                trajectory = builder.build();
                strafeDrive.followTrajectorySync(trajectory);
                currentPos = strafeDrive.getPoseEstimate();

                straightDrive = DriveBuilderReset(true, false, "step17.0, dropped 3rd stones");

                if (getIMUAngle() < Math.PI)
                    straightDrive.turnSync(-((getIMUAngle() + 2 * Math.PI) - 3 * Math.PI / 2));
                else
                    straightDrive.turnSync(-(getIMUAngle() - 3 * Math.PI / 2));
                currentPos = strafeDrive.getPoseEstimate();

                straightDrive = DriveBuilderReset(false, false, "step18");

                if (DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {
                    builder = builder.setReversed(false)
                            .setReversed(true).lineTo(new Vector2d(straightDrive.getPoseEstimate().getX(), -25));
                    //.strafeTo(new Vector2d(42, -30));
                }
                trajectory = builder.build();
                straightDrive.followTrajectorySync(trajectory);
                currentPos = straightDrive.getPoseEstimate();

                hwMap.foundationLock.setPosition(TeleopConstants.foundationLockLock);
                hwMap.transferLock.setPosition(TeleopConstants.transferLockPosUp);

                try {
                    Thread.sleep(300);
                } catch (Exception e) {
                }

                straightDrive = DriveBuilderReset(false, false, "step19");

                if (DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {
                    builder = builder.setReversed(false)//.strafeTo(new Vector2d(-24, -40))
                            .setReversed(false).lineTo(new Vector2d(straightDrive.getPoseEstimate().getX() - 10, -55));
                    //.strafeTo(new Vector2d(42, -30));
                }
                trajectory = builder.build();
                straightDrive.followTrajectorySync(trajectory);
                currentPos = straightDrive.getPoseEstimate();
                RobotLog.dd(TAG, "step20, " + currentPos.toString());

                straightDrive.turnSync(-Math.toRadians(70));
                RobotLog.dd(TAG, "step21, " + straightDrive.getPoseEstimate().toString());
                currentPos = straightDrive.getPoseEstimate();

                straightDrive = DriveBuilderReset(false, false, "step18");

                hwMap.foundationLock.setPosition(TeleopConstants.foundationLockUnlock);
                hwMap.transferLock.setPosition(TeleopConstants.transferLockPosOut);
                try {
                    Thread.sleep(300);
                } catch (Exception e) {
                }

                if (DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {
                    //builder = builder.splineTo(new Pose2d(straightDrive.getPoseEstimate().getX() - 12, straightDrive.getPoseEstimate().getY() - 24, PI / 2));
                    builder = builder.forward(30);
                    //.strafeTo(new Vector2d(42, -30));
                }
                trajectory = builder.build();
                straightDrive.followTrajectorySync(trajectory);*/
                break;
            case 3:
                yCoordMvmtPlane = -44.5;
                wallSkyStoneX = -60.5;
                furtherMostSkyStoneX = -39.0;
                firstRegularStoneX = -36.0;
                foundationX = 46.0;
                strafeDistance = 9.5;

                yCoordMvmtPlane = -41.0;
                wallSkyStoneX = -59.5;
                furtherMostSkyStoneX = -37.5;
                firstRegularStoneX = -34.0;
                foundationX = 46.0;
                strafeDistance = 8.0;

                transferReset();
                initIntakeClaw();
                init();

                straightDrive.resetFollowerWithParameters(true);

                prepGrab(FieldPosition.RED_QUARY);

                straightDrive.getLocalizer().setPoseEstimate(straightDrive.getPoseEstimate());
                straightDrive.getLocalizer().update();
                builder = new TrajectoryBuilder(straightDrive.getPoseEstimate(), DriveConstantsPID.STRAFE_BASE_CONSTRAINTS);
                builder = builder
                        .setReversed(false).strafeTo(new Vector2d(wallSkyStoneX, yCoordMvmtPlane + (strafeDistance - 1)));
                trajectory = builder.build();   //x - 2.812, y + 7.984
                straightDrive.followTrajectorySync(trajectory);

                //straightDrive.resetFollowerWithParameters(false);
                transferReset();
                initIntakeClaw();
                grabStone(FieldPosition.RED_QUARY);

                builder = new TrajectoryBuilder(straightDrive.getPoseEstimate(), DriveConstantsPID.STRAFE_BASE_CONSTRAINTS);
                builder = builder
                        .setReversed(false).strafeTo(new Vector2d(wallSkyStoneX, yCoordMvmtPlane));
                trajectory = builder.build();   //x - 2.812, y + 7.984
                straightDrive.followTrajectorySync(trajectory);

                straightDrive.resetFollowerWithParameters(false);

                builder = new TrajectoryBuilder(straightDrive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);
                builder = builder
                        .setReversed(false).lineTo(new Vector2d(foundationX, yCoordMvmtPlane));
                trajectory = builder.build();   //x - 2.812, y + 7.984
                straightDrive.followTrajectorySync(trajectory);

                straightDrive.resetFollowerWithParameters(true);

                builder = new TrajectoryBuilder(straightDrive.getPoseEstimate(), DriveConstantsPID.STRAFE_BASE_CONSTRAINTS);
                builder = builder
                        .setReversed(false).strafeTo(new Vector2d(foundationX, yCoordMvmtPlane + strafeDistance + 1));
                trajectory = builder.build();   //x - 2.812, y + 7.984
                straightDrive.followTrajectorySync(trajectory);

                dropStone(FieldPosition.RED_QUARY);

                builder = new TrajectoryBuilder(straightDrive.getPoseEstimate(), DriveConstantsPID.STRAFE_BASE_CONSTRAINTS);
                builder = builder
                        .setReversed(false).strafeTo(new Vector2d(foundationX, yCoordMvmtPlane));
                trajectory = builder.build();   //x - 2.812, y + 7.984
                straightDrive.followTrajectorySync(trajectory);

                straightDrive.resetFollowerWithParameters(false);

                builder = new TrajectoryBuilder(straightDrive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);
                builder = builder
                        .setReversed(true).lineTo(new Vector2d(furtherMostSkyStoneX, yCoordMvmtPlane));
                trajectory = builder.build();   //x - 2.812, y + 7.984
                straightDrive.followTrajectorySync(trajectory);

                straightDrive.resetFollowerWithParameters(true);
                prepGrab(FieldPosition.RED_QUARY);

                builder = new TrajectoryBuilder(straightDrive.getPoseEstimate(), DriveConstantsPID.STRAFE_BASE_CONSTRAINTS);
                builder = builder
                        .setReversed(false).strafeTo(new Vector2d(furtherMostSkyStoneX, yCoordMvmtPlane + strafeDistance + 2));
                trajectory = builder.build();   //x - 2.812, y + 7.984
                straightDrive.followTrajectorySync(trajectory);

                grabStone(FieldPosition.RED_QUARY);

                builder = new TrajectoryBuilder(straightDrive.getPoseEstimate(), DriveConstantsPID.STRAFE_BASE_CONSTRAINTS);
                builder = builder
                        .setReversed(false).strafeTo(new Vector2d(furtherMostSkyStoneX, yCoordMvmtPlane));
                trajectory = builder.build();   //x - 2.812, y + 7.984
                straightDrive.followTrajectorySync(trajectory);

                straightDrive.resetFollowerWithParameters(false);

                builder = new TrajectoryBuilder(straightDrive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);
                builder = builder
                        .setReversed(false).lineTo(new Vector2d(foundationX + 7, yCoordMvmtPlane));
                trajectory = builder.build();   //x - 2.812, y + 7.984
                straightDrive.followTrajectorySync(trajectory);

                straightDrive.resetFollowerWithParameters(true);

                builder = new TrajectoryBuilder(straightDrive.getPoseEstimate(), DriveConstantsPID.STRAFE_BASE_CONSTRAINTS);
                builder = builder
                        .setReversed(false).strafeTo(new Vector2d(foundationX + 7, yCoordMvmtPlane + strafeDistance + 3));
                trajectory = builder.build();   //x - 2.812, y + 7.984
                straightDrive.followTrajectorySync(trajectory);

                dropStone(FieldPosition.RED_QUARY);
                straightDrive.resetFollowerWithParameters(false);

                theta = straightDrive.getExternalHeading() >= 0 ? straightDrive.getExternalHeading() :
                        straightDrive.getExternalHeading() + 2 * PI;
                if(theta > PI)
                    straightDrive.turnSync(-(straightDrive.getExternalHeading() - 3 * PI / 2));
                else
                    straightDrive.turnSync(-(straightDrive.getExternalHeading() + 2 * PI - 3 * PI / 2));

                hwMap.foundationLock.setPosition(TeleopConstants.foundationLockUnlock);
                hwMap.transferLock.setPosition(TeleopConstants.transferLockPosOut);

                builder = new TrajectoryBuilder(straightDrive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);
                builder = builder.setReversed(true).lineTo(new Vector2d(straightDrive.getPoseEstimate().getX(),
                        straightDrive.getPoseEstimate().getY() + 6));
                trajectory = builder.build();   //x - 2.812, y + 7.984
                straightDrive.followTrajectorySync(trajectory);

                hwMap.foundationLock.setPosition(TeleopConstants.foundationLockLock);
                hwMap.transferLock.setPosition(TeleopConstants.transferLockPosPlatform);

                try{
                    Thread.sleep(200);
                } catch (Exception e){}

                builder = new TrajectoryBuilder(straightDrive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);
                builder = builder.setReversed(false)
                        .splineTo(new Pose2d(new Vector2d(straightDrive.getPoseEstimate().getX() - 13,
                                straightDrive.getPoseEstimate().getY() - 27.5), 7 *PI / 6));
                trajectory = builder.build();   //x - 2.812, y + 7.984
                straightDrive.followTrajectorySync(trajectory);

                hwMap.foundationLock.setPosition(TeleopConstants.foundationLockUnlock);
                hwMap.transferLock.setPosition(TeleopConstants.transferLockPosOut);

                try{
                    Thread.sleep(200);
                } catch (Exception e){}

                builder = new TrajectoryBuilder(straightDrive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);
                builder = builder
                        .setReversed(false).splineTo(new Pose2d(new Vector2d(5, -48.5), PI));
                trajectory = builder.build();   //x - 2.812, y + 7.984
                straightDrive.followTrajectorySync(trajectory);

                /*hwMap.parkingServo.setPosition(TeleopConstants.parkingServoPosLock);
                hwMap.foundationLock.setPosition(TeleopConstants.foundationLockUnlock);
                hwMap.transferLock.setPosition(TeleopConstants.transferLockPosPlatform);

                straightDrive = DriveBuilderReset(false, false, "step1, after strafe");
                if (DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {

                    builder = builder
                            .setReversed(true).lineTo(new Vector2d(wallSkyStoneX, straightDrive.getPoseEstimate().getY()));

                } else {
                    builder = builder.strafeTo(new Vector2d(straightDrive.getPoseEstimate().getX(), -30)).setReversed(true)
                            .lineTo(new Vector2d(-50, -30));
                }
                trajectory = builder.build();   //x - 2.812, y + 7.984
                straightDrive.followTrajectorySync(trajectory);
                currentPos = straightDrive.getPoseEstimate();

                prepGrab(FieldPosition.RED_QUARY);

                strafeDrive = DriveBuilderReset(true, false, "step1, after strafe");
                builder = new TrajectoryBuilder(strafeDrive.getPoseEstimate(), DriveConstantsPID.STRAFE_BASE_CONSTRAINTS);   //-34.752, -63.936
                if (DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {

                    builder = builder
                            .strafeTo(new Vector2d(strafeDrive.getPoseEstimate().getX(), yCoordMvmtPlane + strafeDistance));

                } else {
                    builder = builder.strafeTo(new Vector2d(strafeDrive.getPoseEstimate().getX(), -30)).setReversed(true)
                            .lineTo(new Vector2d(-50, -30));
                }
                trajectory = builder.build();   //x - 2.812, y + 7.984
                strafeDrive.followTrajectorySync(trajectory);

                grabStone(FieldPosition.RED_QUARY);
                transferReset();
                initIntakeClaw();

                strafeDrive = DriveBuilderReset(true, false, "step2, after grab stone ");
                if (DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {
                    builder = builder.setReversed(false).strafeTo(new Vector2d(wallSkyStoneX, yCoordMvmtPlane));  //-52, -39
                } else {
                    builder = builder.setReversed(false).strafeTo(new Vector2d(-50, -40))
                            .lineTo(new Vector2d(50, -40)).strafeTo(new Vector2d(50, -30));
                }
                trajectory = builder.build();
                strafeDrive.followTrajectorySync(trajectory);
                currentPos = strafeDrive.getPoseEstimate();

                straightDrive = DriveBuilderReset(false, false, "step3");
                if (DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {
                    builder = builder.setReversed(false)
                            .lineTo(new Vector2d(foundationX, straightDrive.getPoseEstimate().getY()));
                    //-52, -39
                } else {
                    builder = builder.setReversed(false).strafeTo(new Vector2d(-50, -40))
                            .lineTo(new Vector2d(50, -40)).strafeTo(new Vector2d(50, -30));
                }
                trajectory = builder.build();
                straightDrive.followTrajectorySync(trajectory);
                currentPos = straightDrive.getPoseEstimate();


                strafeDrive = DriveBuilderReset(true, false, "step4");
                if (DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {
                    builder = builder.strafeTo(new Vector2d(foundationX, yCoordMvmtPlane + strafeDistance));
                } else {
                    builder = builder.strafeTo(new Vector2d(50, -40)).setReversed(true).lineTo(new Vector2d(-20, -40))
                            .strafeTo(new Vector2d(-20, -28));
                }
                trajectory = builder.build();
                strafeDrive.followTrajectorySync(trajectory);
                currentPos = strafeDrive.getPoseEstimate();

                dropStone(FieldPosition.RED_QUARY);

                strafeDrive = DriveBuilderReset(true, false, "step5, after drop 1st stone");
                if (DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {
                    builder = builder.strafeTo(new Vector2d(foundationX, yCoordMvmtPlane));
                } else {
                    builder = builder.strafeTo(new Vector2d(50, -40)).setReversed(true).lineTo(new Vector2d(-20, -40))
                            .strafeTo(new Vector2d(-20, -28));
                }
                trajectory = builder.build();
                strafeDrive.followTrajectorySync(trajectory);
                currentPos = strafeDrive.getPoseEstimate();

                straightDrive = DriveBuilderReset(false, false, "step6");
                if (DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {
                    builder = builder
                            .setReversed(true).lineTo(new Vector2d(furtherMostSkyStoneX, straightDrive.getPoseEstimate().getY()));
                } else {
                    builder = builder.strafeTo(new Vector2d(50, -40)).setReversed(true).lineTo(new Vector2d(-20, -40))
                            .strafeTo(new Vector2d(-20, -28));
                }
                trajectory = builder.build();
                straightDrive.followTrajectorySync(trajectory);
                currentPos = straightDrive.getPoseEstimate();

                prepGrab(FieldPosition.RED_QUARY);

                strafeDrive = DriveBuilderReset(true, false, "step7");
                if (DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {
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

                strafeDrive = DriveBuilderReset(true, false, "step8, grabbed 2nd stone");
                if (DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {
                    builder = builder.setReversed(false).strafeTo(new Vector2d(furtherMostSkyStoneX, yCoordMvmtPlane));
                } else {
                    builder = builder.setReversed(false).strafeTo(new Vector2d(-20, -40)).lineTo(new Vector2d(50, -40))
                            .strafeTo(new Vector2d(50, -30));
                }
                trajectory = builder.build();
                strafeDrive.followTrajectorySync(trajectory);
                currentPos = strafeDrive.getPoseEstimate();

                straightDrive = DriveBuilderReset(false, false, "step9");
                if (DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {
                    builder = builder.setReversed(false)
                            .lineTo(new Vector2d(foundationX, yCoordMvmtPlane));
                } else {
                    builder = builder.setReversed(false).strafeTo(new Vector2d(-20, -40)).lineTo(new Vector2d(50, -40))
                            .strafeTo(new Vector2d(50, -30));
                }
                trajectory = builder.build();
                straightDrive.followTrajectorySync(trajectory);
                currentPos = straightDrive.getPoseEstimate();

                strafeDrive = DriveBuilderReset(true, false, "step10");
                if (DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {
                    builder = builder.setReversed(false)
                            .strafeTo(new Vector2d(foundationX, yCoordMvmtPlane + strafeDistance));
                } else {
                    builder = builder.setReversed(false).strafeTo(new Vector2d(-20, -40)).lineTo(new Vector2d(50, -40))
                            .strafeTo(new Vector2d(50, -30));
                }
                trajectory = builder.build();
                strafeDrive.followTrajectorySync(trajectory);
                currentPos = strafeDrive.getPoseEstimate();

                dropStone(FieldPosition.RED_QUARY);*/

                /*strafeDrive = DriveBuilderReset(true, false, "step11, dropped 2nd stone");
                if (DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {
                    builder = builder.strafeTo(new Vector2d(foundationX, yCoordMvmtPlane));
                }
                trajectory = builder.build();
                strafeDrive.followTrajectorySync(trajectory);
                currentPos = strafeDrive.getPoseEstimate();

                straightDrive = DriveBuilderReset(false, false, "step12");
                if (DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {
                    builder = builder
                            .setReversed(true).lineTo(new Vector2d(firstRegularStoneX, yCoordMvmtPlane));
                }
                trajectory = builder.build();
                straightDrive.followTrajectorySync(trajectory);
                currentPos = straightDrive.getPoseEstimate();

                prepGrab(FieldPosition.RED_QUARY);

                strafeDrive = DriveBuilderReset(true, false, "step13");
                if (DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {
                    builder = builder
                            .strafeTo(new Vector2d(firstRegularStoneX, yCoordMvmtPlane + strafeDistance));
                }
                trajectory = builder.build();
                strafeDrive.followTrajectorySync(trajectory);
                currentPos = strafeDrive.getPoseEstimate();

                grabStone(FieldPosition.RED_QUARY);

                strafeDrive = DriveBuilderReset(true, false, "step14, grabbed 3rd stone");
                if (DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {
                    builder = builder.setReversed(false).strafeTo(new Vector2d(firstRegularStoneX, yCoordMvmtPlane));
                }
                trajectory = builder.build();
                strafeDrive.followTrajectorySync(trajectory);
                currentPos = strafeDrive.getPoseEstimate();

                straightDrive = DriveBuilderReset(false, false, "step15");

                if (DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {
                    builder = builder.setReversed(false)
                            .lineTo(new Vector2d(foundationX, yCoordMvmtPlane));
                }
                trajectory = builder.build();
                straightDrive.followTrajectorySync(trajectory);
                currentPos = straightDrive.getPoseEstimate();

                strafeDrive = DriveBuilderReset(true, false, "step16");

                if (DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {
                    builder = builder.setReversed(false)
                            .strafeTo(new Vector2d(foundationX, yCoordMvmtPlane + strafeDistance));
                }
                trajectory = builder.build();
                strafeDrive.followTrajectorySync(trajectory);
                currentPos = strafeDrive.getPoseEstimate();

                DriveConstantsPID.keep_vuforia_running = false;

                dropStone(FieldPosition.RED_QUARY);*/

               /*hwMap.foundationLock.setPosition(TeleopConstants.foundationLockUnlock);
                hwMap.transferLock.setPosition(TeleopConstants.transferLockPosOut);

                strafeDrive = DriveBuilderReset(true, false, "step17.5");

                if (DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {
                    builder = builder.setReversed(false).strafeTo(new Vector2d(foundationX, yCoordMvmtPlane - 6));
                    //.strafeTo(new Vector2d(42, -30));
                }
                trajectory = builder.build();
                strafeDrive.followTrajectorySync(trajectory);
                currentPos = strafeDrive.getPoseEstimate();

                straightDrive = DriveBuilderReset(true, false, "step17.0, dropped 3rd stones");

                if (getIMUAngle() < Math.PI)
                    straightDrive.turnSync(-((getIMUAngle() + 2 * Math.PI) - 3 * Math.PI / 2));
                else
                    straightDrive.turnSync(-(getIMUAngle() - 3 * Math.PI / 2));
                currentPos = strafeDrive.getPoseEstimate();

                straightDrive = DriveBuilderReset(false, false, "step18");

                if (DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {
                    builder = builder.setReversed(false)
                            .setReversed(true).lineTo(new Vector2d(straightDrive.getPoseEstimate().getX(), -25));
                    //.strafeTo(new Vector2d(42, -30));
                }
                trajectory = builder.build();
                straightDrive.followTrajectorySync(trajectory);
                currentPos = straightDrive.getPoseEstimate();

                hwMap.foundationLock.setPosition(TeleopConstants.foundationLockLock);
                hwMap.transferLock.setPosition(TeleopConstants.transferLockPosUp);

                try {
                    Thread.sleep(300);
                } catch (Exception e) {
                }

                straightDrive = DriveBuilderReset(false, false, "step19");

                if (DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {
                    builder = builder.setReversed(false)//.strafeTo(new Vector2d(-24, -40))
                            .setReversed(false).lineTo(new Vector2d(straightDrive.getPoseEstimate().getX() - 10, -55));
                    //.strafeTo(new Vector2d(42, -30));
                }
                trajectory = builder.build();
                straightDrive.followTrajectorySync(trajectory);
                currentPos = straightDrive.getPoseEstimate();
                RobotLog.dd(TAG, "step20, " + currentPos.toString());

                straightDrive.turnSync(-Math.toRadians(70));
                RobotLog.dd(TAG, "step21, " + straightDrive.getPoseEstimate().toString());
                currentPos = straightDrive.getPoseEstimate();

                straightDrive = DriveBuilderReset(false, false, "step18");

                hwMap.foundationLock.setPosition(TeleopConstants.foundationLockUnlock);
                hwMap.transferLock.setPosition(TeleopConstants.transferLockPosOut);
                try {
                    Thread.sleep(300);
                } catch (Exception e) {
                }

                if (DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {
                    //builder = builder.splineTo(new Pose2d(straightDrive.getPoseEstimate().getX() - 12, straightDrive.getPoseEstimate().getY() - 24, PI / 2));
                    builder = builder.forward(30);
                    //.strafeTo(new Vector2d(42, -30));
                }
                trajectory = builder.build();
                straightDrive.followTrajectorySync(trajectory);*/
                break;
        }
    }

    public void RedFoundationPark() {
        hwMap.parkingServo.setPosition(TeleopConstants.parkingServoPosLock);
        transferReset();
        initIntakeClaw();
        try {
            Thread.sleep(5000);
        } catch (Exception e) {
        }

        intake(0);

        try {
            Thread.sleep(15000);
        } catch (Exception e) {
        }

        hwMap.foundationLock.setPosition(TeleopConstants.foundationLockUnlock);
        hwMap.transferLock.setPosition(TeleopConstants.transferLockPosOut);

        try {
            Thread.sleep(10000);
        } catch (Exception e) {
        }

        /*try {
            Thread.sleep(18000);
        } catch (Exception e) {
        }

        straightDrive.getLocalizer().setPoseEstimate(startingPos);
        straightDrive.getLocalizer().update();
        builder = new TrajectoryBuilder(straightDrive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);
        builder = builder.strafeRight(6).setReversed(false).forward(28);
        trajectory = builder.build();
        straightDrive.followTrajectorySync(trajectory);*/
    }

    public void BlueQuary(int[] skystonePositions) {    // (-x, y)
        switch (skystonePositions[0]) {
            case 1:
                double yCoordMvmtPlane = 45.5;
                double wallSkyStoneX = -48.0;
                double furtherMostSkyStoneX = -25.0;
                double firstRegularStoneX = -34.0;
                double foundationX = 47.0;
                double strafeDistance = -10.0;

                transferReset();
                initIntakeClaw();

                hwMap.parkingServo.setPosition(TeleopConstants.parkingServoPosLock);
                hwMap.foundationLock.setPosition(TeleopConstants.foundationLockUnlock);
                hwMap.transferLock.setPosition(TeleopConstants.transferLockPosPlatform);

                straightDrive = DriveBuilderReset(false, false, "step1, after strafe");
                if (DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {

                    builder = builder
                            .setReversed(true).lineTo(new Vector2d(wallSkyStoneX, straightDrive.getPoseEstimate().getY()));

                } else {
                    builder = builder.strafeTo(new Vector2d(straightDrive.getPoseEstimate().getX(), -30)).setReversed(true)
                            .lineTo(new Vector2d(-50, -30));
                }
                trajectory = builder.build();   //x - 2.812, y + 7.984
                straightDrive.followTrajectorySync(trajectory);
                currentPos = straightDrive.getPoseEstimate();

                prepGrab(FieldPosition.RED_QUARY);

                strafeDrive = DriveBuilderReset(true, false, "step1, after strafe");
                builder = new TrajectoryBuilder(strafeDrive.getPoseEstimate(), DriveConstantsPID.STRAFE_BASE_CONSTRAINTS);   //-34.752, -63.936
                if (DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {

                    builder = builder
                            .strafeTo(new Vector2d(strafeDrive.getPoseEstimate().getX(), yCoordMvmtPlane + strafeDistance));

                } else {
                    builder = builder.strafeTo(new Vector2d(strafeDrive.getPoseEstimate().getX(), -30)).setReversed(true)
                            .lineTo(new Vector2d(-50, -30));
                }
                trajectory = builder.build();   //x - 2.812, y + 7.984
                strafeDrive.followTrajectorySync(trajectory);

                grabStone(FieldPosition.RED_QUARY);

                strafeDrive = DriveBuilderReset(true, false, "step2, after grab stone ");
                if (DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {
                    builder = builder.setReversed(false).strafeTo(new Vector2d(wallSkyStoneX, yCoordMvmtPlane));  //-52, -39
                } else {
                    builder = builder.setReversed(false).strafeTo(new Vector2d(-50, -40))
                            .lineTo(new Vector2d(50, -40)).strafeTo(new Vector2d(50, -30));
                }
                trajectory = builder.build();
                strafeDrive.followTrajectorySync(trajectory);
                currentPos = strafeDrive.getPoseEstimate();

                straightDrive = DriveBuilderReset(false, false, "step3");
                if (DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {
                    builder = builder.setReversed(false)
                            .lineTo(new Vector2d(foundationX, straightDrive.getPoseEstimate().getY()));
                    //-52, -39
                } else {
                    builder = builder.setReversed(false).strafeTo(new Vector2d(-50, -40))
                            .lineTo(new Vector2d(50, -40)).strafeTo(new Vector2d(50, -30));
                }
                trajectory = builder.build();
                straightDrive.followTrajectorySync(trajectory);
                currentPos = straightDrive.getPoseEstimate();


                strafeDrive = DriveBuilderReset(true, false, "step4");
                if (DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {
                    builder = builder.strafeTo(new Vector2d(foundationX, yCoordMvmtPlane + strafeDistance));
                } else {
                    builder = builder.strafeTo(new Vector2d(50, -40)).setReversed(true).lineTo(new Vector2d(-20, -40))
                            .strafeTo(new Vector2d(-20, -28));
                }
                trajectory = builder.build();
                strafeDrive.followTrajectorySync(trajectory);
                currentPos = strafeDrive.getPoseEstimate();

                dropStone(FieldPosition.RED_QUARY);

                /*strafeDrive = DriveBuilderReset(true, false, "step5, after drop 1st stone");
                if (DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {
                    builder = builder.strafeTo(new Vector2d(foundationX, yCoordMvmtPlane));
                } else {
                    builder = builder.strafeTo(new Vector2d(50, -40)).setReversed(true).lineTo(new Vector2d(-20, -40))
                            .strafeTo(new Vector2d(-20, -28));
                }
                trajectory = builder.build();
                strafeDrive.followTrajectorySync(trajectory);
                currentPos = strafeDrive.getPoseEstimate();

                straightDrive = DriveBuilderReset(false, false, "step6");
                if (DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {
                    builder = builder
                            .setReversed(true).lineTo(new Vector2d(furtherMostSkyStoneX, straightDrive.getPoseEstimate().getY()));
                } else {
                    builder = builder.strafeTo(new Vector2d(50, -40)).setReversed(true).lineTo(new Vector2d(-20, -40))
                            .strafeTo(new Vector2d(-20, -28));
                }
                trajectory = builder.build();
                straightDrive.followTrajectorySync(trajectory);
                currentPos = straightDrive.getPoseEstimate();

                prepGrab(FieldPosition.RED_QUARY);

                strafeDrive = DriveBuilderReset(true, false, "step7");
                if (DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {
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

                strafeDrive = DriveBuilderReset(true, false, "step8, grabbed 2nd stone");
                if (DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {
                    builder = builder.setReversed(false).strafeTo(new Vector2d(furtherMostSkyStoneX, yCoordMvmtPlane));
                } else {
                    builder = builder.setReversed(false).strafeTo(new Vector2d(-20, -40)).lineTo(new Vector2d(50, -40))
                            .strafeTo(new Vector2d(50, -30));
                }
                trajectory = builder.build();
                strafeDrive.followTrajectorySync(trajectory);
                currentPos = strafeDrive.getPoseEstimate();

                straightDrive = DriveBuilderReset(false, false, "step9");
                if (DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {
                    builder = builder.setReversed(false)
                            .lineTo(new Vector2d(foundationX, yCoordMvmtPlane));
                } else {
                    builder = builder.setReversed(false).strafeTo(new Vector2d(-20, -40)).lineTo(new Vector2d(50, -40))
                            .strafeTo(new Vector2d(50, -30));
                }
                trajectory = builder.build();
                straightDrive.followTrajectorySync(trajectory);
                currentPos = straightDrive.getPoseEstimate();

                strafeDrive = DriveBuilderReset(true, false, "step10");
                if (DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {
                    builder = builder.setReversed(false)
                            .strafeTo(new Vector2d(foundationX, yCoordMvmtPlane + strafeDistance));
                } else {
                    builder = builder.setReversed(false).strafeTo(new Vector2d(-20, -40)).lineTo(new Vector2d(50, -40))
                            .strafeTo(new Vector2d(50, -30));
                }
                trajectory = builder.build();
                strafeDrive.followTrajectorySync(trajectory);
                currentPos = strafeDrive.getPoseEstimate();

                dropStone(FieldPosition.RED_QUARY);*/

                /*strafeDrive = DriveBuilderReset(true, false, "step11, dropped 2nd stone");
                if (DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {
                    builder = builder.strafeTo(new Vector2d(foundationX, yCoordMvmtPlane));
                }
                trajectory = builder.build();
                strafeDrive.followTrajectorySync(trajectory);
                currentPos = strafeDrive.getPoseEstimate();

                straightDrive = DriveBuilderReset(false, false, "step12");
                if (DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {
                    builder = builder
                            .setReversed(true).lineTo(new Vector2d(firstRegularStoneX, yCoordMvmtPlane));
                }
                trajectory = builder.build();
                straightDrive.followTrajectorySync(trajectory);
                currentPos = straightDrive.getPoseEstimate();

                prepGrab(FieldPosition.RED_QUARY);

                strafeDrive = DriveBuilderReset(true, false, "step13");
                if (DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {
                    builder = builder
                            .strafeTo(new Vector2d(firstRegularStoneX, yCoordMvmtPlane + strafeDistance));
                }
                trajectory = builder.build();
                strafeDrive.followTrajectorySync(trajectory);
                currentPos = strafeDrive.getPoseEstimate();

                grabStone(FieldPosition.RED_QUARY);

                strafeDrive = DriveBuilderReset(true, false, "step14, grabbed 3rd stone");
                if (DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {
                    builder = builder.setReversed(false).strafeTo(new Vector2d(firstRegularStoneX, yCoordMvmtPlane));
                }
                trajectory = builder.build();
                strafeDrive.followTrajectorySync(trajectory);
                currentPos = strafeDrive.getPoseEstimate();

                straightDrive = DriveBuilderReset(false, false, "step15");

                if (DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {
                    builder = builder.setReversed(false)
                            .lineTo(new Vector2d(foundationX, yCoordMvmtPlane));
                }
                trajectory = builder.build();
                straightDrive.followTrajectorySync(trajectory);
                currentPos = straightDrive.getPoseEstimate();

                strafeDrive = DriveBuilderReset(true, false, "step16");

                if (DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {
                    builder = builder.setReversed(false)
                            .strafeTo(new Vector2d(foundationX, yCoordMvmtPlane + strafeDistance));
                }
                trajectory = builder.build();
                strafeDrive.followTrajectorySync(trajectory);
                currentPos = strafeDrive.getPoseEstimate();

                DriveConstantsPID.keep_vuforia_running = false;

                dropStone(FieldPosition.RED_QUARY);*/

                hwMap.foundationLock.setPosition(TeleopConstants.foundationLockUnlock);
                hwMap.transferLock.setPosition(TeleopConstants.transferLockPosOut);

                strafeDrive = DriveBuilderReset(true, false, "step17.5");

                if (DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {
                    builder = builder.setReversed(false).strafeTo(new Vector2d(foundationX, yCoordMvmtPlane - 2));
                    //.strafeTo(new Vector2d(42, -30));
                }
                trajectory = builder.build();
                strafeDrive.followTrajectorySync(trajectory);
                currentPos = strafeDrive.getPoseEstimate();

                straightDrive = DriveBuilderReset(true, false, "step17.0, dropped 3rd stones");

                if (getIMUAngle() < Math.PI)
                    straightDrive.turnSync(Math.PI / 2 - getIMUAngle());
                else
                    straightDrive.turnSync(Math.PI / 2 - (getIMUAngle() - 2 * PI));
                currentPos = strafeDrive.getPoseEstimate();

                straightDrive = DriveBuilderReset(false, false, "step18");

                if (DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {
                    builder = builder.setReversed(false)
                            .setReversed(true).lineTo(new Vector2d(straightDrive.getPoseEstimate().getX(), 30));
                    //.strafeTo(new Vector2d(42, -30));
                }
                trajectory = builder.build();
                straightDrive.followTrajectorySync(trajectory);
                currentPos = straightDrive.getPoseEstimate();

                hwMap.foundationLock.setPosition(TeleopConstants.foundationLockLock);
                hwMap.transferLock.setPosition(TeleopConstants.transferLockPosUp);

                try {
                    Thread.sleep(300);
                } catch (Exception e) {
                }

                straightDrive = DriveBuilderReset(false, false, "step19");

                if (DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {
                    builder = builder.setReversed(false)//.strafeTo(new Vector2d(-24, -40))
                            .setReversed(false).lineTo(new Vector2d(straightDrive.getPoseEstimate().getX() - 10, 55));
                    //.strafeTo(new Vector2d(42, -30));
                }
                trajectory = builder.build();
                straightDrive.followTrajectorySync(trajectory);
                currentPos = straightDrive.getPoseEstimate();
                RobotLog.dd(TAG, "step20, " + currentPos.toString());

                straightDrive.turnSync(Math.toRadians(70));
                RobotLog.dd(TAG, "step21, " + straightDrive.getPoseEstimate().toString());
                currentPos = straightDrive.getPoseEstimate();

                hwMap.foundationLock.setPosition(TeleopConstants.foundationLockUnlock);
                hwMap.transferLock.setPosition(TeleopConstants.transferLockPosOut);
                try {
                    Thread.sleep(300);
                } catch (Exception e) {
                }

                straightDrive = DriveBuilderReset(false, false, "step18");

                if (DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {
                    //builder = builder.splineTo(new Pose2d(straightDrive.getPoseEstimate().getX() - 12, straightDrive.getPoseEstimate().getY() - 24, PI / 2));
                    builder = builder.forward(30);
                    //.strafeTo(new Vector2d(42, -30));
                }
                trajectory = builder.build();
                straightDrive.followTrajectorySync(trajectory);
                break;
            case 2:
                yCoordMvmtPlane = 45.5;
                wallSkyStoneX = -56.5;
                furtherMostSkyStoneX = -30.5;
                firstRegularStoneX = -34.0;
                foundationX = 47.0;
                strafeDistance = -10.0;

                hwMap.parkingServo.setPosition(TeleopConstants.parkingServoPosLock);
                hwMap.foundationLock.setPosition(TeleopConstants.foundationLockUnlock);
                hwMap.transferLock.setPosition(TeleopConstants.transferLockPosPlatform);

                straightDrive = DriveBuilderReset(false, false, "step1, after strafe");
                if (DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {

                    builder = builder
                            .setReversed(true).lineTo(new Vector2d(wallSkyStoneX, straightDrive.getPoseEstimate().getY()));

                } else {
                    builder = builder.strafeTo(new Vector2d(straightDrive.getPoseEstimate().getX(), -30)).setReversed(true)
                            .lineTo(new Vector2d(-50, -30));
                }
                trajectory = builder.build();   //x - 2.812, y + 7.984
                straightDrive.followTrajectorySync(trajectory);
                currentPos = straightDrive.getPoseEstimate();

                prepGrab(FieldPosition.RED_QUARY);

                strafeDrive = DriveBuilderReset(true, false, "step1, after strafe");
                builder = new TrajectoryBuilder(strafeDrive.getPoseEstimate(), DriveConstantsPID.STRAFE_BASE_CONSTRAINTS);   //-34.752, -63.936
                if (DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {

                    builder = builder
                            .strafeTo(new Vector2d(strafeDrive.getPoseEstimate().getX(), yCoordMvmtPlane + strafeDistance));

                } else {
                    builder = builder.strafeTo(new Vector2d(strafeDrive.getPoseEstimate().getX(), -30)).setReversed(true)
                            .lineTo(new Vector2d(-50, -30));
                }
                trajectory = builder.build();   //x - 2.812, y + 7.984
                strafeDrive.followTrajectorySync(trajectory);

                grabStone(FieldPosition.RED_QUARY);
                transferReset();
                initIntakeClaw();

                strafeDrive = DriveBuilderReset(true, false, "step2, after grab stone ");
                if (DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {
                    builder = builder.setReversed(false).strafeTo(new Vector2d(wallSkyStoneX, yCoordMvmtPlane));  //-52, -39
                } else {
                    builder = builder.setReversed(false).strafeTo(new Vector2d(-50, -40))
                            .lineTo(new Vector2d(50, -40)).strafeTo(new Vector2d(50, -30));
                }
                trajectory = builder.build();
                strafeDrive.followTrajectorySync(trajectory);
                currentPos = strafeDrive.getPoseEstimate();

                straightDrive = DriveBuilderReset(false, false, "step3");
                if (DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {
                    builder = builder.setReversed(false)
                            .lineTo(new Vector2d(foundationX, straightDrive.getPoseEstimate().getY()));
                    //-52, -39
                } else {
                    builder = builder.setReversed(false).strafeTo(new Vector2d(-50, -40))
                            .lineTo(new Vector2d(50, -40)).strafeTo(new Vector2d(50, -30));
                }
                trajectory = builder.build();
                straightDrive.followTrajectorySync(trajectory);
                currentPos = straightDrive.getPoseEstimate();


                strafeDrive = DriveBuilderReset(true, false, "step4");
                if (DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {
                    builder = builder.strafeTo(new Vector2d(foundationX, yCoordMvmtPlane + strafeDistance));
                } else {
                    builder = builder.strafeTo(new Vector2d(50, -40)).setReversed(true).lineTo(new Vector2d(-20, -40))
                            .strafeTo(new Vector2d(-20, -28));
                }
                trajectory = builder.build();
                strafeDrive.followTrajectorySync(trajectory);
                currentPos = strafeDrive.getPoseEstimate();

                dropStone(FieldPosition.RED_QUARY);

                /*strafeDrive = DriveBuilderReset(true, false, "step5, after drop 1st stone");
                if (DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {
                    builder = builder.strafeTo(new Vector2d(foundationX, yCoordMvmtPlane));
                } else {
                    builder = builder.strafeTo(new Vector2d(50, -40)).setReversed(true).lineTo(new Vector2d(-20, -40))
                            .strafeTo(new Vector2d(-20, -28));
                }
                trajectory = builder.build();
                strafeDrive.followTrajectorySync(trajectory);
                currentPos = strafeDrive.getPoseEstimate();

                straightDrive = DriveBuilderReset(false, false, "step6");
                if (DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {
                    builder = builder
                            .setReversed(true).lineTo(new Vector2d(furtherMostSkyStoneX, straightDrive.getPoseEstimate().getY()));
                } else {
                    builder = builder.strafeTo(new Vector2d(50, -40)).setReversed(true).lineTo(new Vector2d(-20, -40))
                            .strafeTo(new Vector2d(-20, -28));
                }
                trajectory = builder.build();
                straightDrive.followTrajectorySync(trajectory);
                currentPos = straightDrive.getPoseEstimate();

                prepGrab(FieldPosition.RED_QUARY);

                strafeDrive = DriveBuilderReset(true, false, "step7");
                if (DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {
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

                strafeDrive = DriveBuilderReset(true, false, "step8, grabbed 2nd stone");
                if (DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {
                    builder = builder.setReversed(false).strafeTo(new Vector2d(furtherMostSkyStoneX, yCoordMvmtPlane));
                } else {
                    builder = builder.setReversed(false).strafeTo(new Vector2d(-20, -40)).lineTo(new Vector2d(50, -40))
                            .strafeTo(new Vector2d(50, -30));
                }
                trajectory = builder.build();
                strafeDrive.followTrajectorySync(trajectory);
                currentPos = strafeDrive.getPoseEstimate();

                straightDrive = DriveBuilderReset(false, false, "step9");
                if (DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {
                    builder = builder.setReversed(false)
                            .lineTo(new Vector2d(foundationX, yCoordMvmtPlane));
                } else {
                    builder = builder.setReversed(false).strafeTo(new Vector2d(-20, -40)).lineTo(new Vector2d(50, -40))
                            .strafeTo(new Vector2d(50, -30));
                }
                trajectory = builder.build();
                straightDrive.followTrajectorySync(trajectory);
                currentPos = straightDrive.getPoseEstimate();

                strafeDrive = DriveBuilderReset(true, false, "step10");
                if (DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {
                    builder = builder.setReversed(false)
                            .strafeTo(new Vector2d(foundationX, yCoordMvmtPlane + strafeDistance));
                } else {
                    builder = builder.setReversed(false).strafeTo(new Vector2d(-20, -40)).lineTo(new Vector2d(50, -40))
                            .strafeTo(new Vector2d(50, -30));
                }
                trajectory = builder.build();
                strafeDrive.followTrajectorySync(trajectory);
                currentPos = strafeDrive.getPoseEstimate();

                dropStone(FieldPosition.RED_QUARY);*/

                /*strafeDrive = DriveBuilderReset(true, false, "step11, dropped 2nd stone");
                if (DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {
                    builder = builder.strafeTo(new Vector2d(foundationX, yCoordMvmtPlane));
                }
                trajectory = builder.build();
                strafeDrive.followTrajectorySync(trajectory);
                currentPos = strafeDrive.getPoseEstimate();

                straightDrive = DriveBuilderReset(false, false, "step12");
                if (DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {
                    builder = builder
                            .setReversed(true).lineTo(new Vector2d(firstRegularStoneX, yCoordMvmtPlane));
                }
                trajectory = builder.build();
                straightDrive.followTrajectorySync(trajectory);
                currentPos = straightDrive.getPoseEstimate();

                prepGrab(FieldPosition.RED_QUARY);

                strafeDrive = DriveBuilderReset(true, false, "step13");
                if (DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {
                    builder = builder
                            .strafeTo(new Vector2d(firstRegularStoneX, yCoordMvmtPlane + strafeDistance));
                }
                trajectory = builder.build();
                strafeDrive.followTrajectorySync(trajectory);
                currentPos = strafeDrive.getPoseEstimate();

                grabStone(FieldPosition.RED_QUARY);

                strafeDrive = DriveBuilderReset(true, false, "step14, grabbed 3rd stone");
                if (DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {
                    builder = builder.setReversed(false).strafeTo(new Vector2d(firstRegularStoneX, yCoordMvmtPlane));
                }
                trajectory = builder.build();
                strafeDrive.followTrajectorySync(trajectory);
                currentPos = strafeDrive.getPoseEstimate();

                straightDrive = DriveBuilderReset(false, false, "step15");

                if (DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {
                    builder = builder.setReversed(false)
                            .lineTo(new Vector2d(foundationX, yCoordMvmtPlane));
                }
                trajectory = builder.build();
                straightDrive.followTrajectorySync(trajectory);
                currentPos = straightDrive.getPoseEstimate();

                strafeDrive = DriveBuilderReset(true, false, "step16");

                if (DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {
                    builder = builder.setReversed(false)
                            .strafeTo(new Vector2d(foundationX, yCoordMvmtPlane + strafeDistance));
                }
                trajectory = builder.build();
                strafeDrive.followTrajectorySync(trajectory);
                currentPos = strafeDrive.getPoseEstimate();

                DriveConstantsPID.keep_vuforia_running = false;

                dropStone(FieldPosition.RED_QUARY);*/

                hwMap.foundationLock.setPosition(TeleopConstants.foundationLockUnlock);
                hwMap.transferLock.setPosition(TeleopConstants.transferLockPosOut);

                strafeDrive = DriveBuilderReset(true, false, "step17.5");

                if (DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {
                    builder = builder.setReversed(false).strafeTo(new Vector2d(foundationX, yCoordMvmtPlane - 2));
                    //.strafeTo(new Vector2d(42, -30));
                }
                trajectory = builder.build();
                strafeDrive.followTrajectorySync(trajectory);
                currentPos = strafeDrive.getPoseEstimate();

                straightDrive = DriveBuilderReset(true, false, "step17.0, dropped 3rd stones");

                if (getIMUAngle() < Math.PI)
                    straightDrive.turnSync(Math.PI / 2 - getIMUAngle());
                else
                    straightDrive.turnSync(Math.PI / 2 - (getIMUAngle() - 2 * PI));
                currentPos = strafeDrive.getPoseEstimate();

                straightDrive = DriveBuilderReset(false, false, "step18");

                if (DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {
                    builder = builder.setReversed(false)
                            .setReversed(true).lineTo(new Vector2d(straightDrive.getPoseEstimate().getX(), 30));
                    //.strafeTo(new Vector2d(42, -30));
                }
                trajectory = builder.build();
                straightDrive.followTrajectorySync(trajectory);
                currentPos = straightDrive.getPoseEstimate();

                hwMap.foundationLock.setPosition(TeleopConstants.foundationLockLock);
                hwMap.transferLock.setPosition(TeleopConstants.transferLockPosUp);

                try {
                    Thread.sleep(300);
                } catch (Exception e) {
                }

                straightDrive = DriveBuilderReset(false, false, "step19");

                if (DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {
                    builder = builder.setReversed(false)//.strafeTo(new Vector2d(-24, -40))
                            .setReversed(false).lineTo(new Vector2d(straightDrive.getPoseEstimate().getX() - 10, 57));
                    //.strafeTo(new Vector2d(42, -30));
                }
                trajectory = builder.build();
                straightDrive.followTrajectorySync(trajectory);
                currentPos = straightDrive.getPoseEstimate();
                RobotLog.dd(TAG, "step20, " + currentPos.toString());

                straightDrive.turnSync(Math.toRadians(70));
                RobotLog.dd(TAG, "step21, " + straightDrive.getPoseEstimate().toString());
                currentPos = straightDrive.getPoseEstimate();

                hwMap.foundationLock.setPosition(TeleopConstants.foundationLockUnlock);
                hwMap.transferLock.setPosition(TeleopConstants.transferLockPosOut);
                try {
                    Thread.sleep(300);
                } catch (Exception e) {
                }

                straightDrive = DriveBuilderReset(false, false, "step18");

                if (DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {
                    //builder = builder.splineTo(new Pose2d(straightDrive.getPoseEstimate().getX() - 12, straightDrive.getPoseEstimate().getY() - 24, PI / 2));
                    builder = builder.forward(30);
                    //.strafeTo(new Vector2d(42, -30));
                }
                trajectory = builder.build();
                straightDrive.followTrajectorySync(trajectory);
                break;
            case 3:
                yCoordMvmtPlane = 45.5;
                wallSkyStoneX = -63.5;
                furtherMostSkyStoneX = -40.0;
                firstRegularStoneX = -36.0;
                foundationX = 47.0;
                strafeDistance = -10.0;

                hwMap.parkingServo.setPosition(TeleopConstants.parkingServoPosLock);
                hwMap.foundationLock.setPosition(TeleopConstants.foundationLockUnlock);
                hwMap.transferLock.setPosition(TeleopConstants.transferLockPosPlatform);

                straightDrive = DriveBuilderReset(false, false, "step1, after strafe");
                if (DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {

                    builder = builder
                            .setReversed(true).lineTo(new Vector2d(wallSkyStoneX, straightDrive.getPoseEstimate().getY()));

                } else {
                    builder = builder.strafeTo(new Vector2d(straightDrive.getPoseEstimate().getX(), -30)).setReversed(true)
                            .lineTo(new Vector2d(-50, -30));
                }
                trajectory = builder.build();   //x - 2.812, y + 7.984
                straightDrive.followTrajectorySync(trajectory);
                currentPos = straightDrive.getPoseEstimate();

                prepGrab(FieldPosition.RED_QUARY);

                strafeDrive = DriveBuilderReset(true, false, "step1, after strafe");
                builder = new TrajectoryBuilder(strafeDrive.getPoseEstimate(), DriveConstantsPID.STRAFE_BASE_CONSTRAINTS);   //-34.752, -63.936
                if (DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {

                    builder = builder
                            .strafeTo(new Vector2d(strafeDrive.getPoseEstimate().getX(), yCoordMvmtPlane + strafeDistance));

                } else {
                    builder = builder.strafeTo(new Vector2d(strafeDrive.getPoseEstimate().getX(), -30)).setReversed(true)
                            .lineTo(new Vector2d(-50, -30));
                }
                trajectory = builder.build();   //x - 2.812, y + 7.984
                strafeDrive.followTrajectorySync(trajectory);

                grabStone(FieldPosition.RED_QUARY);
                transferReset();
                initIntakeClaw();

                strafeDrive = DriveBuilderReset(true, false, "step2, after grab stone ");
                if (DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {
                    builder = builder.setReversed(false).strafeTo(new Vector2d(wallSkyStoneX, yCoordMvmtPlane));  //-52, -39
                } else {
                    builder = builder.setReversed(false).strafeTo(new Vector2d(-50, -40))
                            .lineTo(new Vector2d(50, -40)).strafeTo(new Vector2d(50, -30));
                }
                trajectory = builder.build();
                strafeDrive.followTrajectorySync(trajectory);
                currentPos = strafeDrive.getPoseEstimate();

                straightDrive = DriveBuilderReset(false, false, "step3");
                if (DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {
                    builder = builder.setReversed(false)
                            .lineTo(new Vector2d(foundationX, straightDrive.getPoseEstimate().getY()));
                    //-52, -39
                } else {
                    builder = builder.setReversed(false).strafeTo(new Vector2d(-50, -40))
                            .lineTo(new Vector2d(50, -40)).strafeTo(new Vector2d(50, -30));
                }
                trajectory = builder.build();
                straightDrive.followTrajectorySync(trajectory);
                currentPos = straightDrive.getPoseEstimate();


                strafeDrive = DriveBuilderReset(true, false, "step4");
                if (DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {
                    builder = builder.strafeTo(new Vector2d(foundationX, yCoordMvmtPlane + strafeDistance));
                } else {
                    builder = builder.strafeTo(new Vector2d(50, -40)).setReversed(true).lineTo(new Vector2d(-20, -40))
                            .strafeTo(new Vector2d(-20, -28));
                }
                trajectory = builder.build();
                strafeDrive.followTrajectorySync(trajectory);
                currentPos = strafeDrive.getPoseEstimate();

                dropStone(FieldPosition.RED_QUARY);

                /*strafeDrive = DriveBuilderReset(true, false, "step5, after drop 1st stone");
                if (DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {
                    builder = builder.strafeTo(new Vector2d(foundationX, yCoordMvmtPlane));
                } else {
                    builder = builder.strafeTo(new Vector2d(50, -40)).setReversed(true).lineTo(new Vector2d(-20, -40))
                            .strafeTo(new Vector2d(-20, -28));
                }
                trajectory = builder.build();
                strafeDrive.followTrajectorySync(trajectory);
                currentPos = strafeDrive.getPoseEstimate();

                straightDrive = DriveBuilderReset(false, false, "step6");
                if (DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {
                    builder = builder
                            .setReversed(true).lineTo(new Vector2d(furtherMostSkyStoneX, straightDrive.getPoseEstimate().getY()));
                } else {
                    builder = builder.strafeTo(new Vector2d(50, -40)).setReversed(true).lineTo(new Vector2d(-20, -40))
                            .strafeTo(new Vector2d(-20, -28));
                }
                trajectory = builder.build();
                straightDrive.followTrajectorySync(trajectory);
                currentPos = straightDrive.getPoseEstimate();

                prepGrab(FieldPosition.RED_QUARY);

                strafeDrive = DriveBuilderReset(true, false, "step7");
                if (DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {
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

                strafeDrive = DriveBuilderReset(true, false, "step8, grabbed 2nd stone");
                if (DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {
                    builder = builder.setReversed(false).strafeTo(new Vector2d(furtherMostSkyStoneX, yCoordMvmtPlane));
                } else {
                    builder = builder.setReversed(false).strafeTo(new Vector2d(-20, -40)).lineTo(new Vector2d(50, -40))
                            .strafeTo(new Vector2d(50, -30));
                }
                trajectory = builder.build();
                strafeDrive.followTrajectorySync(trajectory);
                currentPos = strafeDrive.getPoseEstimate();

                straightDrive = DriveBuilderReset(false, false, "step9");
                if (DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {
                    builder = builder.setReversed(false)
                            .lineTo(new Vector2d(foundationX, yCoordMvmtPlane));
                } else {
                    builder = builder.setReversed(false).strafeTo(new Vector2d(-20, -40)).lineTo(new Vector2d(50, -40))
                            .strafeTo(new Vector2d(50, -30));
                }
                trajectory = builder.build();
                straightDrive.followTrajectorySync(trajectory);
                currentPos = straightDrive.getPoseEstimate();

                strafeDrive = DriveBuilderReset(true, false, "step10");
                if (DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {
                    builder = builder.setReversed(false)
                            .strafeTo(new Vector2d(foundationX, yCoordMvmtPlane + strafeDistance));
                } else {
                    builder = builder.setReversed(false).strafeTo(new Vector2d(-20, -40)).lineTo(new Vector2d(50, -40))
                            .strafeTo(new Vector2d(50, -30));
                }
                trajectory = builder.build();
                strafeDrive.followTrajectorySync(trajectory);
                currentPos = strafeDrive.getPoseEstimate();

                dropStone(FieldPosition.RED_QUARY);*/

                /*strafeDrive = DriveBuilderReset(true, false, "step11, dropped 2nd stone");
                if (DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {
                    builder = builder.strafeTo(new Vector2d(foundationX, yCoordMvmtPlane));
                }
                trajectory = builder.build();
                strafeDrive.followTrajectorySync(trajectory);
                currentPos = strafeDrive.getPoseEstimate();

                straightDrive = DriveBuilderReset(false, false, "step12");
                if (DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {
                    builder = builder
                            .setReversed(true).lineTo(new Vector2d(firstRegularStoneX, yCoordMvmtPlane));
                }
                trajectory = builder.build();
                straightDrive.followTrajectorySync(trajectory);
                currentPos = straightDrive.getPoseEstimate();

                prepGrab(FieldPosition.RED_QUARY);

                strafeDrive = DriveBuilderReset(true, false, "step13");
                if (DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {
                    builder = builder
                            .strafeTo(new Vector2d(firstRegularStoneX, yCoordMvmtPlane + strafeDistance));
                }
                trajectory = builder.build();
                strafeDrive.followTrajectorySync(trajectory);
                currentPos = strafeDrive.getPoseEstimate();

                grabStone(FieldPosition.RED_QUARY);

                strafeDrive = DriveBuilderReset(true, false, "step14, grabbed 3rd stone");
                if (DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {
                    builder = builder.setReversed(false).strafeTo(new Vector2d(firstRegularStoneX, yCoordMvmtPlane));
                }
                trajectory = builder.build();
                strafeDrive.followTrajectorySync(trajectory);
                currentPos = strafeDrive.getPoseEstimate();

                straightDrive = DriveBuilderReset(false, false, "step15");

                if (DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {
                    builder = builder.setReversed(false)
                            .lineTo(new Vector2d(foundationX, yCoordMvmtPlane));
                }
                trajectory = builder.build();
                straightDrive.followTrajectorySync(trajectory);
                currentPos = straightDrive.getPoseEstimate();

                strafeDrive = DriveBuilderReset(true, false, "step16");

                if (DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {
                    builder = builder.setReversed(false)
                            .strafeTo(new Vector2d(foundationX, yCoordMvmtPlane + strafeDistance));
                }
                trajectory = builder.build();
                strafeDrive.followTrajectorySync(trajectory);
                currentPos = strafeDrive.getPoseEstimate();

                DriveConstantsPID.keep_vuforia_running = false;

                dropStone(FieldPosition.RED_QUARY);*/

                hwMap.foundationLock.setPosition(TeleopConstants.foundationLockUnlock);
                hwMap.transferLock.setPosition(TeleopConstants.transferLockPosOut);

                strafeDrive = DriveBuilderReset(true, false, "step17.5");

                if (DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {
                    builder = builder.setReversed(false).strafeTo(new Vector2d(foundationX, yCoordMvmtPlane - 2));
                    //.strafeTo(new Vector2d(42, -30));
                }
                trajectory = builder.build();
                strafeDrive.followTrajectorySync(trajectory);
                currentPos = strafeDrive.getPoseEstimate();

                straightDrive = DriveBuilderReset(true, false, "step17.0, dropped 3rd stones");

                if (getIMUAngle() < Math.PI)
                    straightDrive.turnSync(Math.PI / 2 - getIMUAngle());
                else
                    straightDrive.turnSync(Math.PI / 2 - (getIMUAngle() - 2 * PI));
                currentPos = strafeDrive.getPoseEstimate();

                straightDrive = DriveBuilderReset(false, false, "step18");

                if (DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {
                    builder = builder.setReversed(false)
                            .setReversed(true).lineTo(new Vector2d(straightDrive.getPoseEstimate().getX(), 28));
                    //.strafeTo(new Vector2d(42, -30));
                }
                trajectory = builder.build();
                straightDrive.followTrajectorySync(trajectory);
                currentPos = straightDrive.getPoseEstimate();

                hwMap.foundationLock.setPosition(TeleopConstants.foundationLockLock);
                hwMap.transferLock.setPosition(TeleopConstants.transferLockPosUp);

                try {
                    Thread.sleep(300);
                } catch (Exception e) {
                }

                straightDrive = DriveBuilderReset(false, false, "step19");

                if (DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {
                    builder = builder.setReversed(false)//.strafeTo(new Vector2d(-24, -40))
                            .setReversed(false).lineTo(new Vector2d(straightDrive.getPoseEstimate().getX() - 10, 57));
                    //.strafeTo(new Vector2d(42, -30));
                }
                trajectory = builder.build();
                straightDrive.followTrajectorySync(trajectory);
                currentPos = straightDrive.getPoseEstimate();
                RobotLog.dd(TAG, "step20, " + currentPos.toString());

                straightDrive.turnSync(Math.toRadians(70));
                RobotLog.dd(TAG, "step21, " + straightDrive.getPoseEstimate().toString());
                currentPos = straightDrive.getPoseEstimate();

                hwMap.foundationLock.setPosition(TeleopConstants.foundationLockUnlock);
                hwMap.transferLock.setPosition(TeleopConstants.transferLockPosOut);
                try {
                    Thread.sleep(300);
                } catch (Exception e) {
                }

                straightDrive = DriveBuilderReset(false, false, "step18");

                if (DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {
                    //builder = builder.splineTo(new Pose2d(straightDrive.getPoseEstimate().getX() - 12, straightDrive.getPoseEstimate().getY() - 24, PI / 2));
                    builder = builder.forward(30);
                    //.strafeTo(new Vector2d(42, -30));
                }
                trajectory = builder.build();
                straightDrive.followTrajectorySync(trajectory);
                break;
        }
    }

    public void BlueFoundationPark() {
        hwMap.parkingServo.setPosition(TeleopConstants.parkingServoPosLock);
        transferReset();
        initIntakeClaw();
        try {
            Thread.sleep(5000);
        } catch (Exception e) {
        }

        intake(0);

        try {
            Thread.sleep(15000);
        } catch (Exception e) {
        }

        hwMap.foundationLock.setPosition(TeleopConstants.foundationLockUnlock);
        hwMap.transferLock.setPosition(TeleopConstants.transferLockPosOut);

        try {
            Thread.sleep(10000);
        } catch (Exception e) {
        }
        /*try {
            Thread.sleep(18000);
        } catch (Exception e) {
        }

        straightDrive.getLocalizer().setPoseEstimate(startingPos);
        straightDrive.getLocalizer().update();
        builder = new TrajectoryBuilder(straightDrive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);
        builder = builder.strafeLeft(6).setReversed(false).forward(28);
        trajectory = builder.build();
        straightDrive.followTrajectorySync(trajectory);

        intake(0);*/
    }

    public void BlueFoundationDrag() {
        transferReset();
        initIntakeClaw();
        startingPos = new Pose2d(new Vector2d(20.736, 63.936), Math.toRadians(270));

        try {
            Thread.sleep(5000);
        } catch (Exception e) {
        }
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
    private void openPlatform() {
        Thread thread = new Thread() {
            public void run() {
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
                    Thread.sleep(700);
                } catch (Exception e) {
                }
                hwMap.intakeInit.setPosition(TeleopConstants.intakeInitPosRight);
                //intake(1);

                try {
                    Thread.sleep(700);
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
                try {
                    Thread.sleep(1800);
                } catch (Exception e) {
                }

                hwMap.liftOne.setPower(0);
                hwMap.liftTwo.setPower(0);
            }
        };
        thread.start();
    }

    private void prepGrab(FieldPosition fieldPosition) {
        hwMap.redAutoClawJoint2.setPosition(TeleopConstants.autoClaw2Open);
        try {
            Thread.sleep(100);
        } catch (Exception e) {
        }
        hwMap.redAutoClawJoint1.setPosition(TeleopConstants.autoClaw1Strafe);
        try {
            Thread.sleep(100);
        } catch (Exception e) {
        }
    }

    private void grabStone(FieldPosition fieldPosition) {
        hwMap.redAutoClawJoint1.setPosition(TeleopConstants.autoClaw1Down);
        try {
            Thread.sleep(100);
        } catch (Exception e) {
        }
        hwMap.redAutoClawJoint2.setPosition(TeleopConstants.autoClaw2Close);
        try {
            Thread.sleep(100);
        } catch (Exception e) {
        }
        hwMap.redAutoClawJoint1.setPosition(TeleopConstants.autoClaw1Up);
    }

    private void dropStone(FieldPosition fieldPosition) {
        hwMap.redAutoClawJoint1.setPosition(TeleopConstants.autoClaw1Down);
        try {
            Thread.sleep(100);
        } catch (Exception e) {
        }
        hwMap.redAutoClawJoint2.setPosition(TeleopConstants.autoClaw2Open);
        try {
            Thread.sleep(100);
        } catch (Exception e) {
        }
        hwMap.redAutoClawJoint1.setPosition(TeleopConstants.autoClaw1Up);
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

        try {
            Thread.sleep(1200);
        } catch (Exception e) {
        }

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

    private void driveTime(double speed, int milisecs) {
        hwMap.frontRight.setPower(speed);
        hwMap.frontLeft.setPower(speed);
        hwMap.backRight.setPower(speed);
        hwMap.backLeft.setPower(speed);

        try {
            Thread.sleep(milisecs);
        } catch (Exception e) {
        }

        hwMap.frontRight.setPower(0);
        hwMap.frontLeft.setPower(0);
        hwMap.backRight.setPower(0);
        hwMap.backLeft.setPower(0);
    }

    private void updatePoseFromStrafe(double yOdoInitPos, boolean left) {
        double strafeDistance = Math.abs(hwMap.rightIntake.getCurrentPosition() - yOdoInitPos) /
                DriveConstantsPID.odoEncoderTicksPerRev * StandardTrackingWheelLocalizer.GEAR_RATIO *
                2 * Math.PI * StandardTrackingWheelLocalizer.WHEEL_RADIUS;

        RobotLog.dd("Current Position (X, Y, Heading)", straightDrive.getPoseEstimate().toString());
        RobotLog.dd("Strafe Distance", String.valueOf(strafeDistance));
        RobotLog.dd("Direction", left ? "Left" : "Right");
        if (!left)
            strafeDistance = -strafeDistance;

        double heading = straightDrive.getExternalHeading() < 0 ? Math.PI * 2 - straightDrive.getExternalHeading() : straightDrive.getExternalHeading();

        straightDrive.getLocalizer().setPoseEstimate(new Pose2d(new Vector2d(straightDrive.getPoseEstimate().getX(),
                straightDrive.getPoseEstimate().getY() + strafeDistance), heading));
        straightDrive.getLocalizer().update();
        RobotLog.dd("Updated Position (X, Y, Heading)", straightDrive.getPoseEstimate().toString());
    }

    public void odometryStrafe(double power, double inches, boolean right) {
        double counts = inches / (2 * PI * 1.25) * 1550.0;
        int sidewaysStart = hwMap.rightIntake.getCurrentPosition();

        if (!right)
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

    private double getIMUAngle() {
        return imu.getAngularOrientation().firstAngle >= 0 ? imu.getAngularOrientation().firstAngle :
                imu.getAngularOrientation().firstAngle + PI * 2;
    }

    private Function0 prepare(){
        return new Function0() {
            @Override
            public Object invoke() {
                prepGrab(FieldPosition.RED_QUARY);
                return Unit.INSTANCE;
            }
        };
    }

    private Function0 grab(){
        return new Function0() {
            @Override
            public Object invoke() {
                grabStone(FieldPosition.RED_QUARY);
                return Unit.INSTANCE;
            }
        };
    }

    private void init(){
        Thread thread = new Thread(){
            public void run() {
                hwMap.parkingServo.setPosition(TeleopConstants.parkingServoPosLock);
                hwMap.foundationLock.setPosition(TeleopConstants.foundationLockUnlock);
                hwMap.transferLock.setPosition(TeleopConstants.transferLockPosPlatform);
            }
        };
        thread.start();
        try{
            Thread.sleep(50);
        } catch(Exception e){}
    }

    private Function0 drop(){
        return new Function0() {
            @Override
            public Object invoke() {
                dropStone(FieldPosition.RED_QUARY);
                return Unit.INSTANCE;
            }
        };
    }
}
