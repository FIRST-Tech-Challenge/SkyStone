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
import org.firstinspires.ftc.teamcode.PID.RobotLogger;
import org.firstinspires.ftc.teamcode.PID.localizer.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.PID.localizer.VuforiaCamLocalizer;
import org.firstinspires.ftc.teamcode.PID.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.PID.mecanum.SampleMecanumDriveREV;
import org.firstinspires.ftc.teamcode.PID.mecanum.SampleMecanumDriveREVOptimized;
import org.firstinspires.ftc.teamcode.TeleOp.Teleop;
import org.firstinspires.ftc.teamcode.TeleOp.TeleopConstants;

import java.lang.reflect.Field;
import java.util.List;

import kotlin.Unit;
import kotlin.jvm.functions.Function0;

import static java.lang.Math.PI;
import static java.lang.Math.abs;
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
                Pose2d startingPos,
                com.qualcomm.robotcore.hardware.HardwareMap hardwareMap, BNO055IMU imu) {
        this.straightDrive = straightDrive;
        this.strafeDrive = straightDrive;
        this.startingPos = startingPos;
        this.hwMap = hwMap;
        this.opMode = opMode;
        this.hardwareMap = hardwareMap;
        align = new Align(hwMap, opMode, DcMotor.ZeroPowerBehavior.BRAKE);
        this.straightDrive.setPoseEstimate(startingPos);
        this.straightDrive.update();
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
    private SampleMecanumDriveBase DriveBuilderReset(boolean isStrafe, boolean init_imu, String label) {
        currentPos = _drive.getPoseEstimate();
        Pose2d error_pose = _drive.follower.getLastError();
        RobotLog.dd(TAG, "start new step: %s, count[%d], currentPos %s, errorPos %s",
                label, step_count++, currentPos.toString(), error_pose.toString());
        /*if (DriveConstantsPID.drvCorrection)
        {
            boolean done = false;
            if ((abs(error_pose.getX())>1.5))// && (abs(error_pose.getX())>abs(error_pose.getY())))
            {
                RobotLogger.dd(TAG, "pose correction by straight move");
                _drive.resetFollowerWithParameters(false, false);
                _drive.followTrajectorySync(
                        _drive.trajectoryBuilder()
                                .setReversed((error_pose.getX()>0)?false:true)
                                .lineTo(new Vector2d(newPos.getX() + error_pose.getX(), newPos.getY()))
                                .build());
                done = true;
                newPos = _drive.getPoseEstimate();
                RobotLogger.dd(TAG, "after pose correction: currentPos %s, errorPos %s",
                        newPos.toString(), _drive.follower.getLastError().toString());
            }
            if ((abs(error_pose.getY())>1.5))// && (abs(error_pose.getX())<abs(error_pose.getY())))
            {
                RobotLogger.dd(TAG, "pose correction by strafing");
                _drive.resetFollowerWithParameters(true, false);
                _drive.followTrajectorySync(
                        _drive.trajectoryBuilder()
                                .setReversed(false)
                                .strafeTo(new Vector2d(newPos.getX(), newPos.getY() + error_pose.getY()))
                                .build());
                done = true;
                newPos = _drive.getPoseEstimate();
                RobotLogger.dd(TAG, "after pose correction: currentPos %s, errorPos %s",
                        newPos.toString(), _drive.follower.getLastError().toString());
            }
            /*
            if (Math.toDegrees(error_pose.getHeading())>10)
            {
                RobotLog.dd(TAG, "correct heading by turning");
                _drive.resetFollowerWithParameters(false, false);
                _drive.turnSync(error_pose.getHeading());
                done = true;
                newPos = _drive.getPoseEstimate();
            }*/
         /*   if (done) {
                currentPos = newPos;
            }
        }*/
        //RobotLogger.dd(TAG, "vuforia localization info: %s", vu.getPoseEstimate().toString());

        if (DriveConstantsPID.RECREATE_DRIVE_AND_BUILDER) {
            if (DriveConstantsPID.USING_BULK_READ)
                _drive = new SampleMecanumDriveREVOptimized(hardwareMap, isStrafe);
            else
                _drive = new SampleMecanumDriveREV(hardwareMap, isStrafe);
        }
        else
            _drive.resetFollowerWithParameters(isStrafe, false);

        //_drive = new SampleMecanumDriveREV(hardwareMap, isStrafe, init_imu);
        _drive.getLocalizer().setPoseEstimate(currentPos);
        _drive.getLocalizer().update();
        if (!isStrafe) {
            builder = new TrajectoryBuilder(_drive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);
        } else {
            builder = new TrajectoryBuilder(_drive.getPoseEstimate(), DriveConstantsPID.STRAFE_BASE_CONSTRAINTS);
        }
        RobotLog.dd(TAG, "drive and builder created, initialized with pose: " + _drive.getPoseEstimate().toString());
        return _drive;
    }

    public void RedQuary(int[] skystonePositions) {
        DriveConstantsPID.updateConstantsFromProperties();
        switch (skystonePositions[0]) {
            case 1:
                double firstRegularStoneX = -34.0;
                double theta;

                double yCoordMvmtPlane = -23.0; //Y-coordinate value which the robot moves back and forth on
                double wallSkyStoneX = -40.0;   //X-coordinate of the Skystone closest to the wall
                double furtherMostSkyStoneX = -18.5;    //X-coordinate of the Skystone furthest away from the wall
                double foundationX = 45.0;  //X-coordinate of the foundation
                double strafeDistanceY = 7.0;    //Distance to strafe out before grabbing foundation
                double strafeDistanceX = 4.0;   //Distance to strafe down before grabbing foundation
                double reverseToFoundationInches = 13.0;    //Distance to back up to grab foundation
                double foundationDragXDecrease = 12.0;  //Decrease in X from original position when dragging foundation
                double foundationDragYDecrease = 32.0;  //Decrease in Y from original posiion when dragging foundation
                double parkingX = 0.0;    //X-coordinate of parking location
                double parkingY = -38.0;    //Y-coordinate of parking location

                transferReset();
                initIntakeClaw();
                init();

                //prepGrab(FieldPosition.RED_QUARY);    //*******

                DriveBuilderReset(true, false, "step2, after strafe, grab, to strafe back");

                builder = builder
                        .setReversed(false).strafeTo(new Vector2d(wallSkyStoneX, yCoordMvmtPlane));
                trajectory = builder.build();   //x - 2.812, y + 7.984
                _drive.followTrajectorySync(trajectory);

                RobotLog.dd(TAG, "step1.5, after strafe, to grab");
                //grabStone(FieldPosition.RED_QUARY);   //*******

                DriveBuilderReset(false, false, "step3, after strafe, to go straight");

                builder = builder
                        .setReversed(false).lineTo(new Vector2d(foundationX, yCoordMvmtPlane));
                trajectory = builder.build();   //x - 2.812, y + 7.984
                _drive.followTrajectorySync(trajectory);

                /*DriveBuilderReset(true, false, "step4, after long straight to drop stone");

                builder = builder
                        .setReversed(false).strafeTo(new Vector2d(foundationX, yCoordMvmtPlane + strafeDistance));
                trajectory = builder.build();   //x - 2.812, y + 7.984
                _drive.followTrajectorySync(trajectory);*/

                RobotLog.dd(TAG, "step4.5, after strafe");
                //dropStone(FieldPosition.RED_QUARY); //*******

                /*DriveBuilderReset(true, false, "step5, after strafe and drop stone");
                builder = builder
                        .setReversed(false).strafeTo(new Vector2d(foundationX, yCoordMvmtPlane));
                trajectory = builder.build();   //x - 2.812, y + 7.984
                _drive.followTrajectorySync(trajectory);*/

                DriveBuilderReset(false, false, "step6, after strafe, straight move back?");
                builder = builder
                        .setReversed(true).lineTo(new Vector2d(furtherMostSkyStoneX, yCoordMvmtPlane));
                trajectory = builder.build();   //x - 2.812, y + 7.984
                _drive.followTrajectorySync(trajectory);

                DriveBuilderReset(true, false, "step7, after straight move, to prepGrab");
                //prepGrab(FieldPosition.RED_QUARY); //*******

                /*builder = builder
                        .setReversed(false).strafeTo(new Vector2d(furtherMostSkyStoneX, yCoordMvmtPlane + strafeDistance));
                trajectory = builder.build();   //x - 2.812, y + 7.984
                _drive.followTrajectorySync(trajectory);
                RobotLog.dd(TAG, "step7.5 after strafe, to grab");*/

                //grabStone(FieldPosition.RED_QUARY);   //*******

                /*DriveBuilderReset(true, false, "step8, after strafe and grab");
                builder = builder
                        .setReversed(false).strafeTo(new Vector2d(furtherMostSkyStoneX, yCoordMvmtPlane));
                trajectory = builder.build();   //x - 2.812, y + 7.984
                _drive.followTrajectorySync(trajectory);*/

                DriveBuilderReset(false, false, "step9, after strafe back");

                builder = builder
                        .setReversed(false).lineTo(new Vector2d(foundationX, yCoordMvmtPlane));
                trajectory = builder.build();   //x - 2.812, y + 7.984
                _drive.followTrajectorySync(trajectory);

               /* DriveBuilderReset(true, false, "step10, after straight move");
                builder = builder
                        .setReversed(false).strafeTo(new Vector2d(foundationX + 3, yCoordMvmtPlane + strafeDistance));
                trajectory = builder.build();   //x - 2.812, y + 7.984
                _drive.followTrajectorySync(trajectory);*/

                RobotLog.dd(TAG, "step10.5, after strafe, to drop");
                //dropStone(FieldPosition.RED_QUARY);   //*******

                DriveBuilderReset(true, false, "step10, after straight move");
                builder = builder
                        .setReversed(false).strafeTo(new Vector2d(foundationX - strafeDistanceX, yCoordMvmtPlane - strafeDistanceY));
                trajectory = builder.build();   //x - 2.812, y + 7.984
                _drive.followTrajectorySync(trajectory);

                DriveBuilderReset(false, false, "step11, after strafe and drop");
                theta = _drive.getExternalHeading() >= 0 ? _drive.getExternalHeading() :
                        _drive.getExternalHeading() + 2 * PI;

                if (theta > PI)
                    _drive.turnSync(-(_drive.getExternalHeading() - 3 * PI / 2) + PI / 6);
                else
                    _drive.turnSync(-(_drive.getExternalHeading() + 2 * PI - 3 * PI / 2) + PI / 6);

                hwMap.foundationLock.setPosition(TeleopConstants.foundationLockUnlock);
                hwMap.transferLock.setPosition(TeleopConstants.transferLockPosOut);

                try {
                    Thread.sleep(100);
                } catch (Exception e) {
                }

                DriveBuilderReset(false, false, "step12, after turn");
                builder = builder.setReversed(true).lineTo(new Vector2d(_drive.getPoseEstimate().getX(),
                        _drive.getPoseEstimate().getY() + reverseToFoundationInches));
                trajectory = builder.build();   //x - 2.812, y + 7.984
                _drive.followTrajectorySync(trajectory);

                hwMap.foundationLock.setPosition(TeleopConstants.foundationLockLock);
                hwMap.transferLock.setPosition(TeleopConstants.transferLockPosPlatform);

                try {
                    Thread.sleep(100);
                } catch (Exception e) {
                }

                dropStone(FieldPosition.RED_QUARY);

                try {
                    Thread.sleep(50);
                } catch (Exception e) {
                }

                DriveBuilderReset(false, false, "step13, after straight move, Dragging foundation");
                builder = builder.setReversed(false)
                        .splineTo(new Pose2d(new Vector2d(_drive.getPoseEstimate().getX() - foundationDragXDecrease,
                                _drive.getPoseEstimate().getY() - foundationDragYDecrease), PI));
                trajectory = builder.build();   //x - 2.812, y + 7.984
                _drive.followTrajectorySync(trajectory);

                hwMap.foundationLock.setPosition(TeleopConstants.foundationLockUnlock);
                hwMap.transferLock.setPosition(TeleopConstants.transferLockPosOut);

                try {
                    Thread.sleep(100);
                } catch (Exception e) {
                }

                DriveBuilderReset(false, false, "step14, after spline");
                //builder = new TrajectoryBuilder(_drive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);
                builder = builder
                        .setReversed(false).splineTo(new Pose2d(new Vector2d(parkingX, parkingY), PI));
                trajectory = builder.build();   //x - 2.812, y + 7.984
                _drive.followTrajectorySync(trajectory);

                break;
            case 2:
                yCoordMvmtPlane = -42.8;
                wallSkyStoneX = -54.8;
                furtherMostSkyStoneX = -29.5;
                firstRegularStoneX = -34.0;
                foundationX = 46.0;
                strafeDistanceY = 9.5;

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
                            .strafeTo(new Vector2d(strafeDrive.getPoseEstimate().getX(), yCoordMvmtPlane + strafeDistanceY));

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
                            .lineTo(new Vector2d(foundationX, straightDrive.getPoseEstimate().getY() + 1));
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
                    builder = builder.strafeTo(new Vector2d(foundationX, yCoordMvmtPlane + strafeDistanceY));
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

                hwMap.parkingServo.setPosition(TeleopConstants.parkingServoPosLock);
                hwMap.foundationLock.setPosition(TeleopConstants.foundationLockUnlock);
                hwMap.transferLock.setPosition(TeleopConstants.transferLockPosOut);

                strafeDrive = DriveBuilderReset(true, false, "step17.5");

                if (DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {
                    builder = builder.setReversed(false).strafeTo(new Vector2d(foundationX, yCoordMvmtPlane - 1));
                    //.strafeTo(new Vector2d(42, -30));
                }
                trajectory = builder.build();
                strafeDrive.followTrajectorySync(trajectory);
                currentPos = strafeDrive.getPoseEstimate();

                straightDrive = DriveBuilderReset(true, false, "step17.0, dropped 3rd stones");

                /*if (getIMUAngle() < Math.PI)
                    straightDrive.turnSync(-((getIMUAngle() + 2 * Math.PI) - 3 * Math.PI / 2));
                else
                    straightDrive.turnSync(-(getIMUAngle() - 3 * Math.PI / 2));*/
                currentPos = strafeDrive.getPoseEstimate();

                straightDrive = DriveBuilderReset(false, false, "step18");

                if (DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {
                    builder = builder.setReversed(false)
                            .setReversed(true).lineTo(new Vector2d(straightDrive.getPoseEstimate().getX(), -28));
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
                            .setReversed(false).lineTo(new Vector2d(straightDrive.getPoseEstimate().getX() - 6, -54));
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
                straightDrive.followTrajectorySync(trajectory);
                break;
            case 3:
                yCoordMvmtPlane = -43.5;
                wallSkyStoneX = -62.5;
                furtherMostSkyStoneX = -39.0;
                firstRegularStoneX = -36.0;
                foundationX = 46.0;
                strafeDistanceY = 9.5;

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
                            .strafeTo(new Vector2d(strafeDrive.getPoseEstimate().getX(), yCoordMvmtPlane + strafeDistanceY));

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
                            .lineTo(new Vector2d(foundationX, straightDrive.getPoseEstimate().getY() + 2));
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
                    builder = builder.strafeTo(new Vector2d(foundationX, yCoordMvmtPlane + strafeDistanceY));
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
                    builder = builder.setReversed(false).strafeTo(new Vector2d(foundationX, yCoordMvmtPlane - 3));
                    //.strafeTo(new Vector2d(42, -30));
                }
                trajectory = builder.build();
                strafeDrive.followTrajectorySync(trajectory);
                currentPos = strafeDrive.getPoseEstimate();

                straightDrive = DriveBuilderReset(true, false, "step17.0, dropped 3rd stones");

                /*if (getIMUAngle() < Math.PI)
                    straightDrive.turnSync(-((getIMUAngle() + 2 * Math.PI) - 3 * Math.PI / 2));
                else
                    straightDrive.turnSync(-(getIMUAngle() - 3 * Math.PI / 2));*/
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
                            .setReversed(false).lineTo(new Vector2d(straightDrive.getPoseEstimate().getX() - 6, -54));
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
                straightDrive.followTrajectorySync(trajectory);
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
                double yCoordMvmtPlane = 43.5;
                double wallSkyStoneX = -49.5;
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
                            .lineTo(new Vector2d(foundationX, straightDrive.getPoseEstimate().getY() - 1));
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

                /*if (getIMUAngle() < Math.PI)
                    straightDrive.turnSync(Math.PI / 2 - getIMUAngle());
                else
                    straightDrive.turnSync(Math.PI / 2 - (getIMUAngle() - 2 * PI));*/
                currentPos = strafeDrive.getPoseEstimate();

                straightDrive = DriveBuilderReset(false, false, "step18");

                if (DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {
                    builder = builder.setReversed(false)
                            .setReversed(true).lineTo(new Vector2d(straightDrive.getPoseEstimate().getX(), 31));
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
                            .setReversed(false).lineTo(new Vector2d(straightDrive.getPoseEstimate().getX() - 6, 58));
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
                yCoordMvmtPlane = 43.5;
                wallSkyStoneX = -56.5;
                furtherMostSkyStoneX = -30.5;
                firstRegularStoneX = -34.0;
                foundationX = 47.0;
                strafeDistance = -10.0;

               // hwMap.parkingServo.setPosition(TeleopConstants.parkingServoPosLock);
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
                            .lineTo(new Vector2d(foundationX, straightDrive.getPoseEstimate().getY() + 2));
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

                /*if (getIMUAngle() < Math.PI)
                    straightDrive.turnSync(Math.PI / 2 - getIMUAngle());
                else
                    straightDrive.turnSync(Math.PI / 2 - (getIMUAngle() - 2 * PI));*/
                currentPos = strafeDrive.getPoseEstimate();

                straightDrive = DriveBuilderReset(false, false, "step18");

                if (DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL) {
                    builder = builder.setReversed(false)
                            .setReversed(true).lineTo(new Vector2d(straightDrive.getPoseEstimate().getX(), 31.5));
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
                            .setReversed(false).lineTo(new Vector2d(straightDrive.getPoseEstimate().getX() - 6, 60));
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
                yCoordMvmtPlane = 46.0;
                wallSkyStoneX = -63.5;
                furtherMostSkyStoneX = -40.0;
                firstRegularStoneX = -36.0;
                foundationX = 47.0;
                strafeDistance = -10.0;

                //hwMap.parkingServo.setPosition(TeleopConstants.parkingServoPosLock);
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

                /*if (getIMUAngle() < Math.PI)
                    straightDrive.turnSync(Math.PI / 2 - getIMUAngle());
                else
                    straightDrive.turnSync(Math.PI / 2 - (getIMUAngle() - 2 * PI));*/
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
                            .setReversed(false).lineTo(new Vector2d(straightDrive.getPoseEstimate().getX() - 6, 58));
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
        //hwMap.parkingServo.setPosition(TeleopConstants.parkingServoPosLock);
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
                hwMap.clawServo2.setPosition(0.9336);

                try{
                    Thread.sleep(2800);
                } catch(Exception e){}

                //hwMap.clawServo2.setPosition(TeleopConstants.clawServo2Block + 0.08);
                //resetLift(TeleopConstants.liftPower);
                try {
                    Thread.sleep(300);
                } catch (Exception e) {
                }
                hwMap.innerTransfer.setPosition(TeleopConstants.intakeInitPosRight);
                try {
                    Thread.sleep(500);
                } catch (Exception e) {
                }
                hwMap.innerTransfer.setPosition(TeleopConstants.intakeInitPosLeft);
                //intake(1);

                try {
                    Thread.sleep(500);
                } catch (Exception e) {
                }

                hwMap.innerTransfer.setPosition(TeleopConstants.intakeInitPosReset);
            }
        };

        Thread t = new Thread(){
            public void run(){
                //hwMap.clawServo2.setPosition(0.9336);
                try{
                    Thread.sleep(2600);
                } catch(Exception e){}
                //hwMap.clawInit.setPosition(TeleopConstants.clawInitPosReset);
                hwMap.clawInit.setPosition(TeleopConstants.clawInitPosReset);

                try {
                    Thread.sleep(600);
                } catch (Exception e) {
                }

                hwMap.clawInit.setPosition(TeleopConstants.clawInitPosCapstone);
            }
        };
        thread.start();
        t.start();
    }

    private void prepStone(double sliderEncoders) {
        Thread thread = new Thread() {
            public void run() {
                hwMap.foundationLock.setPosition(TeleopConstants.foundationLockUnlock);
                hwMap.transferLock.setPosition(TeleopConstants.transferLockPosOut);
                hwMap.clawServo2.setPosition(TeleopConstants.clawServo2Block);
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
        hwMap.redAutoClawJoint1.setPosition(TeleopConstants.autoClaw1Extended);
        try {
            Thread.sleep(100);
        } catch (Exception e) {
        }
        hwMap.redAutoClawJoint2.setPosition(TeleopConstants.autoClaw2Prep);
        try {
            Thread.sleep(100);
        } catch (Exception e) {
        }
        hwMap.redAutoClawJoint3.setPosition(TeleopConstants.autoClaw3Open);
        try {
            Thread.sleep(100);
        } catch (Exception e) {
        }
    }

    private void grabStone(FieldPosition fieldPosition) {
        hwMap.redAutoClawJoint2.setPosition(TeleopConstants.autoClaw2Grabbing);
        try {
            Thread.sleep(100);
        } catch (Exception e) {
        }
        hwMap.redAutoClawJoint3.setPosition(TeleopConstants.autoClaw3Closed);
        try {
            Thread.sleep(100);
        } catch (Exception e) {
        }
        hwMap.redAutoClawJoint1.setPosition(TeleopConstants.autoClaw1Stone);
        try {
            Thread.sleep(100);
        } catch (Exception e) {
        }
        hwMap.redAutoClawJoint2.setPosition(TeleopConstants.autoClaw2Init);
    }

    private void dropStone(FieldPosition fieldPosition) {
        /*hwMap.redAutoClawJoint2.setPosition(TeleopConstants.autoClaw1Down);
        try {
            Thread.sleep(100);
        } catch (Exception e) {
        }
        hwMap.redAutoClawJoint2.setPosition(TeleopConstants.autoClaw2Open);
        try {
            Thread.sleep(100);
        } catch (Exception e) {
        }
        hwMap.redAutoClawJoint1.setPosition(TeleopConstants.autoClaw1Init);
        try {
            Thread.sleep(100);
        } catch (Exception e) {
        }
        hwMap.redAutoClawJoint2.setPosition(TeleopConstants.autoClaw2Init);*/
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



    private void init() {
        Thread thread = new Thread() {
            public void run() {
                //hwMap.parkingServo.setPosition(TeleopConstants.parkingServoPosLock);
                hwMap.foundationLock.setPosition(TeleopConstants.foundationLockUnlock);
                hwMap.transferLock.setPosition(TeleopConstants.transferLockPosPlatform);
            }
        };
        thread.start();
        try {
            Thread.sleep(50);
        } catch (Exception e) {
        }
    }
}