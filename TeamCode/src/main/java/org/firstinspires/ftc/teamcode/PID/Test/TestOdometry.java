package org.firstinspires.ftc.teamcode.PID.Test;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.heading.HeadingInterpolator;
import com.acmerobotics.roadrunner.trajectory.BaseTrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.All.DriveConstant;
import org.firstinspires.ftc.teamcode.All.FourWheelMecanumDrivetrain;
import org.firstinspires.ftc.teamcode.All.HardwareMap;
import org.firstinspires.ftc.teamcode.PID.DriveConstantsPID;
import org.firstinspires.ftc.teamcode.PID.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.PID.mecanum.SampleMecanumDriveREV;

import java.util.ArrayList;

@Config
@Autonomous(name = "StraightLineOdometryTest", group = "drive")
public class TestOdometry extends LinearOpMode {
    FourWheelMecanumDrivetrain dt;
    HardwareMap hwMap;
    SampleMecanumDriveBase drive;
    private BaseTrajectoryBuilder builder;
    private Trajectory trajectory;
    private static double driftRemoverConstant = 1.3;   //inches

    private ArrayList<String> savedData = new ArrayList<>();

    private ArrayList<String> odometryLR = new ArrayList<>();
    private ArrayList<String> wheelLR = new ArrayList<>();
    private ArrayList<String> odometryCenter = new ArrayList<>();
    private ArrayList<String> imuAngle = new ArrayList<>();
    private ArrayList<String> error = new ArrayList<>();
    private int counter = 0;

    public void runOpMode(){
        hwMap = new HardwareMap(hardwareMap);
        dt = new FourWheelMecanumDrivetrain(hwMap);

        //dt.setMotorZeroPower(DcMotor.ZeroPowerBehavior.BRAKE);
        dt.resetEncoders();
        dt.setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
        dt.setRunningOpMode(this);
        hwMap.liftOne.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hwMap.liftOne.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hwMap.liftTwo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hwMap.liftOne.setDirection(DcMotorSimple.Direction.REVERSE);
        hwMap.frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        hwMap.backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        DriveConstantsPID.updateConstantsFromProperties();

        drive = new SampleMecanumDriveREV(hardwareMap);

        waitForStart();

        dt.resetEncoders();

        builder = new TrajectoryBuilder(drive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);

        counter = 0;

        collectData();

        drive.getLocalizer().setPoseEstimate(new Pose2d(new Vector2d(0,0),0));
        drive.getLocalizer().update();

        double driftRemover = 0.0;

        while(opModeIsActive()){
            /*drive.getLocalizer().setPoseEstimate(new Pose2d(new Vector2d(drive.getPoseEstimate().getX(),
                    drive.getPoseEstimate().getY()), drive.getPoseEstimate().getHeading()));    //Straight Test
            drive.getLocalizer().update();
            builder = new TrajectoryBuilder(drive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);
            builder = builder.setReversed(false).lineTo(new Vector2d(72,0));
            trajectory = builder.build();
            drive.followTrajectorySync(trajectory);

            try{
                Thread.sleep(500);
            } catch(Exception e){}

            drive.turnSync(Math.toRadians(-1));
            drive.getLocalizer().setPoseEstimate(new Pose2d(new Vector2d(drive.getPoseEstimate().getX(),
                   0), 0));
            drive.getLocalizer().update();
            builder = new TrajectoryBuilder(drive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);
            builder = builder.setReversed(true).lineTo(new Vector2d(0, 0));
            trajectory = builder.build();
            drive.followTrajectorySync(trajectory);*/

            drive.getLocalizer().setPoseEstimate(new Pose2d(new Vector2d(drive.getPoseEstimate().getX(),
                    drive.getPoseEstimate().getY()), drive.getPoseEstimate().getHeading()));    //Strafe Test
            drive.getLocalizer().update();
            builder = new TrajectoryBuilder(drive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);
            builder = builder.setReversed(false).strafeTo(new Vector2d(0,72));
            trajectory = builder.build();
            drive.followTrajectorySync(trajectory);

            try{
                Thread.sleep(500);
            } catch(Exception e){}

            drive.getLocalizer().setPoseEstimate(new Pose2d(new Vector2d(drive.getPoseEstimate().getX(),
                    drive.getPoseEstimate().getY()), drive.getPoseEstimate().getHeading()));
            drive.getLocalizer().update();
            builder = new TrajectoryBuilder(drive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);
            builder = builder.setReversed(false).strafeTo(new Vector2d(0,0));
            trajectory = builder.build();
            drive.followTrajectorySync(trajectory);

            //drive.turnSync(Math.PI);  //Rotation Test

            try{
                Thread.sleep(500);
            } catch(Exception e){}

            counter += 1;

            if(counter == 6)
                stop();
        }
    }

    private void collectData(){
        Thread thread = new Thread(){
            public void run() {
                while (!isStopRequested()) {
                    savedData.add("\n" + "FrontLeftWheel: " + hwMap.frontLeft.getCurrentPosition() + ", FrontRightWheel: " +
                            hwMap.frontRight.getCurrentPosition() + ", BackLeftWheel: " + hwMap.backLeft.getCurrentPosition() +
                            ", BackRightWheel: " + hwMap.backRight.getCurrentPosition() + ", FrontLeftOdo: " +
                            hwMap.leftIntake.getCurrentPosition() + ", FrontRightOdo: " + hwMap.liftTwo.getCurrentPosition() +
                            ", BackMiddleOdo: " + hwMap.rightIntake.getCurrentPosition() + ", IMU Angle (Rad): " +
                            drive.getExternalHeading() + ", IMU Angle (Deg): " +
                            Math.toDegrees(drive.getExternalHeading()));

                    odometryCenter.add(hwMap.rightIntake.getCurrentPosition() + "\n");
                    imuAngle.add(drive.getExternalHeading() + "\n");
                    odometryLR.add(hwMap.leftIntake.getCurrentPosition() + "," + hwMap.liftTwo.getCurrentPosition() + "\n");
                    wheelLR.add(hwMap.frontLeft.getCurrentPosition() + "," + hwMap.backLeft.getCurrentPosition() + "," +
                            hwMap.frontRight.getCurrentPosition() + "," + hwMap.backRight.getCurrentPosition() + "\n");
                    error.add(drive.getLastError().getX() + "," + drive.getLastError().getY() + "," + drive.getLastError().getHeading() + "\n");
                }
                if(counter == 6) {
                    String temp = odometryCenter.toString().replaceAll(",", "");
                    temp = temp.replaceAll("\\[", "");
                    temp = temp.replaceAll("]", "");
                    DriveConstant.writeFile(AppUtil.ROOT_FOLDER + "/RoadRunner/OdometryCenter_" +
                            System.currentTimeMillis() + ".csv", temp);

                    temp = imuAngle.toString().replaceAll(",", "");
                    temp = temp.replaceAll("\\[", "");
                    temp = temp.replaceAll("]", "");
                    DriveConstant.writeFile(AppUtil.ROOT_FOLDER + "/RoadRunner/ImuAngle_" +
                            System.currentTimeMillis() + ".csv", temp);

                    temp = odometryLR.toString().replaceAll("\\[", "");
                    temp = temp.replaceAll("]", "");
                    int counter = 0;
                    for (int i = 0; i < temp.length(); i++) {
                        if (temp.charAt(i) == ',') {
                            counter += 1;
                        }

                        if (counter == 2) {
                            temp = temp.substring(0, i) + temp.substring(i + 1);
                            counter = 0;
                        }
                    }
                    DriveConstant.writeFile(AppUtil.ROOT_FOLDER + "/RoadRunner/OdometryLR_" +
                            System.currentTimeMillis() + ".csv", temp);

                    temp = wheelLR.toString().replaceAll("\\[", "");
                    temp = temp.replaceAll("]", "");
                    counter = 0;
                    for (int i = 0; i < temp.length(); i++) {
                        if (temp.charAt(i) == ',') {
                            counter += 1;
                        }

                        if (counter == 4) {
                            temp = temp.substring(0, i) + temp.substring(i + 1);
                            counter = 0;
                        }
                    }
                    DriveConstant.writeFile(AppUtil.ROOT_FOLDER + "/RoadRunner/WheelLR_" +
                            System.currentTimeMillis() + ".csv", temp);

                    temp = error.toString().replaceAll("\\[", "");
                    temp = temp.replaceAll("]", "");
                    counter = 0;
                    for (int i = 0; i < temp.length(); i++) {
                        if (temp.charAt(i) == ',') {
                            counter += 1;
                        }

                        if (counter == 3) {
                            temp = temp.substring(0, i) + temp.substring(i + 1);
                            counter = 0;
                        }
                    }
                    DriveConstant.writeFile(AppUtil.ROOT_FOLDER + "/RoadRunner/EstimatedError_" +
                            System.currentTimeMillis() + ".csv", temp);

                    DriveConstant.writeFile(AppUtil.ROOT_FOLDER + "/RoadRunner/DriveVelRAWData_" +
                            System.currentTimeMillis() + ".txt", savedData.toString());
                }
            }
        };
        thread.start();
    }
}

