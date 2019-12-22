package org.firstinspires.ftc.teamcode;

import android.content.SharedPreferences;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.io.File;

@TeleOp(name="Test Autonomous", group="Main")
public class LinearOpModeTest extends LinearOpMode {
    RobotHardware robotHardware;
    RobotNavigator navigator;
    RobotProfile robotProfile;

    int leftEncoderCounts;
    int rightEncoderCounts;
    int horizontalEncoderCounts;
    int loopCount;

    int leftEncoder = 39322;
    int rightEncoder = 44157;

    public void initRobot() {
        try {
            robotProfile = RobotProfile.loadFromFile(new File("/sdcard/FIRST/profile.json"));
        } catch (Exception e) {
        }
        Logger.init();
        robotHardware = new RobotHardware();
        robotHardware.init(hardwareMap, robotProfile);
        navigator = new RobotNavigator(robotProfile);
        navigator.reset();
        navigator.setInitPosition(0, 0, 0);
        robotHardware.setClampPosition(RobotHardware.ClampPosition.INITIAL);
        Logger.logFile("Init completed");
        Logger.logFile("DistancePID:" + robotProfile.distancePID.p + ", " + robotProfile.distancePID.i + ", " + robotProfile.distancePID.d);
        Logger.logFile("DistancePID:" + robotProfile.headingPID.p + ", " + robotProfile.headingPID.i + ", " + robotProfile.headingPID.d);
        SharedPreferences prefs = AutonomousOptions.getSharedPrefs(hardwareMap);

    }

    public void runOpMode() {
        initRobot();
        telemetry.addData("init the robot","");
        Logger.logFile("init the robot");
        waitForStart();

        double angle1 = 15;
        double angle2 = -15;
        while (opModeIsActive()) {
            loopCount++;
            robotHardware.getBulkData1();
            robotHardware.getBulkData2();
            leftEncoderCounts = robotHardware.getEncoderCounts(RobotHardware.EncoderType.LEFT);
            rightEncoderCounts = robotHardware.getEncoderCounts(RobotHardware.EncoderType.RIGHT);
            horizontalEncoderCounts = robotHardware.getEncoderCounts(RobotHardware.EncoderType.HORIZONTAL);
            navigator.updateEncoderPos(leftEncoderCounts, rightEncoderCounts, horizontalEncoderCounts);

            Logger.logFile("LE: " + leftEncoderCounts);
            Logger.logFile("RE: " + rightEncoderCounts);
            Logger.logFile("HE: " + horizontalEncoderCounts);
            telemetry.addData("LE: ", leftEncoderCounts);
            telemetry.addData("RE: ", rightEncoderCounts);

            if (leftEncoderCounts < 10000 / 2 && rightEncoderCounts < 8800 / 2 ) {
                if(robotHardware.getGyroAngle()<angle1) {
                    telemetry.addData("angle is small than 15 - ", robotHardware.getGyroAngle());
                    robotHardware.mecanumDriveTest(-0.5, angle1 - robotHardware.getGyroAngle(), 0. - 25, 2);
                }else {
                    telemetry.addData("in here", "");
                    robotHardware.flMotor.setPower(-0.5);
                    robotHardware.rlMotor.setPower(-0.5);
                    robotHardware.frMotor.setPower(-0.5 * 0.88);
                    robotHardware.rrMotor.setPower(-0.5 * 0.88);
                }
                telemetry.update();
            }else if(leftEncoderCounts <= 10000 && rightEncoderCounts <= 8800 ){
                if(robotHardware.getGyroAngle() > angle2) {
                    robotHardware.mecanumDriveTest(-0.5, angle2 - robotHardware.getGyroAngle(), 0.25, 2);
                }else {
                    robotHardware.flMotor.setPower(-0.5);
                    robotHardware.rlMotor.setPower(-0.5);
                    robotHardware.frMotor.setPower(-0.5*0.88);
                    robotHardware.rrMotor.setPower(-0.5*0.88);
                }
            } else {
                robotHardware.flMotor.setPower(0);
                robotHardware.rlMotor.setPower(0);
                robotHardware.frMotor.setPower(0);
                robotHardware.rrMotor.setPower(0);
            }
        }
    }
}