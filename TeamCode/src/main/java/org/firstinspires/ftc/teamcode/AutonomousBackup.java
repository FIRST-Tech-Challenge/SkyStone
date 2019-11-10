package org.firstinspires.ftc.teamcode;

import android.content.SharedPreferences;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.io.File;

import static org.firstinspires.ftc.teamcode.AutonomousOptions.START_POS_MODES_PREF;

@TeleOp(name="AutonomousBackup", group="Main")
public class AutonomousBackup extends LinearOpMode {

    RobotHardware robotHardware;
    RobotNavigator navigator;
    RobotProfile robotProfile;

    RobotProfile.StartPosition startPosition;
    String startingPositionModes;

    double distance;

    public void initRobot() {
        try{
            robotProfile = RobotProfile.loadFromFile(new File("/sdcard/FIRST/profile.json"));
        } catch (Exception e) {
        }
        Logger.init();
        robotHardware = new RobotHardware();
        robotHardware.init(hardwareMap, robotProfile);
        navigator = new RobotNavigator(robotProfile);
        navigator.reset();
        SharedPreferences prefs = AutonomousOptions.getSharedPrefs(hardwareMap);

        startingPositionModes =  prefs.getString(START_POS_MODES_PREF, "");
        double heading;
        if (startingPositionModes.equals("RED_2")) {
            startPosition = RobotProfile.StartPosition.RED_2;
            heading = 0;
        }
        else if (startingPositionModes.equals("RED_3")) {
            startPosition = RobotProfile.StartPosition.RED_3;
            heading = 0;
        }
        else if (startingPositionModes.equals("BLUE_2")) {
            startPosition = RobotProfile.StartPosition.BLUE_2;
            heading =0;
        }
        else {
            startPosition = RobotProfile.StartPosition.BLUE_3;
            heading =0;
        }

        navigator.setInitPosition(0,0, 0);

    }
    @Override
    public void runOpMode() {

        initRobot();

        waitForStart();
        robotHardware.setMotorPower(0.5,0.5,0.5,0.5);
        while (opModeIsActive()) {

            robotHardware.getBulkData1();
            long counts = robotHardware.getEncoderCounts(RobotHardware.EncoderType.RIGHT);
            distance = (counts * 3.8*Math.PI)/1440;

            if(startPosition == RobotProfile.StartPosition.RED_2 || startPosition == RobotProfile.StartPosition.BLUE_2){
                if(distance > 90){
                    robotHardware.setMotorPower(0,0,0,0);
                }
            }
            if(startPosition == RobotProfile.StartPosition.RED_3 || startPosition == RobotProfile.StartPosition.BLUE_3) {
                if (distance > 30) {
                    robotHardware.setMotorPower(0, 0, 0, 0);
                }

            }
        }

    }
}
