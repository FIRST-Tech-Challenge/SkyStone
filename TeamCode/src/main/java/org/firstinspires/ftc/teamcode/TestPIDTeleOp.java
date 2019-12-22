package org.firstinspires.ftc.teamcode;

import android.content.SharedPreferences;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import java.io.File;
import java.util.ArrayList;
import java.util.Vector;

import static org.firstinspires.ftc.teamcode.AutonomousOptions.DELAY_PREF;
import static org.firstinspires.ftc.teamcode.AutonomousOptions.PARKING_PREF;
import static org.firstinspires.ftc.teamcode.AutonomousOptions.START_POS_MODES_PREF;

//import static org.firstinspires.ftc.teamcode.AutonomousOptions.getSharedPrefs;

/**
 * 2019.10.26
 * Created by Ian Q.
 */
@TeleOp(name="TEST PID", group="Test")
@Disabled
public class TestPIDTeleOp extends LinearOpMode {

    RobotHardware robotHardware;
    RobotNavigator navigator;
    RobotProfile robotProfile;

    int leftEncoderCounts;
    int rightEncoderCounts;
    int horizontalEncoderCounts;

    ArrayList<RobotControl> taskList;

    long loopCount = 0;
    int countTasks = 0;



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
        navigator.setInitPosition(0, 0, 0);
        robotHardware.setClampPosition(RobotHardware.ClampPosition.INITIAL);
        Logger.logFile("Init completed");
        Logger.logFile("DistancePID:" + robotProfile.distancePID.p + ", " + robotProfile.distancePID.i + ", " + robotProfile.distancePID.d);
        Logger.logFile("DistancePID:" + robotProfile.headingPID.p + ", " + robotProfile.headingPID.i + ", " + robotProfile.headingPID.d);
    }

    @Override
    public void runOpMode() {
        initRobot();
        setUpTaskList();

        waitForStart();

        Logger.logFile("Task list items: " + taskList.size());
        robotHardware.setClampPosition(RobotHardware.ClampPosition.CLOSE);
        if (taskList.size()>0) {
            taskList.get(0).prepare();
        }
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            loopCount++;
            robotHardware.getBulkData1();
            robotHardware.getBulkData2();
            leftEncoderCounts = robotHardware.getEncoderCounts(RobotHardware.EncoderType.LEFT);
            rightEncoderCounts = robotHardware.getEncoderCounts(RobotHardware.EncoderType.RIGHT);
            horizontalEncoderCounts = robotHardware.getEncoderCounts(RobotHardware.EncoderType.HORIZONTAL);
            navigator.updateEncoderPos(leftEncoderCounts, rightEncoderCounts, horizontalEncoderCounts);

            if (taskList.size() > 0) {
                taskList.get(0).execute();
                if (taskList.get(0).isDone()) {
                    taskList.get(0).cleanUp();
                    taskList.remove(0);
                    countTasks++;
                    telemetry.update();
                    if (taskList.size() > 0) {
                        taskList.get(0).prepare();
                    }
                }
            }
            Logger.logFile("Loop:" + loopCount + " TaskSize:" + taskList.size() + " Task:" +
                    (taskList.size() > 0 ? taskList.get(0) : "NA"));
            Logger.logFile("x pos"+ navigator.getWorldX());
            Logger.logFile("y pos"+ navigator.getWorldY());
            Logger.logFile("heading"+ navigator.getHeading());
           // telemetry.update();
        }
        // Regardless, open the clamp to save the servo
        try {
            Logger.flushToFile();
        }
        catch (Exception ex) {
        }

        robotHardware.setClampPosition(RobotHardware.ClampPosition.INITIAL);
    }

    void setUpTaskList() {
        taskList = new ArrayList<RobotControl>();
        PIDMecanumMoveTask move = new PIDMecanumMoveTask(robotHardware, robotProfile, navigator);
        move.setPath(new RobotPosition(0, 0, 0), new RobotPosition(50, 0, 0));
        taskList.add(move);
        taskList.add(new RobotSleep(3000));
    }
}
