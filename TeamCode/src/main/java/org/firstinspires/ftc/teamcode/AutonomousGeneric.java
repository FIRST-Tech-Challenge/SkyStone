package org.firstinspires.ftc.teamcode;

import android.content.SharedPreferences;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

import java.io.File;
import java.util.ArrayList;

import static org.firstinspires.ftc.teamcode.AutonomousOptions.DELAY_PREF;
import static org.firstinspires.ftc.teamcode.AutonomousOptions.START_POS_MODES_PREF;
//import static org.firstinspires.ftc.teamcode.AutonomousOptions.getSharedPrefs;
import static org.firstinspires.ftc.teamcode.AutonomousOptions.PARKING_PREF;

import java.util.Vector;
import org.opencv.core.Core;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

/**
 * 2019.10.26
 * Created by Ian Q.
 */


@TeleOp(name="AutonomousGeneric", group="Test")
public class AutonomousGeneric extends LinearOpMode {

    RobotHardware robotHardware;
    RobotNavigator navigator;
    RobotProfile robotProfile;

    int leftEncoderCounts;
    int rightEncoderCounts;
    int horizontalEncoderCounts;
    boolean isSkyStoneLeft = false;
    boolean isSkyStoneMiddle = false;
    boolean isSkyStoneRight = false;

    private int skystonePosition = -1;
    private boolean snappedPic = false;

    ArrayList<RobotControl> taskList = new ArrayList<>();

    long loopCount = 0;
    int countTasks = 0;
    private int delay;
    String startingPositionModes;
    String parkingLocation;

    RobotProfile.StartPosition startPosition = RobotProfile.StartPosition.RED_2;

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
        Logger.logFile("Hardware init completed");
        Logger.logFile("DistancePID:" + robotProfile.distancePID.p + ", " + robotProfile.distancePID.i + ", " + robotProfile.distancePID.d);
        Logger.logFile("DistancePID:" + robotProfile.headingPID.p + ", " + robotProfile.headingPID.i + ", " + robotProfile.headingPID.d);
        this.delay = 0;
        SharedPreferences prefs = AutonomousOptions.getSharedPrefs(hardwareMap);
        try {
            String delaystring = prefs.getString(DELAY_PREF, "");
            delaystring = delaystring.replace(" sec", "");
            this.delay = Integer.parseInt(delaystring);
            parkingLocation = prefs.getString(PARKING_PREF, "");
            startingPositionModes = prefs.getString(START_POS_MODES_PREF, "");

        } catch (Exception e) {
            this.delay = 0;
        }

        if (startingPositionModes.equals("RED_2")) {
            startPosition = RobotProfile.StartPosition.RED_2;
        } else if (startingPositionModes.equals("RED_3")) {
            startPosition = RobotProfile.StartPosition.RED_3;
        } else if (startingPositionModes.equals("BLUE_2")) {
            startPosition = RobotProfile.StartPosition.BLUE_2;
        } else{
            startPosition = RobotProfile.StartPosition.BLUE_3;
        }
        Logger.logFile("prefs: delay " + delay);
        Logger.logFile("parking Location" + parkingLocation );
        Logger.logFile("starting Position Modes" + startPosition);
        setUpTaskList();
        Logger.logFile("Number of tasks:" + taskList.size());
//
//        SharedPreferences.Editor editor = prefs.edit();
//        editor.putString(START_POS_MODES_PREF, startingPositionModes);
//        editor.putString(PARKING_PREF, parkingLocation);
//        editor.apply();
    }

    @Override
    public void runOpMode() {
        initRobot();
        // start camera view
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        OpenCvCamera phoneCam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        phoneCam.openCameraDevice();
        phoneCam.setPipeline(new AutonomousGeneric.Pipeline());
        phoneCam.startStreaming(640, 480, OpenCvCameraRotation.SIDEWAYS_LEFT);

        // taskBuilder
        // according to start position, and option, build the task list
        waitForStart();
        Logger.logFile("Skystone location:" + skystonePosition);
        phoneCam.stopStreaming();   // stop streaming, don't waste CPU
        phoneCam.closeCameraDevice();
        if (taskList.size()>0) {
            taskList.get(0).prepare();
        }

        while (opModeIsActive()) {
            loopCount++;
            robotHardware.getBulkData1();
            robotHardware.getBulkData2();
            leftEncoderCounts = robotHardware.getEncoderCounts(RobotHardware.EncoderType.LEFT);
            rightEncoderCounts = robotHardware.getEncoderCounts(RobotHardware.EncoderType.RIGHT);
            horizontalEncoderCounts = robotHardware.getEncoderCounts(RobotHardware.EncoderType.HORIZONTAL);
            navigator.updateAllPositions(leftEncoderCounts, rightEncoderCounts, horizontalEncoderCounts);

            if(taskList.size() > 0){
                taskList.get(0).execute();
                if(taskList.get(0).isDone()){
                    taskList.get(0).cleanUp();
                    taskList.remove(0);
                    countTasks++;
                    telemetry.update();
                    if(taskList.size() > 0){
                        taskList.get(0).prepare();
                    }
                }
            }
//            Logger.logFile("Loop:" + loopCount + " TaskSize:" + taskList.size() + " Task:" +
//                    (taskList.size()>0?taskList.get(0):"NA"));

            telemetry.addData("startPosition:", startPosition);
            telemetry.addData("Skystone:", skystonePosition);
            telemetry.addData("x pos", navigator.getWorldX());
            telemetry.addData("y pos", navigator.getWorldY());
            telemetry.addData("heading", navigator.getHeading());
            telemetry.addData("completed tasks", countTasks);
            telemetry.update();
        }
        Logger.flushToFile();
    }

    public class Pipeline extends OpenCvPipeline {

        @Override
        public Mat processFrame(Mat input) {

            Scalar rectColor = new Scalar(255, 255, 255); //red color

            //gets point array
            RobotProfile.Point [] points = robotProfile.stoneScanPoints.get(startPosition);
            int rectLength = 30;
            int rectHalf = rectLength / 2;
            for(int i = 0; i < 3; i++){
                Imgproc.rectangle(input, new Rect(points[i].x - rectHalf, points[i].y - rectHalf, 30, 30), rectColor);
            }

            Mat imgBright = new Mat();

            //convert normal picture into HSV picture (deals with brightness)
            Imgproc.cvtColor(input, imgBright, Imgproc.COLOR_BGR2HSV);

            //get the V channel image of imgBright and save as a new image
            Vector<Mat> hsv_planes = new Vector<>();
            Core.split(imgBright, hsv_planes);

            Mat imgV = hsv_planes.get(2);
            double darkestVal = 500;

            //crop imgV based on the 3 rectangles and save each as a new image in rectCrops
            for(int i = 0; i < 3; i++){
                Imgproc.rectangle(input, new Rect(points[i].x - rectHalf, points[i].y - rectHalf, 30, 30), rectColor);
                Mat subPic = new Mat(imgV,  new Rect(points[i].x - rectHalf, points[i].y - rectHalf, 30, 30));
                Scalar temp = Core.mean(subPic);
                if(temp.val[0] < darkestVal){
                    darkestVal = temp.val[0];
                    skystonePosition = i;
                }
                subPic.release();
            }
            hsv_planes.get(0).release();
            hsv_planes.get(1).release();
            hsv_planes.get(2).release();
            imgBright.release();
            return input;
        }
    }

    void setUpTaskList() {
//        taskList.add(new RobotSleep(1000));
//        taskList.add(new SetLiftPositionTask(robotHardware, robotProfile, robotProfile.hardwareSpec.liftStoneBase + robotProfile.hardwareSpec.liftGrabExtra, 2000));
//        taskList.add(new RobotSleep(1000));
//        taskList.add(new ClampStraightAngleTask(robotHardware, robotProfile));
//        taskList.add(new RobotSleep(1000));
//        taskList.add(new SetSliderPositionTask(robotHardware, robotProfile, robotProfile.hardwareSpec.sliderOutPos, 2000));
//        taskList.add(new RobotSleep(1000));
//        taskList.add(new ClampOpenCloseTask(robotHardware, robotProfile, RobotHardware.ClampPosition.OPEN));
//        taskList.add(new RobotSleep(1000));
//        taskList.add(new SetLiftPositionTask(robotHardware, robotProfile, robotProfile.hardwareSpec.liftStoneBase, 2000));
//        taskList.add(new RobotSleep(1000));
//        taskList.add(new ClampOpenCloseTask(robotHardware, robotProfile, RobotHardware.ClampPosition.CLOSE));
//        taskList.add(new RobotSleep(1000));
//        taskList.add(new SetLiftPositionTask(robotHardware, robotProfile, robotProfile.hardwareSpec.liftStoneBase + robotProfile.hardwareSpec.liftPerStone, 2000));
//        taskList.add(new RobotSleep(1000));
//        taskList.add(new SetSliderPositionTask(robotHardware, robotProfile, robotProfile.hardwareSpec.sliderOrigPos, 2000));

        PIDMecanumMoveTask task_1;
        SetLiftPositionTask task_2;
        SetSliderPositionTask task_3;
        switch (startingPositionModes){
//            case ("RED_2") :
//                task_1 =  new PIDMecanumMoveTask(robotHardware, robotProfile, navigator);
//                if (isSkystoneLeft())
//                    task_1.setPath(robotProfile.RED_2[0], robotProfile.RED_2[1]);
//                if (isSkystoneMiddle())
//                    task_1.setPath(robotProfile.RED_2[0], robotProfile.RED_2[2]);
//                if (isSkystoneRight())
//                    task_1.setPath(robotProfile.RED_2[0], robotProfile.RED_2[3]);
//                taskList.add(task_1);
//
//                task_2 = new SetLiftPositionTask(robotHardware, robotProfile, robotProfile.hardwareSpec.liftGrabExtra, 2000);
//                taskList.add(task_2);
//                task_3 = new SetSliderPositionTask(robotHardware, robotProfile, robotProfile.hardwareSpec.sliderOutPos, 2000);
//                taskList.add(task_3);
        }

//        taskList.add(new PIDDriveStraightTask(robotHardware, navigator, 100));
//        taskList.add(new RobotSleep(3000));
//        taskList.add(new PIDDriveStraightTask(robotHardware, navigator, 200));


//        PIDMecanumMoveTask task1 = new PIDMecanumMoveTask(robotHardware, robotProfile,navigator);
//        task1.setPath(new RobotPosition(0,0,0), new RobotPosition(0, 200, 0));
//        taskList.add(task1);
//        taskList.add(new RobotSleep(3000));
//
//        PIDMecanumMoveTask task2 = new PIDMecanumMoveTask(robotHardware, robotProfile,navigator);
//        task2.setPath(new RobotPosition(0,200,0), new RobotPosition(0, 0, 0));
//        taskList.add(task2);
//        taskList.add(new RobotSleep(3000));
//
//        PIDMecanumMoveTask task3 = new PIDMecanumMoveTask(robotHardware, robotProfile,navigator);
//        task3.setPath(new RobotPosition(0,0,0), new RobotPosition(0, 200, 0));
//        taskList.add(task3);
//        taskList.add(new RobotSleep(3000));
//
//        PIDMecanumMoveTask task4 = new PIDMecanumMoveTask(robotHardware, robotProfile,navigator);
//        task4.setPath(new RobotPosition(0,200,0), new RobotPosition(0, 0, 0));
//        taskList.add(task4);
//        taskList.add(new RobotSleep(3000));

//        PIDMecanumMoveTask task2 = new PIDMecanumMoveTask(robotHardware, robotProfile, navigator);
//        task2.setPath(new RobotPosition(25,0,0), new RobotPosition(25, 50, 0));
//        taskList.add(task2);
    }
}
