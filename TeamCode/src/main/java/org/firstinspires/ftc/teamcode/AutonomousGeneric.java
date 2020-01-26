package org.firstinspires.ftc.teamcode;

import android.content.SharedPreferences;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.opencv.core.Mat;
import org.opencv.core.Core;
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
import static org.firstinspires.ftc.teamcode.AutonomousOptions.FIRST_BLOCK_BY_WALL_PREF;
import static org.firstinspires.ftc.teamcode.AutonomousOptions.START_POS_MODES_PREF;
//import static org.firstinspires.ftc.teamcode.AutonomousOptions.getSharedPrefs;
import static org.firstinspires.ftc.teamcode.AutonomousOptions.PARKING_PREF;
import static org.firstinspires.ftc.teamcode.AutonomousOptions.FOUNDATION_PREF;
import static org.firstinspires.ftc.teamcode.AutonomousOptions.DELIVER_ROUTE_PREF;
import static org.firstinspires.ftc.teamcode.AutonomousOptions.PARKING_ONLY_PREF;
import static org.firstinspires.ftc.teamcode.AutonomousOptions.STONE_PREF;

/**
 * 2019.10.26
 * Created by Ian Q.
 */
@TeleOp(name="SKYSTONE Autonomous", group="Main")
public class AutonomousGeneric extends LinearOpMode {

    RobotHardware robotHardware;
    RobotNavigator navigator;
    RobotProfile robotProfile;
    DriverOptions driverOptions;

    int leftEncoderCounts;
    int rightEncoderCounts;
    int horizontalEncoderCounts;

    ArrayList<RobotControl> taskList;

    long loopCount = 0;
    int countTasks = 0;
    int skystonePosition = 2;
    boolean needStoneRecognition = true;
    RobotProfile.StartPosition startPosition;
    String startingPositionModes;
    String parkingLocation;

    SequentialComboTask pickUpTask;
    private int delay;

    public void initRobot() {
        try {
            robotProfile = RobotProfile.loadFromFile(new File("/sdcard/FIRST/profile.json"));
        } catch (Exception e) {
        }

        Logger.init();

//        robotHardware = new RobotHardware();
//        robotHardware.init(hardwareMap, robotProfile);
        robotHardware = RobotFactory.getRobotHardware(hardwareMap,robotProfile);
        robotHardware.setClampPosition(RobotHardware.ClampPosition.INITIAL);
        robotHardware.setMotorStopBrake(true);

        navigator = new RobotNavigator(robotProfile);
        navigator.reset();
        navigator.setInitPosition(0, 0, 0);

        robotHardware.getBulkData1();
        robotHardware.getBulkData2();
        robotHardware.retractBlockHolderWheel();

        driverOptions = new DriverOptions();

        Logger.logFile("Init completed");
        Logger.logFile("DistancePID:" + robotProfile.distancePID.p + ", " + robotProfile.distancePID.i + ", " + robotProfile.distancePID.d);
        Logger.logFile("DistancePID:" + robotProfile.headingPID.p + ", " + robotProfile.headingPID.i + ", " + robotProfile.headingPID.d);

        SharedPreferences prefs = AutonomousOptions.getSharedPrefs(hardwareMap);

        try {
            String delaystring = prefs.getString(DELAY_PREF, "");
            delaystring = delaystring.replace(" sec", "");
            driverOptions.setDelay(Integer.parseInt(delaystring));
            Logger.logFile("delay: "+ this.delay);

            driverOptions.setParking(prefs.getString(PARKING_PREF,""));
            driverOptions.setStartingPositionModes(prefs.getString(START_POS_MODES_PREF, ""));
            driverOptions.setDeliverRoute(prefs.getString(DELIVER_ROUTE_PREF,""));
            driverOptions.setMoveFoundation(prefs.getString(FOUNDATION_PREF,""));
            driverOptions.setIsParkOnly(prefs.getString(PARKING_ONLY_PREF,""));
            driverOptions.setIsTwoSkystones(prefs.getString(STONE_PREF,""));
            driverOptions.setIsFirstBlockByWall(prefs.getString(FIRST_BLOCK_BY_WALL_PREF, ""));

            Logger.logFile("parking: "+ driverOptions.getParking());
            Logger.logFile("startingPositionModes: "+ driverOptions.getStartingPositionModes());
            Logger.logFile("deliverRoute: " + driverOptions.getDeliverRoute());
            Logger.logFile("moveFoundation: " + driverOptions.getMoveFoundation());
            Logger.logFile("isParkOnly: " + driverOptions.getIsParkOnly());
            Logger.logFile("isTwoSkystone: " + driverOptions.getIsTwoSkystones());
        } catch (Exception e) {
            this.delay = 0;
        }

         double heading;
         if (driverOptions.getStartingPositionModes().equals("RED_2")) {
            startPosition = RobotProfile.StartPosition.RED_2;
            heading = 0;
         } else if (driverOptions.getStartingPositionModes().equals("RED_3")) {
             startPosition = RobotProfile.StartPosition.RED_3;
             heading = 0;
         } else if (driverOptions.getStartingPositionModes().equals("RED_5")) {
             startPosition = RobotProfile.StartPosition.RED_5;
             heading = 0;
             needStoneRecognition = false;
        } else if (driverOptions.getStartingPositionModes().equals("BLUE_2")) {
            startPosition = RobotProfile.StartPosition.BLUE_2;
            heading = 0;
        } else if (driverOptions.getStartingPositionModes().equals("BLUE_3")){
            startPosition = RobotProfile.StartPosition.BLUE_3;
            heading = 0;
        } else {
            startPosition = RobotProfile.StartPosition.BLUE_5;
            heading = 0;
            needStoneRecognition = false;
         }

        navigator.setInitPosition(0,0, 0);
//        Logger.logFile("init x: " + robotProfile.robotStartPoints.get(startPosition).getX());
//        Logger.logFile("init y: " + robotProfile.robotStartPoints.get(startPosition).getY());
//        Logger.logFile("heading" + heading );

        //commented out because we only need to retrieve the option and no need to modify the editor in here.
        //SharedPreferences.Editor editor = prefs.edit();
        //editor.putString(START_POS_MODES_PREF, driverOptions.getStartingPositionModes());
        //editor.putString(PARKING_PREF, driverOptions.getParking());
        //editor.apply();
    }

    @Override
    public void runOpMode() {
        initRobot();
        OpenCvCamera phoneCam = null;
        if (needStoneRecognition) {
            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            phoneCam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
            phoneCam.openCameraDevice();
            phoneCam.setPipeline(new Pipeline());
            phoneCam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
        }
        //setUpTaskList();

        waitForStart();
        if (needStoneRecognition) {
            phoneCam.closeCameraDevice();
        }
        // initial navigator, reset position since the encoder might have moved during adjustment
        robotHardware.getBulkData1();
        robotHardware.getBulkData2();

        leftEncoderCounts = robotHardware.getEncoderCounts(RobotHardware.EncoderType.LEFT);
        rightEncoderCounts = robotHardware.getEncoderCounts(RobotHardware.EncoderType.RIGHT);
        horizontalEncoderCounts = robotHardware.getEncoderCounts(RobotHardware.EncoderType.HORIZONTAL);
        navigator.updateEncoderPos(leftEncoderCounts, rightEncoderCounts, horizontalEncoderCounts);
        navigator.reset();

        // do the task list building after click start, which we should have the skystone position
       // skystonePosition = 1;   // until we have the phone camera pointing correctly
        Logger.logFile("SkyStone Position: " + skystonePosition);
        AutonomousTaskBuilder builder = new AutonomousTaskBuilder(driverOptions, skystonePosition, robotHardware, navigator, robotProfile);
        if (driverOptions.getIsParkOnly().contains("yes")) {                         //5 points - do nothing but parking
            taskList = builder.buildParkingOnlyTask(driverOptions.getParking());
        } else if (driverOptions.getIsTwoSkystones().contains("yes")) {               //25 points - pick up and drop off two stones across bridge line plus parking
            taskList = builder.buildDropTwoStoneTask();
        } else if (driverOptions.getIsFirstBlockByWall()) {                           //15 points - pick up closest block by the wall and then park
            taskList = builder.buildPickUpFirstBlockAndPark();
        } else if (driverOptions.getMoveFoundation().equals("move only")) {           //15 points - only do the platform movement and parking
            taskList = builder.buildMovePlatformAndParkTask();
        }else if(driverOptions.getMoveFoundation().equals("no move")){               //15 points - no platform movement, pick up only one stone, drop after bridge line and parking only
            taskList = builder.buildDeliverOneStoneOnlyTask();
        } else {
            taskList = builder.buildDeliverOneStoneCompleteTask();                   //29 points, pick up sky stone, deliver and relocate platform, parking
        }

        Logger.logFile("Task list items: " + taskList.size());

        if (taskList.size() > 0) {
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
                    Logger.logFile("MainTaskComplete: " + taskList.get(0) + " Position:" + navigator.getWorldX() + "," + navigator.getWorldY() + " :" + navigator.getHeading());
                    Logger.flushToFile();

                    taskList.get(0).cleanUp();
                    taskList.remove(0);

                    countTasks++;
                    telemetry.update();

                    if (taskList.size() > 0) {
                        taskList.get(0).prepare();
                    }
                }
            }
        }

        // Regardless, open the clamp to save the servo
        try {
            Logger.logFile("Autonomous - Final Location:" + navigator.getLocationString());
            Logger.flushToFile();
        } catch (Exception ex) {
        }

        robotHardware.setClampPosition(RobotHardware.ClampPosition.INITIAL);
        robotHardware.rotateGrabberOriginPos();
    }

    void setUpTaskList() {
        taskList = new ArrayList<RobotControl>();
        taskList.add(new RobotSleep(3000));
//        ArrayList<RobotControl> comboList = new ArrayList<RobotControl>();
//
//        comboList.add(new RobotSleep(1000));
//        comboList.add(new SetLiftPositionTask(robotHardware, robotProfile, robotProfile.hardwareSpec.liftStoneBase + robotProfile.hardwareSpec.liftGrabExtra, 2000));
//        comboList.add(new RobotSleep(1000));
//        comboList.add(new ClampStraightAngleTask(robotHardware, robotProfile));
//        comboList.add(new RobotSleep(1000));
//        comboList.add(new SetSliderPositionTask(robotHardware, robotProfile, robotProfile.hardwareSpec.sliderOutPosAutonomous, 2000));
//        comboList.add(new RobotSleep(1000));
//        comboList.add(new ClampOpenCloseTask(robotHardware, robotProfile, RobotHardware.ClampPosition.OPEN));
//        comboList.add(new RobotSleep(1000));
//        comboList.add(new SetLiftPositionTask(robotHardware, robotProfile, robotProfile.hardwareSpec.liftStoneBase, 2000));
//        comboList.add(new RobotSleep(1000));
//        comboList.add(new ClampOpenCloseTask(robotHardware, robotProfile, RobotHardware.ClampPosition.CLOSE));
//        comboList.add(new RobotSleep(1000));
//        comboList.add(new SetLiftPositionTask(robotHardware, robotProfile, robotProfile.hardwareSpec.liftStoneBase + robotProfile.hardwareSpec.liftPerStone, 2000));
//        comboList.add(new RobotSleep(1000));
//
//        SequentialComboTask comboTask = new SequentialComboTask();
//
//        comboTask.setTaskList(comboList);
//        taskList.add(comboTask);
    }

//    void setupCombos() {
//        ArrayList<RobotControl> comboList = new ArrayList<RobotControl>();
//        comboList.add(new SetLiftPositionTask(robotHardware, robotProfile, robotProfile.hardwareSpec.liftStoneBase +
//                robotProfile.hardwareSpec.liftPerStone + robotProfile.hardwareSpec.liftGrabExtra, 1000));
//        comboList.add(new ClampStraightAngleTask(robotHardware, robotProfile));
//        comboList.add(new SetSliderPositionTask(robotHardware, robotProfile, robotProfile.hardwareSpec.sliderOutPosAutonomous, 1000));
//        comboList.add(new ClampOpenCloseTask(robotHardware, robotProfile, RobotHardware.ClampPosition.OPEN));
//        comboList.add(new SetLiftPositionTask(robotHardware, robotProfile, robotProfile.hardwareSpec.liftStoneBase +
//                robotProfile.hardwareSpec.liftGrabExtra, 1000));
//        comboList.add(new ClampOpenCloseTask(robotHardware, robotProfile, RobotHardware.ClampPosition.CLOSE));
//        comboList.add(new SetLiftPositionTask(robotHardware, robotProfile, robotProfile.hardwareSpec.liftGrabExtra, 1000));
//        pickUpTask = new SequentialComboTask();
//    }

        class Pipeline extends OpenCvPipeline {

        @Override
        public Mat processFrame(Mat input) {

            Scalar rectColor = new Scalar(255, 255, 255); //red color
            //TODO: the 3 stones' center coordinates are hardcoded for now, but will be not later on

            //gets point array
            RobotProfile.Point [] points = robotProfile.stoneScanPoints.get(startPosition);

            int rectLength = 30;
            int rectHalf = rectLength / 2;

            for(int i = 0; i < 3; i++) {
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
}
