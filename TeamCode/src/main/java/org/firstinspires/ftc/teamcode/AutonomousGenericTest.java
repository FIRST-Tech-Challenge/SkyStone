package org.firstinspires.ftc.teamcode;

import android.content.SharedPreferences;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.robot.Robot;

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
import static org.firstinspires.ftc.teamcode.AutonomousOptions.DELIVER_ROUTE_PREF;
import static org.firstinspires.ftc.teamcode.AutonomousOptions.FOUNDATION_PREF;
import static org.firstinspires.ftc.teamcode.AutonomousOptions.PARKING_ONLY_PREF;
import static org.firstinspires.ftc.teamcode.AutonomousOptions.PARKING_PREF;
import static org.firstinspires.ftc.teamcode.AutonomousOptions.START_POS_MODES_PREF;
import static org.firstinspires.ftc.teamcode.AutonomousOptions.STONE_PREF;

//import static org.firstinspires.ftc.teamcode.AutonomousOptions.getSharedPrefs;

/**
 * 2019.10.26
 * Created by Ian Q.
 */
@TeleOp(name="AutonomousTest", group="Test")
public class AutonomousGenericTest extends LinearOpMode {

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
    int skystonePosition = -1;
    RobotProfile.StartPosition startPosition;
    String startingPositionModes;
    String parkingLocation;

    SequentialComboTask pickUpTask;
    private int delay;

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
        }
        else if (driverOptions.getStartingPositionModes().equals("RED_3")) {
            startPosition = RobotProfile.StartPosition.RED_3;
            heading = 0;
        }
        else if (driverOptions.getStartingPositionModes().equals("BLUE_2")) {
            startPosition = RobotProfile.StartPosition.BLUE_2;
            heading =0;
        }
        else {
            startPosition = RobotProfile.StartPosition.BLUE_3;
            heading =0;
        }

        navigator.setInitPosition(0,0, 0);
        Logger.logFile("init x: " + robotProfile.robotStartPoints.get(startPosition).getX());
        Logger.logFile("init y: " + robotProfile.robotStartPoints.get(startPosition).getY());
        Logger.logFile("heading" + heading );

        SharedPreferences.Editor editor = prefs.edit();
        editor.putString(START_POS_MODES_PREF, driverOptions.getStartingPositionModes());
        editor.putString(PARKING_PREF, driverOptions.getParking());
        editor.apply();
    }

    @Override
    public void runOpMode() {
        initRobot();
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        OpenCvCamera phoneCam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        phoneCam.openCameraDevice();
        phoneCam.setPipeline(new Pipeline());
        phoneCam.startStreaming(640, 480, OpenCvCameraRotation.SIDEWAYS_LEFT);
        setUpTaskList();

        //
        //moveAndSlideToStone(new RobotPosition(0, 0, 0), new RobotPosition(70, 20, 0));
        taskList.add(new RobotSleep(10000));
        double origImu = robotHardware.getGyroAngle();

        waitForStart();
        phoneCam.closeCameraDevice();

        robotHardware.setClampPosition(RobotHardware.ClampPosition.INITIAL);
        if (taskList.size()>0) {
            taskList.get(0).prepare();
        }
        // run until the end of the match (driver presses STOP)
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
                    Logger.logFile("TaskComplete: " + taskList.get(0) + " Position:" + navigator.getWorldX() + "," + navigator.getWorldY() + " :" + navigator.getHeading());
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
            Logger.flushToFile();
        }
        catch (Exception ex) {
        }

        robotHardware.setClampPosition(RobotHardware.ClampPosition.INITIAL);
    }


    void setUpTaskList() {
        navigator.reset();
        //navigator.setInitPosition(0, 0, -Math.PI/2);
        RobotPosition lastPos ;

        taskList = new ArrayList<RobotControl>();
        HookPositionTask hookOn = new HookPositionTask(robotHardware, robotProfile, RobotHardware.HookPosition.HOOK_ON);
        taskList.add(hookOn);
        PIDMecanumMoveTask secondMove = new PIDMecanumMoveTask(robotHardware, robotProfile, navigator);
        secondMove.setPath(new RobotPosition(0,0,0), new RobotPosition(30, 0, 0));
        taskList.add(secondMove);
        taskList.add(new RobotSleep(100));
        PIDMecanumMoveTask lastMove = new PIDMecanumMoveTask(robotHardware, robotProfile, navigator);
        lastMove.setPath(new RobotPosition(30,0,0), new RobotPosition(30, 45, 0));
        taskList.add(lastMove);
        MecanumRotateTask mecanumRotate = new MecanumRotateTask(robotHardware, robotProfile,navigator);
        mecanumRotate.setPower(0.8);
        mecanumRotate.setMinPower(0.8);
        mecanumRotate.setRotateHeading(new RobotPosition(30,45,0), new RobotPosition(45,65,0.5*Math.PI));
        taskList.add(mecanumRotate);


        taskList.add(new RobotSleep(5000));
        HookPositionTask hookOff = new HookPositionTask(robotHardware, robotProfile, RobotHardware.HookPosition.HOOK_OFF);
        taskList.add(hookOff);

        //BuildingPlateMoveTask plateTask = new BuildingPlateMoveTask(robotHardware, robotProfile, navigator, BuildingPlateMoveTask.ApproachFrom.RIGHT);
        //taskList.add(plateTask);
//        seqList.add(new ClampOpenCloseTask(robotHardware, robotProfile, RobotHardware.ClampPosition.OPEN));


        //        taskList = new ArrayList<RobotControl>();
//        taskList.add(new RobotSleep(1000));
//
//        SequentialComboTask seqTask = new SequentialComboTask();
//        ArrayList<RobotControl> seqList = new ArrayList<RobotControl>();
//        seqList.add(new WaitForNavigationTask(navigator, new RobotPosition(10, 300, 1),
//                        new RobotPosition(-10, 30, -1)));
//        seqList.add(new ClampOpenCloseTask(robotHardware, robotProfile, RobotHardware.ClampPosition.CLOSE));
//        seqList.add(new RobotSleep(200));
//        seqList.add(new ClampOpenCloseTask(robotHardware, robotProfile, RobotHardware.ClampPosition.OPEN));
//        seqList.add(new RobotSleep(200));
//        seqList.add(new ClampOpenCloseTask(robotHardware, robotProfile, RobotHardware.ClampPosition.CLOSE));
//        seqList.add(new RobotSleep(200));
//        seqList.add(new ClampOpenCloseTask(robotHardware, robotProfile, RobotHardware.ClampPosition.OPEN));
//        seqTask.setTaskList(seqList);
//
//        ParallelComboTask ptask = new ParallelComboTask();
//        ArrayList<RobotControl> parList = new ArrayList<RobotControl>();
//        PIDMecanumMoveTask lastMove = new PIDMecanumMoveTask(robotHardware, robotProfile, navigator);
//        lastMove.setPath(new RobotPosition(0,0,0), new RobotPosition(0, 200, 0));
//        parList.add(lastMove);
//        parList.add(seqTask);
//        ptask.setTaskList(parList);
//        taskList.add(ptask);
//
//


//        taskList.add(new HookPositionTask(robotHardware, robotProfile, RobotHardware.HookPosition.HOOK_ON));
//        taskList.add(new RobotSleep(500));
//        PIDMecanumMoveTask move1 = new PIDMecanumMoveTask(robotHardware, robotProfile, navigator);
//        move1.setPath(new RobotPosition(0,0,0), new RobotPosition(0, 45, 0));
//        taskList.add(move1);
//        taskList.add(new RobotSleep(2000));
//        MecanumRotateTask mecanumRotateBack = new MecanumRotateTask(robotHardware, robotProfile,navigator);
//        mecanumRotateBack.setRotateHeading(new RobotPosition(0, 55, 0), new RobotPosition(15, 55, Math.PI/2));
//        taskList.add(mecanumRotateBack);
//        taskList.add(new HookPositionTask(robotHardware, robotProfile, RobotHardware.HookPosition.HOOK_OFF));

//        MecanumRotateTask mecanumRotate = new MecanumRotateTask(robotHardware, robotProfile,navigator);
//        mecanumRotate.setRotateHeading(new RobotPosition(0,0,0), new RobotPosition(10,10,0.5*Math.PI));
//        taskList.add(mecanumRotate);
//        taskList.add(new RobotSleep(5000));
//        MecanumRotateTask mecanumRotateBack = new MecanumRotateTask(robotHardware, robotProfile,navigator);
//        mecanumRotateBack.setRotateHeading(new RobotPosition(10,10,0.5*Math.PI), new RobotPosition(0,20,0));
//        taskList.add(mecanumRotateBack);
//        MecanumRotateTask move = new MecanumRotateTask(robotHardware, robotProfile, navigator);
//        move.setRotateHeading(new RobotPosition(0,20,0), new RobotPosition(0,-50,-0.5*Math.PI));
//        taskList.add(move);
    }

        class Pipeline extends OpenCvPipeline {

        @Override
        public Mat processFrame(Mat input) {

            Scalar rectColor = new Scalar(255, 255, 255); //red color
            //TODO: the 3 stones' center coordinates are hardcoded for now, but will be not later on

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
}
