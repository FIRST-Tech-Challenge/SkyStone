package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.io.File;
import java.util.ArrayList;

@TeleOp(name="SKYSTONE DriverOpMode", group="Main")
//@Disabledxxs
public class DriverOpMode extends OpMode {
    RobotHardware robotHardware;
    RobotNavigator navigator;
    boolean fieldMode;
    RobotProfile robotProfile;

    int prevLiftEncoderCnt;
    double gyroAngleOffset;
    double gyroCurrAngle;
    int liftEncoderCnt;
    LiftDirection liftDir;
    boolean xAlreadyPressed = false;
    int sliderEncoderCnt;
    RobotControl currentTask = null;
    boolean aAlreadyPressed = false;
    boolean bAlreadyPressed = false;
    boolean yAlreadyPressed = false;
    boolean sliderPosReseted = false;
    SequentialComboTask homePositionTask;
    SequentialComboTask homeGrabTask;
    SequentialComboTask capstonePositionTask;
    SequentialComboTask deliveryPositionTask;

    @Override
    public void init() {
        try {
            robotProfile = RobotProfile.loadFromFile(new File("/sdcard/FIRST/profile.json"));
        } catch (Exception e) {
        }

        fieldMode = true; //robot starts in field orientation
        Logger.init();
        robotHardware = new RobotHardware();
        robotHardware.init(hardwareMap, robotProfile);

        gyroAngleOffset = robotHardware.getGyroAngle();

        navigator = new RobotNavigator(robotProfile);
        navigator.reset();
        navigator.setInitPosition(0, 0, 0);
//        if(!robotHardware.imu1.isGyroCalibrated()){
//
//        }
        setupCombos();
        robotHardware.rotateGrabberOriginPos();
        robotHardware.setClampPosition(RobotHardware.ClampPosition.OPEN);
        robotHardware.setHookPosition(RobotHardware.HookPosition.HOOK_OFF);
        robotHardware.setCapStoneServo(RobotHardware.CapPosition.CAP_UP);

        prevLiftEncoderCnt = 0;
        sliderPosReseted = false;
        liftDir = LiftDirection.LIFT_STOP;
    }

    @Override
    public void loop() {
        robotHardware.getBulkData1();
        robotHardware.getBulkData2();

        liftEncoderCnt = robotHardware.getEncoderCounts(RobotHardware.EncoderType.LIFT);
        sliderEncoderCnt = robotHardware.getEncoderCounts(RobotHardware.EncoderType.SLIDER);
        gyroCurrAngle = robotHardware.getGyroAngle();

        // Robot movement
        handleMovement();

        // toggle field mode on/off.
        // Driver 1: left trigger - enable; right trigger - disable
        if (gamepad1.left_trigger>0) {
            fieldMode = true;
            gyroAngleOffset = gyroCurrAngle;
        }
        else if (gamepad1.right_trigger>0) {
            fieldMode = false;  //good luck driving
        }

        // pulling hook
        // Driver 1: x - enable hook, y - off hook
        if(gamepad1.x){
            robotHardware.setHookPosition(RobotHardware.HookPosition.HOOK_ON);
        }else if(gamepad1.y){
            robotHardware.setHookPosition(RobotHardware.HookPosition.HOOK_OFF);
        }

        if (gamepad1.a) {
            robotHardware.setCapStoneServo(RobotHardware.CapPosition.CAP_DOWN);
        }
        else if (gamepad1.b) {
            robotHardware.setCapStoneServo(RobotHardware.CapPosition.CAP_UP);
        }

//
//        if(gamepad2.x){
//            robotHardware.rotateGrabberOriginPos();
//        }

        // Intake on/off
        // Driver 1: dpad up - spin, dpad down - stop spin
        if (gamepad1.dpad_up) {
            robotHardware.startIntakeWheel();
        }
        else if(gamepad1.dpad_down){
            robotHardware.reverseIntakeWheels();
        }
        else if(gamepad1.dpad_left) {
            robotHardware.stopIntakeWheel();
        }

        // clamp open or close
        // Driver 2: a - open, b - close
        if(gamepad2.a && !gamepad2.start){
            robotHardware.setClampPosition(RobotHardware.ClampPosition.OPEN);
        }else if (gamepad2.b && !gamepad2.start){
            robotHardware.setClampPosition(RobotHardware.ClampPosition.CLOSE);
        }

        if(!xAlreadyPressed) {
            if(gamepad2.x && !gamepad2.start){
                currentTask = deliveryPositionTask;
                currentTask.prepare();
                Logger.logFile(currentTask.toString());
                xAlreadyPressed = true;
            }
        }
        if (!gamepad2.x)
        {
            xAlreadyPressed = false;
        }

        // IF driver B is pressed, first will slide in, 2nd time will go down to grab
        if(!yAlreadyPressed && !gamepad2.start) {
            if(gamepad2.y) {
                if (robotHardware.getEncoderCounts(RobotHardware.EncoderType.SLIDER)<100) {
                    currentTask = homeGrabTask;
                }
                else {
                    currentTask = homePositionTask;
                }
                currentTask.prepare();
                Logger.logFile(currentTask.toString());
                yAlreadyPressed = true;
            }
        }
        if (!gamepad2.y)
        {
            yAlreadyPressed = false;
        }
//
//        if(!yAlreadyPressed) {
//            if(gamepad2.y && !gamepad2.start){
//                currentTask = capstonePositionTask;
//                currentTask.prepare();
//                Logger.logFile(currentTask.toString());
//                yAlreadyPressed = true;
//            }
//        }
//        if (!gamepad2.y)
//        {
//            yAlreadyPressed = false;
//        }

        //12/12
        if(gamepad2.left_bumper && currentTask!=capstonePositionTask){
                currentTask = capstonePositionTask;
                currentTask.prepare();
                Logger.logFile(currentTask.toString());
        }

        //12/13
        if(gamepad2.right_bumper){
            robotHardware.setCapStoneServo(RobotHardware.CapPosition.CAP_OTHER);
        }

//        if(gamepad2.right
//        _bumper){
//            robotHardware.setClampAnglePosition(RobotHardware.ClampAnglePosition.NORMAL);
//        }


        handleLiftAndSlide();

        if(currentTask != null) {
            currentTask.execute();
            if(currentTask.isDone()) {
                currentTask.cleanUp();
                currentTask = null;
            }
        }

        telemetry.addData("Field Mode", fieldMode);
        telemetry.addData("Gyro", gyroCurrAngle);

        telemetry.addData("Lift encoder", liftEncoderCnt);
        telemetry.addData("Slider encoder", sliderEncoderCnt);
        telemetry.addData("SliderTouched", robotHardware.sliderTouched());
        telemetry.addData("Right Distance", robotHardware.getRightDistance());
   }

   @Override
    public void stop() {
        // open the clamp to relief the grabber servo
       robotHardware.setClampPosition(RobotHardware.ClampPosition.OPEN);
       try {
           Logger.flushToFile();
       } catch (Exception e) {
       }
    }

   private void handleMovement() {
        double turn = gamepad1.right_stick_x/2;
       double power = Math.hypot(gamepad1.left_stick_x, -gamepad1.left_stick_y);
       double moveAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI/4;
       if (fieldMode) {
           moveAngle += Math.toRadians(gyroCurrAngle - gyroAngleOffset);
       }
       if (gamepad1.left_bumper) {
           power = power/3;
           turn = turn/12;
       }
       robotHardware.mecanumDriveTest(power, moveAngle, turn, 0);
   }

   void setupCombos() {
        //**** HOME POSITION TASK
       // Lift home, slide in, lift drop to ready position
       ArrayList<RobotControl> homePositionList = new ArrayList<RobotControl>();
       homePositionList.add(new SetLiftPositionTask(robotHardware, robotProfile, robotProfile.hardwareSpec.liftStoneBase +
               robotProfile.hardwareSpec.liftPerStone + robotProfile.hardwareSpec.liftGrabExtra, 500));
       homePositionList.add(new ClampStraightAngleTask(robotHardware, robotProfile));
       homePositionList.add(new SetSliderPositionTask(robotHardware, robotProfile, robotProfile.hardwareSpec.sliderOrigPos, 500));
       homePositionList.add(new ClampOpenCloseTask(robotHardware, robotProfile, RobotHardware.ClampPosition.OPEN));
       homePositionList.add(new SetLiftPositionTask(robotHardware, robotProfile, robotProfile.hardwareSpec.liftStoneBase +
               robotProfile.hardwareSpec.liftHomeReadyPos, 500));
       homePositionTask = new SequentialComboTask();
       homePositionTask.setTaskList(homePositionList);

        //*** HOME GRAB TASK
       // If already slide in, slide in again, lift drop to grab position and grab
       homeGrabTask = new SequentialComboTask();
       ArrayList<RobotControl> homeGrabList = new ArrayList<RobotControl>();
       homeGrabList.add(new SetSliderPositionTask(robotHardware, robotProfile, robotProfile.hardwareSpec.sliderOrigPos, 100));
       homeGrabList.add(new SetLiftPositionTask(robotHardware, robotProfile, robotProfile.hardwareSpec.liftStoneBase +
               robotProfile.hardwareSpec.liftHomeGrabPos, 500));
       homeGrabList.add(new ClampOpenCloseTask(robotHardware, robotProfile, RobotHardware.ClampPosition.CLOSE));
       homeGrabTask.setTaskList(homeGrabList);

       ArrayList<RobotControl> deliveryPositionList = new ArrayList<RobotControl>();
       deliveryPositionList.add(new SetLiftPositionTask(robotHardware, robotProfile, robotProfile.hardwareSpec.liftStoneBase +
               robotProfile.hardwareSpec.liftHomeGrabPos, 500));
       deliveryPositionList.add(new ClampOpenCloseTask(robotHardware, robotProfile, RobotHardware.ClampPosition.CLOSE));
       deliveryPositionList.add(new SetLiftPositionTask(robotHardware, robotProfile, robotProfile.hardwareSpec.liftStoneBase +
               robotProfile.hardwareSpec.liftPerStone + robotProfile.hardwareSpec.liftGrabExtra, 500));
       deliveryPositionList.add(new SetSliderPositionTask(robotHardware, robotProfile, robotProfile.hardwareSpec.sliderOutPos, 500));
       deliveryPositionTask = new SequentialComboTask();
       deliveryPositionTask.setTaskList(deliveryPositionList);

       ArrayList<RobotControl> capstonePositionList = new ArrayList<RobotControl>();
       capstonePositionList.add(new ClampOpenCloseTask(robotHardware, robotProfile, RobotHardware.ClampPosition.OPEN));
       capstonePositionList.add(new SetLiftPositionTask(robotHardware, robotProfile, robotProfile.hardwareSpec.liftStoneBase +
               robotProfile.hardwareSpec.liftPerStone*3 + robotProfile.hardwareSpec.liftGrabExtra, 500));
       capstonePositionList.add(new CapstonePositionTask(robotHardware, robotProfile, RobotHardware.CapPosition.CAP_DOWN));
       capstonePositionList.add(new RobotSleep(200));
       capstonePositionList.add(new CapstonePositionTask(robotHardware, robotProfile, RobotHardware.CapPosition.CAP_UP));
       capstonePositionList.add(new RobotSleep(200));
       capstonePositionList.add(new SetLiftPositionTask(robotHardware, robotProfile, robotProfile.hardwareSpec.liftOrigPos, 500));
       capstonePositionList.add(new ClampOpenCloseTask(robotHardware, robotProfile, RobotHardware.ClampPosition.CLOSE));
       capstonePositionTask = new SequentialComboTask();
       capstonePositionTask.setTaskList(capstonePositionList);

   }

    private void handleLiftAndSlide() {
        int currSliderPos = sliderEncoderCnt;
        //double percent = (currSliderPos - robotProfile.hardwareSpec.sliderOrigPos)/(robotProfile.hardwareSpec.sliderOutPos-robotProfile.hardwareSpec.sliderOrigPos);
        int currLiftPos = liftEncoderCnt;
        //double newPercent = percent;       // using percent calculation, we don't need to worry about if open is positive or negative
        // each time move 5 percent for now
        if(gamepad2.left_stick_x > 0.3) {//slide out position
            if (currLiftPos < robotProfile.hardwareSpec.liftPerStone + robotProfile.hardwareSpec.liftGrabExtra && currSliderPos < 700 ) {
                robotHardware.setLiftPosition(robotProfile.hardwareSpec.liftPerStone + robotProfile.hardwareSpec.liftGrabExtra + 10, 0.8);
            }
            else {
                //newPercent = Math.max(percent + 0.05, 1);
                //int newPos = (int)(newPercent * (robotProfile.hardwareSpec.sliderOutPos-robotProfile.hardwareSpec.sliderOrigPos)) + robotProfile.hardwareSpec.sliderOrigPos;
                int newPos = Math.min((int)(currSliderPos + 200 * (gamepad2.left_stick_x-0.3)), robotProfile.hardwareSpec.sliderOutMax);
                robotHardware.setSliderPosition(newPos);

            }
        }
        else if (gamepad2.left_stick_x < -0.3) {
            // keep moving until touch sensor is touch, then reset the 0
            if (robotHardware.sliderTouched()) {
                robotHardware.resetSlideToZero();
                sliderPosReseted = true;
            }
            else {
                int newPos = Math.max((int) (currSliderPos + 200 * (gamepad2.left_stick_x + 0.3)), (sliderPosReseted)?0:-3000);
                robotHardware.setSliderPosition(newPos);
            }
        }

        // lift go down
        if(gamepad2.left_stick_y > 0.9){
            //int newPos = currLiftPos - (int)(300 * (gamepad2.left_stick_y-0.3)/0.7);
            Logger.logFile("Fast down Curr:" + currLiftPos + " stick_y:" + gamepad2.left_stick_y);
            robotHardware.setLiftPosition(0, currLiftPos>200?0:0.8);  // free fall to 200, then push down with 0.5 power
            liftDir = LiftDirection.LIFT_DOWN;
        }
        else if (gamepad2.left_stick_y > 0.2) {
            // slow going down
            Logger.logFile("Slow down Curr:" + currLiftPos + " stick_y:" + gamepad2.left_stick_y);
            robotHardware.setLiftPosition(Math.max(currLiftPos-20, 0), 0.3);
        }
        // lift go up
        else if(gamepad2.left_stick_y < -0.9){
            Logger.logFile("Fast up Curr:" + currLiftPos + " stick_y:" + gamepad2.left_stick_y);
            robotHardware.setLiftPosition(2000, -gamepad2.left_stick_y);  // max height 11 stone
            liftDir = LiftDirection.LIFT_UP;
        }
        else if (gamepad2.left_stick_y < -0.2) {
            // slow up
            Logger.logFile("Slow up Curr:" + currLiftPos + " stick_y:" + gamepad2.left_stick_y);
            robotHardware.setLiftPosition(Math.min(currLiftPos-(int)(70*gamepad2.left_stick_y), 2000), -gamepad2.left_stick_y);
        }
        else if (liftDir!=LiftDirection.LIFT_STOP){
            // based on the previous lift position, project the next and stop
            int newPos = Math.max(Math.min(2*currLiftPos-prevLiftEncoderCnt, 2000), 0);
            Logger.logFile("Curr:" + currLiftPos + " target:" + newPos + " stick_y:" + gamepad2.left_stick_y);
            robotHardware.setLiftPosition(newPos, 0.5);
            liftDir=LiftDirection.LIFT_STOP;
        }

        if (gamepad2.dpad_up) {
            // goto the next up stone height
            int numStone = (currLiftPos + 10 - robotProfile.hardwareSpec.liftStoneBase - robotProfile.hardwareSpec.liftGrabExtra)/robotProfile.hardwareSpec.liftPerStone;
            robotHardware.setLiftPosition(robotProfile.hardwareSpec.liftStoneBase + robotProfile.hardwareSpec.liftGrabExtra +
                    (numStone+1)* robotProfile.hardwareSpec.liftPerStone);
        }
        if (gamepad2.dpad_down) {
            // goto the next down stone height
            int numStone = (currLiftPos - 10 - robotProfile.hardwareSpec.liftStoneBase - robotProfile.hardwareSpec.liftGrabExtra)/robotProfile.hardwareSpec.liftPerStone;
            robotHardware.setLiftPosition(robotProfile.hardwareSpec.liftStoneBase + robotProfile.hardwareSpec.liftGrabExtra +
                    (numStone-1)* robotProfile.hardwareSpec.liftPerStone);
        }
        if (gamepad2.dpad_right) {
            robotHardware.setSliderPosition(robotProfile.hardwareSpec.sliderOutPos);
            return;
        }
        if (gamepad2.dpad_left) {
            robotHardware.setSliderPosition(robotProfile.hardwareSpec.sliderOrigPos);
            return;
        }
        prevLiftEncoderCnt = currLiftPos;
    }
//    double forward, strafe, rotation;
    enum LiftDirection { LIFT_DOWN, LIFT_UP, LIFT_STOP }
}
