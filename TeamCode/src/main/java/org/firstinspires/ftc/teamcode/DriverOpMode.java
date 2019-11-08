package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.io.File;

@TeleOp(name="SKYSTONE DriverOpMode", group="Main")
//@Disabledxxs
public class DriverOpMode extends OpMode {
    RobotHardware robotHardware;
    RobotNavigator navigator;
    MecanumDrive mecanumDrive;
    boolean fieldMode;
    RobotProfile robotProfile;

//    double forward, strafe, rotation;

    double gyroAngleOffset;
    double gyroCurrAngle;
    int liftEncoderCnt;
    int sliderEncoderCnt;

    @Override
    public void init() {
        try{
            robotProfile = RobotProfile.loadFromFile(new File("/sdcard/FIRST/profile.json"));
        } catch (Exception e) {
        }

        fieldMode = true;
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
            fieldMode = true;   // easy mode
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


        // grabber ready position or initial position (this will never used)
        // Driver 1: dpad right - good, dpad left - why?!
        if(gamepad2.dpad_right){
            robotHardware.rotateGrabberForPickup();
        }else if(gamepad2.dpad_left){
            robotHardware.rotateGrabberOriginPos();
        }

        // clamp open or close
        // Driver 2: a - open, b - close
        if(gamepad2.a){
            robotHardware.setClampPosition(RobotHardware.ClampPosition.OPEN);
        }else if (gamepad2.b){
            robotHardware.setClampPosition(RobotHardware.ClampPosition.CLOSE);
        }

        // slide movement
        // Driver 2: right stick left or right
        handleSlide();

        // Lift movement


        // Driver 2
        // manual up/down: left stick up/down
        // index move: keypad up/down
        handleLift();

        telemetry.addData("Field Mode", fieldMode);
        telemetry.addData("Gyro", gyroCurrAngle);

        telemetry.addData("Lift encoder", liftEncoderCnt);
        telemetry.addData("Slider encoder", sliderEncoderCnt);
   }

   @Override
    public void stop() {
        // open the clamp to relief the grabber servo
       robotHardware.setClampPosition(RobotHardware.ClampPosition.OPEN);
    }

   private void handleMovement() {
       double power = Math.hypot(gamepad1.left_stick_x, -gamepad1.left_stick_y);
       double moveAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI/4;
       if (fieldMode) {
           moveAngle += Math.toRadians(gyroCurrAngle - gyroAngleOffset);
       }
       robotHardware.mecanumDrive(power, moveAngle, gamepad1.right_stick_x);
   }

   private void handleSlide() {
       int currSliderPos = robotHardware.sliderMotor.getCurrentPosition();
       // use 1.0 * xxx to force into a double calculation, otherwise always return 0.0
       double percent = (1.0*currSliderPos - robotProfile.hardwareSpec.sliderOrigPos)/(robotProfile.hardwareSpec.sliderOutPos-robotProfile.hardwareSpec.sliderOrigPos);
       double newPercent;       // using percent calculation, we don't need to worry about if open is positive or negative
       // each time move 5 percent for now
       if(gamepad2.right_stick_x > 0.5) {               //slide out position
           newPercent = Math.min(percent + 0.05, 1);
       }
       else if (gamepad2.right_stick_x < -0.5) {
           newPercent = Math.max(percent - 0.05, 0);
       }
       else {
           return;  // stick around middle, do nothing
       }
       int newPos = (int)(newPercent * (robotProfile.hardwareSpec.sliderOutPos-robotProfile.hardwareSpec.sliderOrigPos)) + robotProfile.hardwareSpec.sliderOrigPos;
       robotHardware.setSliderPosition(newPos);
   }

   private void handleLift() {
       int currLiftPos = robotHardware.liftMotor.getCurrentPosition();
       if(gamepad2.left_stick_y > 0.5){
           robotHardware.setLiftPosition(currLiftPos - 30);  // max height 11 stone
       }
       else if(gamepad2.left_stick_y < -0.5){
           robotHardware.setLiftPosition(currLiftPos + 30);  // max height 11 stone
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

   }
}