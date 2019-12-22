package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.io.File;

/**
 * This OpMod is used to tune PID parameters.
 * Left joystick to set angle
 * Left keypad to set distance
 * Press X to decrease power, Y to increase power
 * Press A to set target position based on angle and distance
 * Press B to execute the move
 * End of the move, display error X, Y in centimeter
 */
@TeleOp(name="Interacive PID Test", group="Test")
//@Disabled
public class InteractivePIDTest extends OpMode {
    RobotHardware robotHardware;
    RobotNavigator navigator;
    RobotProfile robotProfile;
    RobotControl currentTask = null;
    double angle;  // 15 degree increment
    double power;
    long distance;   // cm
    RobotPosition endPos;
    boolean dpadUp, dpadDown, buttonB, buttonX, buttonY;
    RobotPosition lastPosition, targetPosition;
    double errorX, errorY, errorHeading;


    @Override
    public void init() {
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
        lastPosition = new RobotPosition(0,0,0);
        targetPosition = new RobotPosition(0,0,0);

        distance = 150;
        dpadUp = false;
        dpadDown = false;
        buttonX = buttonY = false;
        power = 0.8;
        errorX = errorY = errorHeading = 0;
    }

    @Override
    public void loop() {
        robotHardware.getBulkData1();
        robotHardware.getBulkData2();
        navigator.updateEncoderPos(robotHardware.getEncoderCounts(RobotHardware.EncoderType.LEFT),
                robotHardware.getEncoderCounts(RobotHardware.EncoderType.RIGHT),
                robotHardware.getEncoderCounts(RobotHardware.EncoderType.HORIZONTAL));


        double moveAngle = Math.PI/2 - Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x);
        angle = Math.round(moveAngle * 180/Math.PI/15) * 15;
        if (gamepad1.x) {
            if (!buttonX) {
                power = Math.max(0.3, power-0.1);
                buttonX = true;
            }
        }
        else {
            buttonX = false;
        }
        if (gamepad1.y) {
            if (!buttonY) {
                power = Math.min(1.0, power+0.1);
                buttonY = true;
            }
        }
        else {
            buttonY = false;
        }
        if (gamepad1.dpad_up) {
            if (!dpadUp) {
                distance = Math.min(distance + 10, 250);
            }
            dpadUp = true;
        }
        else {
            dpadUp = false;
        }
        if (gamepad1.dpad_down) {
            if (!dpadDown) {
                distance = Math.max(10, distance-10);
            }
            dpadDown = true;
        }
        else {
            dpadDown = false;
        }
        if (gamepad1.b) {
            if (currentTask==null) {
                createTestMove();
            }
        }
        if (gamepad1.a) {
            targetPosition.setX(Math.round(navigator.getWorldX()+Math.sin(angle/180*Math.PI)*distance));
            targetPosition.setY(Math.round(navigator.getWorldY()+Math.cos(angle/180*Math.PI)*distance));
        }
        if(currentTask != null) {
            currentTask.execute();
            if(currentTask.isDone()) {
                currentTask.cleanUp();
                errorX = navigator.getWorldX() - targetPosition.getX();
                errorY = navigator.getWorldY() - targetPosition.getY();
                errorHeading= navigator.getHeading() - targetPosition.getHeading();
                Logger.flushToFile();
                navigator.setInitPosition(0, 0, 0);
                lastPosition.setX(navigator.getWorldX());
                lastPosition.setY(navigator.getWorldY());
                currentTask = null;
            }
        }
        telemetry.addData("Angle:", angle);
        telemetry.addData("Distance:", distance);
        telemetry.addData("Power:", power);
        telemetry.addData("TargetX:", targetPosition.getX());
        telemetry.addData("TargetY:", targetPosition.getY());
        telemetry.addData("Err X:", "%.2f", errorX);
        telemetry.addData("Err Y:", "%.2f",errorY);
        telemetry.addData("Heading:", "%.1f", 180*errorHeading/Math.PI);
   }

   private void createTestMove() {
        // reload profile for latest PID parameter
       try {
           robotProfile = RobotProfile.loadFromFile(new File("/sdcard/FIRST/profile.json"));
       }
       catch (Exception ex) {
       }
       PIDMecanumMoveTask move = new PIDMecanumMoveTask(robotHardware, robotProfile, navigator);
       move.setPath(lastPosition, targetPosition);
       move.setPower(power);
       currentTask = move;
       currentTask.prepare();
   }
}