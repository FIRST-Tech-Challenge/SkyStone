package org.firstinspires.ftc.teamcode.gamecode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.sun.tools.javac.util.Position;

import org.firstinspires.ftc.teamcode.RC;
import org.firstinspires.ftc.teamcode.opmodesupport.TeleOpMode;
import org.firstinspires.ftc.teamcode.robots.Joules;

@TeleOp
public class JoulesTeleOp extends TeleOpMode {
    Joules joules;

    @Override
    public void initialize() {
        joules = new Joules();
        joules.CapDown();
        telemetry.addData("Status", "Initialized");
    }


    public void loopOpMode() {
        if (gamepad1.left_bumper){
            joules.DriveBackward(gamepad1.left_stick_y*2);
            joules.StrafeRight(gamepad1.left_stick_x*2);
            joules.TurnLeft(gamepad1.right_stick_x*2);
        }
        else {
            if (gamepad1.left_stick_y > 0) {
                joules.DriveBackward(gamepad1.left_stick_y * 2);
            } else if (gamepad1.left_stick_x > 0) {
                joules.StrafeRight(gamepad1.left_stick_x * 2);
            } else if (gamepad1.right_stick_x > 0) {
                joules.TurnLeft(gamepad1.right_stick_x * 2);
            } else if (gamepad1.left_stick_y < 0) {
                joules.DriveForward(gamepad1.left_stick_y * -2);
            } else if (gamepad1.left_stick_x < 0) {
                joules.StrafeLeft(gamepad1.left_stick_x * -2);
            } else if (gamepad1.right_stick_x < 0) {
                joules.TurnRight(gamepad1.right_stick_x * -2);
            } else {
                joules.Stop();
            }
        }


        /*joules.DriveBackward(gamepad1.left_stick_y*2);
        joules.StrafeRight(gamepad1.left_stick_x*2);
        joules.TurnLeft(gamepad1.right_stick_x*2);*/


        if (joy2.buttonX()){
            joules.CapUp();
        }
        if (joy2.buttonB()){
            joules.CapDown();
        }

        if (joy2.leftTrigger()){
            joules.FoundationGrab();
        }
        else if (joy2.leftBumper()){
            joules.FoundationDrop();
        }

        if (joy2.rightTrigger()){
            joules.StoneDown();
        }
        else if (joy2.rightBumper()){
            joules.StoneUp();
        }
        else{
            joules.StoneStop();
        }

    }
}
