package org.firstinspires.ftc.teamcode.gamecode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.opmodesupport.AutoOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.RC;
import org.firstinspires.ftc.teamcode.newhardware.Motor;
import org.firstinspires.ftc.teamcode.opmodesupport.TeleOpMode;
import org.firstinspires.ftc.teamcode.robots.Armstrong;
@TeleOp


public class ArmstrongTeleop extends TeleOpMode {
    Armstrong armstrong;
    @Override
    public void initialize() {
        armstrong = new Armstrong();
        telemetry.addData("Status", "Initialized");
    }


    @Override
    public void loopOpMode() {


        if (joy1.rightTrigger()){
            telemetry.addData("Status", "Slow Mode");
            armstrong.driveL(-gamepad1.left_stick_y/4);
            armstrong.driveR(-gamepad1.right_stick_y/4);
            telemetry.addData("slow left", -gamepad1.left_stick_y);
            telemetry.addData("Slow right", -gamepad1.right_stick_y);

        }

        if (joy1.leftTrigger()){
            telemetry.addData("Status", "Reverse");
            armstrong.driveR(gamepad1.left_stick_y*0.9);
            armstrong.driveL(gamepad1.right_stick_y*0.9);
        }

        if (!joy1.leftTrigger()){
            if (!joy1.rightTrigger()) {
                telemetry.addData("Status", "Normal Driving");
                armstrong.driveL(-gamepad1.left_stick_y);
                armstrong.driveR(-gamepad1.right_stick_y);
            } //if !rightTrigger
        } // If !leftTrigger


        if (joy2.buttonUp()) {
            armstrong.RightSample();
        }
        if (joy2.buttonDown()){
            armstrong.RightWingStore();
        }
        if (joy2.buttonLeft()){
            armstrong.LeftSample();
        }
        if (joy2.buttonRight()){
            armstrong.LeftWingStore();
        }

        //linear wall servo
        if (joy1.buttonA()){
            armstrong.wallIn();
        }
        if (joy1.buttonX()){
            armstrong.wallPush();
        }


        //marker
        if (joy2.buttonB()){
            armstrong.markUp();
        }
        if (joy2.buttonX()){
            armstrong.markDown();
        }




        //lifter
        if (joy2.rightBumper()) {
            armstrong.lifterUp();
        }
        else if(joy2.rightTrigger()){
            armstrong.lifterDown();
        }
        else {
            armstrong.lifterStop();
        }


        //latcher
        if (joy2.leftBumper()) {
            armstrong.setLatch();
            telemetry.addData("latch is UP", "set latch");

        }
        else if (joy2.leftTrigger()){
            armstrong.unlatch();
            telemetry.addData("latch is DOWN", "unlatch");

        }

        if(joy2.buttonY()) {
            armstrong.collectServoLeftUp();
            armstrong.collectServoRightUp();
            RC.t.addData("servos up");
        }
        else if (joy2.buttonA()){
            armstrong.collectServoLeftDown();
            armstrong.collectServoRightDown();
            RC.t.addData("servos down");
        }
        else{
            armstrong.collectServoLeftStop();
            armstrong.collectServoRightStop();
            RC.t.addData("stop");
        }

        armstrong.CollectMotor.setPower(gamepad2.left_stick_y/5);


        //to make sure the robot doesn't flip

        if(System.currentTimeMillis()%1000 == 0);{
            armstrong.MOTOR_SPEED_PAST = armstrong.MOTOR_SPEED;
            armstrong.MOTOR_SPEED = gamepad1.left_stick_y;
            armstrong.AVR_MOTOR_DIFF = Math.abs(armstrong.MOTOR_SPEED - armstrong.MOTOR_SPEED_PAST);
        }

        if(armstrong.AVR_MOTOR_DIFF > 0.7){
            armstrong.driveR((armstrong.MOTOR_SPEED + armstrong.MOTOR_SPEED_PAST)/2);
        }



    }
}

