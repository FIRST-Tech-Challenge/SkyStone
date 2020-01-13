package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name = "servoTest", group = "test")
public class servoPositioner extends LinearOpMode {
    Servo mGrab;

    @Override
    public void runOpMode() throws InterruptedException {
        mGrab = hardwareMap.get(Servo.class, "middleGrab");

        mGrab.setPosition(0.9);

        waitForStart();
        while(opModeIsActive()){
            telemetry.addData("Position of Servo: ", mGrab.getPosition());
            telemetry.update();

            if(gamepad1.a){
                mGrab.setPosition(mGrab.getPosition()+0.05);
            }
            if(gamepad1.y){
                mGrab.setPosition(mGrab.getPosition()-0.05);
            }
        }
    }
}
