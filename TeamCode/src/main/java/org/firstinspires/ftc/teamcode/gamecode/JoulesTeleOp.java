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
        joules.TapeMeasurePush();
        joules.FoundationDrop();
        telemetry.addData("Status", "Initialized");
    }

    public void loopOpMode() {
        if (joy1.leftTrigger()) {
            float pivot = gamepad1.right_stick_y / 2;
            float horizontal = -gamepad1.right_stick_x / 2;
            float vertical = gamepad1.left_stick_x / 2;
            joules.FrontRight.setPower(-pivot + (vertical + horizontal));
            joules.BackRight.setPower(-pivot + (vertical - horizontal));
            joules.FrontLeft.setPower(pivot + (vertical + horizontal));
            joules.BackLeft.setPower(pivot + (vertical - horizontal));
        } else {

            float pivot = gamepad1.right_stick_y;
            float horizontal = -gamepad1.right_stick_x;
            float vertical = gamepad1.left_stick_x;
            joules.FrontRight.setPower(-pivot + (vertical + horizontal));
            joules.BackRight.setPower(-pivot + (vertical - horizontal));
            joules.FrontLeft.setPower(pivot + (vertical + horizontal));
            joules.BackLeft.setPower(pivot + (vertical - horizontal));
        }


        if (joy2.buttonX()) {
            joules.StoneUp();
        }
        if (joy2.buttonB()) {
            joules.StoneDown();
        }

        if (joy2.buttonUp()) {
            joules.CapDown();
        } else if (joy2.buttonDown()) {
            joules.CapUp();
        }

        if (joy1.buttonX()) {
            joules.TapeMeasurePush();
            telemetry.addData("Push", "kj");
        } else if (joy1.buttonB()) {
            joules.TapeMeasureSpring();
            telemetry.addData("spring", "kj");

        }

        if (joy1.buttonY()) {
            telemetry.addData("position", joules.ScissorValues());
            joules.ScissorLiftUp();
        } else if (joy1.buttonA()) {
            telemetry.addData("position", joules.ScissorValues());
            joules.ScissorLiftDown();
            telemetry.addData("position 2", joules.ScissorLift.getPosition());
        } else if (joy1.leftTrigger()) {
            telemetry.addData("position", joules.ScissorValues());
            joules.ScissorLiftEst();
            telemetry.addData("position 2", joules.ScissorLift.getPosition());

        }


        if (joy2.leftTrigger()) {
            joules.DaffyGrab();

        } else if (joy2.leftBumper()) {
            joules.DaffyUp();
        }

        if (joy2.rightTrigger()) {
            joules.SlidesDown();
        } else if (joy2.rightBumper()) {
            joules.SlidesUp();
        } else {
            joules.SlidesStop();
        }

        if (joy2.buttonY()) {
            joules.FoundationGrab();

        } else if (joy2.buttonA()) {
            joules.FoundationDrop();

        }




        if (joy1.rightBumper()){
            clearTimer(2);
            if (getMilliSeconds(2)%1000 == 0){
                joules.ScissorLift.setPosition(joules.ScissorLift.getPosition() + ((0.8/5.5))/200);

            }

        }

        if (joy1.rightTrigger()){
            clearTimer(3);
            if (getMilliSeconds(3)%1000 == 0){
                joules.ScissorLift.setPosition(joules.ScissorLift.getPosition() - ((0.8/4.5))/200);

            }

        }


    }
}
