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

        if (gamepad1.left_bumper) {
            float vertical = -gamepad1.right_stick_y/2;
            float horizontal = gamepad1.right_stick_x/2;
            float pivot = gamepad1.left_stick_x/2;
            joules.FrontRight.setPower(-pivot + (vertical - horizontal));
            joules.BackRight.setPower(-pivot + vertical + horizontal);
            joules.FrontLeft.setPower(pivot + vertical + horizontal);
            joules.BackLeft.setPower(pivot + (vertical - horizontal));

        }
        else{
            float vertical = -gamepad1.right_stick_y;
            float horizontal = gamepad1.right_stick_x;
            float pivot = gamepad1.left_stick_x;
            joules.FrontRight.setPower(-pivot + vertical - horizontal);
            joules.BackRight.setPower(-pivot + vertical + horizontal);
            joules.FrontLeft.setPower(pivot + vertical + horizontal);
            joules.BackLeft.setPower(pivot + vertical - horizontal);
        }

        if (joy2.buttonX()){
            joules.StoneUp();
        }
        if (joy2.buttonB()){
            joules.StoneDown();
        }

        if (joy2.buttonUp()){
            joules.CapDown();
        }
        else if (joy2.buttonDown()){
            joules.CapUp();
        }

        if (joy2.leftTrigger()){
            joules.DaffyGrab();

        }
        else if (joy2.leftBumper()){
            joules.DaffyUp();
        }

        if (joy2.rightTrigger()){
            joules.SlidesDown();
        }
        else if (joy2.rightBumper()){
            joules.SlidesUp();
        }
        else{
            joules.SlidesStop();
        }

        if (joy2.buttonY()){
            joules.FoundationGrab();

        }
        else if (joy2.buttonA()){
            joules.FoundationDrop();

        }


    }
}
