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
        joules.CapUp();
        joules.TapeMeasurePush();
        joules.FoundationDrop();
        telemetry.addData("Status", "Initialized");
    }

    public int HoldOntoStone = 0;
    private Boolean pushed = Boolean.FALSE;
    private Boolean slow = Boolean.FALSE;
    int a = 0;
    int b = 0;


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

        if (joy1.buttonA()){
            joules.FoundationTowerMove();
            telemetry.addData("task", "failed successfully");

        }


        if (joy2.leftTrigger()) {
            joules.DaffyGrab();
            telemetry.addData("Daffy", "grabbed");

        } else if (joy2.leftBumper()) {
            joules.DaffyUp();
            telemetry.addData("Daffy", "up");
        }
        else if (pushed == Boolean.TRUE){
            joules.DaffyGrab();
            telemetry.addData("","Daffy Grab");
        }
        else {
            joules.DaffyStop();
        }



        if (joy2.buttonLeft()) {
            if (a == 0) {
                if (pushed == Boolean.FALSE) {
                    pushed = Boolean.TRUE;
                    telemetry.addData("boolean", "True");
                    a = 1;

                }
                else if (pushed == Boolean.TRUE) {
                    pushed = Boolean.FALSE;
                    telemetry.addData("boolean", "False");
                    a = 1;
                }
            }
        }
        else if (!joy2.buttonLeft()){
            a = 0;
        }


        if (slow == Boolean.FALSE){
            if (joy2.rightTrigger()) {
                joules.SlidesDown();
            } else if (joy2.rightBumper()) {
                 joules.SlidesUp();
            } else {
              joules.SlidesStop();
            }
        }

        else if (slow == Boolean.TRUE){
            if (joy2.rightTrigger()) {
                joules.SlidesDownSlow();
            } else if (joy2.rightBumper()) {
                joules.SlidesUpSlow();
            } else {
                joules.SlidesStop();
            }
        }

        if (joy2.buttonY()) {
            joules.FoundationGrab();

        } else if (joy2.buttonA()) {
            joules.FoundationDrop();

        }

        if (joy2.buttonRight()){
            if (b == 0) {
                if (slow == Boolean.FALSE) {
                    slow = Boolean.TRUE;
                    telemetry.addData("slow", "True");
                    b = 1;

                }
                else if (slow == Boolean.TRUE) {
                    slow = Boolean.FALSE;
                    telemetry.addData("slow", "False");
                    b = 1;
                }
            }
        }
        else if (!joy2.buttonRight()){
            b = 0;
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
