package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="FoundationRed", group="Linear Opmode")
//Disabled
public class FoundationRed extends Movement {
    private ElapsedTime runtime = new ElapsedTime();

    //public void runOpMode() {
    @Override public void runOpModeImpl() {

        waitForStart();
        runtime.reset();

        //wait 22 seconds
        sleep(22000);

        //start with back servos up
        backServosUp();

        // strafe right to align with Construction Site
        goRight( 1, 550);

        //drive backward
        goBackward(0.5, 1800);

        //back servos move down
        backServosDown();

        stopWithSleep("motors stopped",800);

        //drive forward
        goForward(0.5,1900);

        //back servos move up
        backServosUp();

        //stop motors
        stopWithSleep("motors stopped",300);

        goForward(1, 10);

        //strafe to the left (park under the bridge)
        goLeft(1,1250);

        goBackward(0.5, 250);

        goLeft(1,800);

        telemetry.addData("Status", "Stop Program");
        telemetry.update();



    }
}


