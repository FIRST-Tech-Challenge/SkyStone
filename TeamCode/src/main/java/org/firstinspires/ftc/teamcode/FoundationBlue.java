package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="FoundationBlue", group="Linear Opmode")
//Disabled

public class FoundationBlue extends Movement {
    private ElapsedTime runtime = new ElapsedTime();


    @Override public void runOpModeImpl() {

        waitForStart();
        runtime.reset();

        //start with back servos up
        backServosUp();

        goLeft(1, 500);

        //drive backward
        goBackward(0.5, 1800);

        //back servos move down
        backServosDown();

        //stop motors
        stopWithSleep("motors stopped",300);

        //drive forward
        goForward(0.5,2250);

        //back servos move up
        backServosUp();

        //stop motors
        stopWithSleep("motors stopped",300);

        goForward(1,10);


        //strafe to the left (maybe park under the bridge)
        goRight(1,2250);

        telemetry.addData("Status", "Stop Program");
        telemetry.update();



    }
}

