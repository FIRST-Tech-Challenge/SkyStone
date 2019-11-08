package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name="Autonomous-Blue", group="Linear Opmode")
public class AutonomousBlue extends Movement
{
    protected final Double SERVO_INITIAL_POS = 0.4;

    public void runOpModeImpl() {
        backServo.setPosition(SERVO_INITIAL_POS);

        waitForStart();

        // Move Backward
        goBackward(0.5, 1700, "Going back");
        stop("Pausing");

        //Servo down

        backServo.setPosition(1);
        sleep(900);

        //Move forward at angle towards the left

        leftfront.setPower(0.37);
        leftback.setPower(0.37);
        rightfront.setPower(0.34);
        rightback.setPower(0.34);
        sleep(2950);

        updateTelemetryMessage("Going forward at angle towards left");
        stop("Stopping");

        // Servo up
        backServo.setPosition(SERVO_INITIAL_POS);
        sleep(500);

        //Move right

        goRight(0.8, 2250, "Going right");
        stop("Stopping");


    }}
