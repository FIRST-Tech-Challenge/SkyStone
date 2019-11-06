package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name="Autonomous-Red", group="Linear Opmode")
public class AutonomousRed extends Movement
{
    protected final Double SERVO_INITIAL_POS = 0.4;

    public void runOpModeImpl() {
        backServo.setPosition(SERVO_INITIAL_POS);

        waitForStart();

        // Move Forward
        goBackward(0.5, 1700, "Going back");
        stop("Pausing");

        backServo.setPosition(1);
        sleep(900);

        goForward(0.35, 2550, "Going forward");
        stop("Stopping");

        // Reset Servo
        backServo.setPosition(SERVO_INITIAL_POS);
        sleep(500);

        goLeft(0.8,2000, "Going left");
        stop("Stopping");

        goForward(0.8, 200, "Going left");
        stop("Stopping");


    }

}