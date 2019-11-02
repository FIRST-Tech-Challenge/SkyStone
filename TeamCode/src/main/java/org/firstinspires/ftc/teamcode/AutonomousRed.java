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
        goBackward(0.5, 1760, "Going back");
        stop("Pausing");

        backServo.setPosition(0.95);
        sleep(900);

        goForward(0.59, 2250, "Going forward");
        stop("Stopping");

        // Reset Servo
        backServo.setPosition(SERVO_INITIAL_POS);
        sleep(500);
    }

}