package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Autonomous-Red", group="Linear Opmode")
public class AutonomousRed extends Movement
{
    AutonomousRed() {
        super();
    }
    public void runOpMode() {
        waitForStart();

        // Move Forward
        goForward(1.0, 1760, "Moving goForward");
        // stop("Pausing");

        // TODO: Roatae servo motor to bring down the arm

        // Move goBackward
        goBackward(0.5, 2300, "Moving backwards");
        // stop("Pausing");

        //

    }

}