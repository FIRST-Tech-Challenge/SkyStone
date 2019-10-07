package org.firstinspires.ftc.teamcode;

import static java.lang.Math.sqrt;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous

public class Octo358AutoTest extends Octo358AutoCentral {
    public void runOpMode() {

        configureMotors();
        waitForStart();

        drive(0.6, 20);

        strafeLeft(0.6, 20);

        turnTo(0.6, -315);

        drive(0.7, -sqrt(2) * 20);

        rotate(0.8, 540);

        strafeLeft(0.6, -20);

    }
}
