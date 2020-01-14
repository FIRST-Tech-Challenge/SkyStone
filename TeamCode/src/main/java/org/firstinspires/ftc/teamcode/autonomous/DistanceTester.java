package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Distance Tester")
public class DistanceTester extends RobotController {

    public void runOpMode() {
        waitForStart();

        doAction("forward", 5);

        stop();
    }
}
