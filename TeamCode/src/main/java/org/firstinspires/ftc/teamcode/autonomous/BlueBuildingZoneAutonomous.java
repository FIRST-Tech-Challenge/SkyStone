package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Blue Building Zone")
public class BlueBuildingZoneAutonomous extends RobotController {

    public void runOpMode() {
        waitForStart();

        doAction("left", 5);
        doAction("forward", 5);
        //clip foundation
        doAction("rotate", -5);
        doAction("backward", 2);
        doAction("rotate", -5);
        doAction("forward", 2);
        //release clip
        doAction("backward", 7);

        stop();
    }

}
