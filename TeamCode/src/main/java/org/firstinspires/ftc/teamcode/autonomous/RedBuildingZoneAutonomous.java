package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Red Building Zone")
public class RedBuildingZoneAutonomous extends RobotController {

    // {"forward", "backward", "left", "right", "rotate"}

    public void runOpMode() {
        waitForStart();

        doAction("right", 5);
        doAction("forward", 5);
        //clip foundation
        doAction("rotate", 5);
        doAction("backward", 2);
        doAction("rotate", 5);
        doAction("forward", 2);
        //release clip
        doAction("backward", 7);

        stop();
    }
}





