package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Red Loading Zone")
public class RedLoadingZoneAutonomous extends RobotController {

    public void runOpMode() {
        waitForStart();

        moveForward(-1, 2);
        straifLeft(5, 10);

        stop();
    }


}
