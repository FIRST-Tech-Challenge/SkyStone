package org.firstinspires.ftc.teamcode;

/**
 * Created by student on 10/26/2017.
 */

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "RedBuildToBridge", group = "Autonomous")
public class RedBuildToBridge extends Autonomous {
    @Override
    public void runPath() {
        move(27, 1, 1); // was 14*1.5
        sleep(100);
        move(34, 1, 0);
        sleep(500);
        robot.hookOne.setPosition(0.8);
        robot.hookTwo.setPosition(0.8);
        sleep(1500);
        move(45, -1, 0);
        sleep(500);
        robot.hookOne.setPosition(0);
        robot.hookTwo.setPosition(0);
        sleep(500);
        move(57/2, -1, 1);
        move(45, -1, 0);
        move(57/2, -1, 1);
    }
}
