package org.firstinspires.ftc.teamcode;

/**
 * Created by student on 10/26/2017.
 */

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "RedBuildToWall", group = "Autonomous")
public class RedBuildToWall extends Autonomous {
    @Override
    public void runPath() {
        robot.hookOne.setPosition(0);
        robot.hookTwo.setPosition(0);

        move(27*1.5, 1, 1); // was 14*1.5
        sleep(100);
        move(34*1.5, 1, 0);
        sleep(500);
        robot.hookOne.setPosition(0.8);
        robot.hookTwo.setPosition(0.8);
        sleep(1500);
        move(45*1.5, -1, 0);
        sleep(500);
        robot.hookOne.setPosition(0);
        robot.hookTwo.setPosition(0);
        sleep(500);
        move(57*1.5, -1, 1);
    }
}