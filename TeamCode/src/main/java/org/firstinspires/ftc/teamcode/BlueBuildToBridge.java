package org.firstinspires.ftc.teamcode;

/**
 * Created by student on 10/26/2017.
 */

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "BlueBuildToBridge", group = "Autonomous")
public class BlueBuildToBridge extends Autonomous {
    @Override
    public void runPath() {
        robot.hookOne.setPosition(0.2);
        robot.hookTwo.setPosition(0.2);

        move(14, -1, 1);
        sleep(100);
        move(34, 1, 0);
        sleep(500);
        robot.hookOne.setPosition(1.5);
        robot.hookTwo.setPosition(1.5);
        sleep(1500);
        move(48, -1, 0); // was 45
        sleep(500);
        robot.hookOne.setPosition(0.2);
        robot.hookTwo.setPosition(0.2);
        sleep(500);
        move(57/2, 1, 1);
        move(48, 1, 0);
        move(57/2, 1, 1);
    }
}
