package org.firstinspires.ftc.teamcode;

/**
 * Created by student on 10/26/2017.
 */

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "BlueBuildToWall", group = "Autonomous")
public class BlueBuildToWall extends Autonomous{

    @Override
    public void runPath(){
        robot.hookOne.setPosition(0.2);
        //robot.hookTwo.setPosition(0);

        move(20, -1, 1);
        sleep(100);
        move(34, 1, 0);
        sleep(500);
        robot.hookOne.setPosition(01.5);
        //robot.hookTwo.setPosition(0.8);
        sleep(1500);
        move(48, -1, 0); // was 45*1.5
        sleep(500);
        robot.hookOne.setPosition(0.2);
        //robot.hookTwo.setPosition(0);
        sleep(500);
        move(53, 1, 1);
    }
}
