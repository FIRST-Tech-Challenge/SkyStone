package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Maccabot;

@Autonomous(name="Auto Outside", group="Autonomous")
@Disabled
public class auto_competition1_outside extends LinearOpMode {
    private Maccabot robot;


    public void runOpMode() throws InterruptedException {
        robot = new Maccabot(this);
        robot.initializeRobot();

        waitForStart();

        while(opModeIsActive()){
            robot.auto_forward(800, 0.65);
            sleep(1000);
            robot.auto_straferight(800, 0.65);
            sleep(800);
            stop();
        }

    }
}
