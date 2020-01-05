package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Auto Inside", group="Autonomous")
public class auto_competition1_inside extends LinearOpMode {
    private Maccabot robot;


    public void runOpMode() throws InterruptedException {
        robot = new Maccabot(this);
        robot.initializeRobot();

        waitForStart();

        while(opModeIsActive()){
            robot.auto_forward(800, 0.65);
            sleep(1000);
            stop();
        }

    }
}
