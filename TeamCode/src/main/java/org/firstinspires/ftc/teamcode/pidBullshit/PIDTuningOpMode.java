package org.firstinspires.ftc.teamcode.pidBullshit;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Maccabot;

@Autonomous(name="Why da hecc does i do this")
//@Disabled
public class PIDTuningOpMode extends OpMode {

    Maccabot robot;

    @Override
    public void init() {
        robot = new Maccabot(this);
        robot.initializeRobot();
    }

    @Override
    public void loop() {
        robot.drive(1, 1, 1, 1);
        try {
            wait(1000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        robot.drive(0,0,0,0);
    }
}
