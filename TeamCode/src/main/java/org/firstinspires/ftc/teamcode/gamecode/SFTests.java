package org.firstinspires.ftc.teamcode.gamecode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodesupport.AutoOpMode;
import org.firstinspires.ftc.teamcode.robots.Armstrong;
import org.firstinspires.ftc.teamcode.robots.Robot;

@Autonomous
public class SFTests extends AutoOpMode {
    @Override
    public void runOp() throws InterruptedException {
        Robot robot = new Robot();

        waitForStart();
        robot.SfIMUTurnLNoIn(180, 0.3);
    }
}
