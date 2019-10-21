package org.firstinspires.ftc.teamcode.gamecode;


import com.qualcomm.hardware.lynx.LynxEmbeddedIMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodesupport.AutoOpMode;
import org.firstinspires.ftc.teamcode.robots.Armstrong;
import org.firstinspires.ftc.teamcode.robots.Robot;

@Autonomous
public class turn extends AutoOpMode {
    @Override
    public void runOp() throws InterruptedException {
        Robot robot = new Robot();


        waitForStart();
        while (opModeIsActive()) {
            robot.slantforward(-0.5, 0);
        }
    }
}
