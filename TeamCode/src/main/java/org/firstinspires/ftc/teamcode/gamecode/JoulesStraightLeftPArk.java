package org.firstinspires.ftc.teamcode.gamecode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodesupport.AutoOpMode;
import org.firstinspires.ftc.teamcode.robots.Armstrong;
import org.firstinspires.ftc.teamcode.robots.Joules;

@Autonomous
public class JoulesStraightLeftPArk extends AutoOpMode {
    public void runOp() throws InterruptedException {
        Joules joules = new Joules();

        telemetry.addData("Status", "initialized");
        waitForStart();
        joules.DriveForward(0.5);
        sleep(200);
        joules.Stop();
        joules.StrafeRight(0.5);
        sleep(2500);
        joules.Stop();


    }


}
