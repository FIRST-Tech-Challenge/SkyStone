package org.firstinspires.ftc.teamcode.gamecode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodesupport.AutoOpMode;
import org.firstinspires.ftc.teamcode.robots.Joules;

@Autonomous
public class JoulesUpRighttPark extends AutoOpMode {
    public void runOp() throws InterruptedException {
        Joules joules = new Joules();

        telemetry.addData("Status", "initialized");
        waitForStart();
        joules.DriveBackward(0.5);
        sleep(1000);
        joules.Stop();
        joules.StrafeLeft(0.5);
        sleep(2500);
        joules.Stop();


    }


}
