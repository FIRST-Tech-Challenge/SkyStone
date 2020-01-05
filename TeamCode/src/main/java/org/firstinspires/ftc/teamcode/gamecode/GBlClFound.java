package org.firstinspires.ftc.teamcode.gamecode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodesupport.AutoOpMode;
import org.firstinspires.ftc.teamcode.robots.Joules;

@Autonomous
public class GBlClFound extends AutoOpMode {
    public void runOp() throws InterruptedException {
        Joules joules = new Joules();

        telemetry.addData("Status", "initialized");
        waitForStart();

        /*joules.SlidesUp();
        sleep(1000);
        joules.SlidesStop();*/

        joules.StrafeLeft(0.5);
        sleep(650);
        joules.Stop();

        joules.DriveForward(0.5);
        sleep(900);
        joules.Stop();

        joules.FoundationGrab();
        sleep(2000);
        joules.Stop();

        joules.DriveBackward(0.5);
        sleep(800);
        joules.Stop();

        joules.FoundationDrop();
        sleep(2000);
        joules.Stop();



    }

}
