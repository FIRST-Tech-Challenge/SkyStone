package org.firstinspires.ftc.teamcode.gamecode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodesupport.AutoOpMode;
import org.firstinspires.ftc.teamcode.robots.Joules;

@Autonomous
public class JoulesRedFoundation extends AutoOpMode {
    public void runOp() throws InterruptedException {
        Joules joules = new Joules();

        telemetry.addData("Status", "initialized");
        waitForStart();

        joules.DriveForward(0.5);
        sleep(1800);
        joules.Stop();

        joules.FoundationGrab();
        sleep(2500);
        joules.Stop();

        joules.DriveBackward(0.7);
        sleep(1800);
        joules.Stop();

        joules.DriveForward(0.7);
        sleep(100);
        joules.Stop();

        joules.StrafeRight(1);
        sleep(600);
        joules.Stop();

        joules.DriveBackward(1);
        sleep(300);
        joules.Stop();

        joules.FoundationDrop();
        sleep(2000);
        joules.Stop();


        joules.DriveBackward(1);
        sleep(200);
        joules.Stop();

        joules.StrafeRight(1);
        sleep(500); //1700 is good for parking
        joules.Stop();

        joules.DriveForward(1);
        sleep(300);
        joules.Stop();

        joules.TurnRight(1);
        sleep(500);
        joules.Stop();

        joules.StrafeLeft(0.8);
        sleep(1500);
        joules.Stop();




    }


}
