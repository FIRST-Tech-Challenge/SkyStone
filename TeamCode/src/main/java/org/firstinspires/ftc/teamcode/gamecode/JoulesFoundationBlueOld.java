package org.firstinspires.ftc.teamcode.gamecode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodesupport.AutoOpMode;
import org.firstinspires.ftc.teamcode.robots.Joules;

public class JoulesFoundationBlueOld extends AutoOpMode {
    public void runOp() throws InterruptedException {
        Joules joules = new Joules();

        telemetry.addData("Status", "initialized");
        waitForStart();


        joules.DriveBackward(0.5);
        sleep(1500);
        joules.Stop();

        joules.TurnRight(0.5);
        sleep(100);
        joules.Stop();

        joules.DriveBackward(0.5);
        sleep(100);
        joules.Stop();

        joules.FoundationGrab();
        sleep(2000);
        joules.Stop();

        joules.StrafeRight(0.5);
        sleep(2500);
        joules.Stop();

        joules.DriveForward(0.5);
        sleep(2500);
        joules.Stop();

        joules.StrafeLeft(0.5);
        sleep(500);
        joules.Stop();

//        joules.TurnLeft(0.5);
//        sleep(1000);
//        joules.Stop();

        joules.FoundationDrop();
        sleep(2000);

        joules.DriveForward(0.5);
        sleep(500);
        joules.Stop();

        joules.StrafeLeft(0.5);
        sleep(4500);
        joules.Stop();


    }


}
