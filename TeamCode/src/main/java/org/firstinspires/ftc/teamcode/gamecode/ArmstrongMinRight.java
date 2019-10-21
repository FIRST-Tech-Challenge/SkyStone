package org.firstinspires.ftc.teamcode.gamecode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodesupport.AutoOpMode;
import org.firstinspires.ftc.teamcode.robots.Armstrong;
@Autonomous
public class ArmstrongMinRight extends AutoOpMode {

    @Override


    public void runOp() throws InterruptedException {
        //init phase
        Armstrong armstrong = new Armstrong();
        armstrong.markUp();
        telemetry.addData("Status", "Initialized");
        waitForStart();
        //this is after the driver presses play
//        armstrong.lifterUp();
//        armstrong.collectServoLeftSlow();
//        armstrong.collectServoRightSlow();
//        sleep(10790);
//        armstrong.lifterStop();
//        armstrong.collectServoLeftStop();
//        armstrong.collectServoRightStop();
//
//        armstrong.unlatch();
//        armstrong.armup();
//        sleep(250);
//        armstrong.armstop();

        armstrong.forward(0.3);
        sleep(900);
        armstrong.stop();
        armstrong.RightSample();
        sleep(100);

        armstrong.forward(0.3);
        sleep(1200);
        armstrong.stop();

        armstrong.RightWingStore();
        sleep(100);
        armstrong.forward(0.3);
        sleep(1000);
        armstrong.stop();


        armstrong.markWallDown();
        sleep(1000);
        telemetry.addData("Status", "Wall and Marker Down");
    }
}


