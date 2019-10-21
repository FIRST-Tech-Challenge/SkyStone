package org.firstinspires.ftc.teamcode.gamecode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.opmodesupport.AutoOpMode;
import org.firstinspires.ftc.teamcode.robots.Armstrong;

@Autonomous
@Disabled
public class ArmstrongLandDumpMiddletheirsdos extends AutoOpMode {
    @Override

    public void runOp() throws InterruptedException {
        //init phase

        Armstrong armstrong = new Armstrong();
        armstrong.markUp();
        telemetry.addData("Status", "Initialized");
        waitForStart();
            //this is after the driver presses play
        armstrong.lifterUp();
        armstrong.collectServoLeftSlow();
        armstrong.collectServoRightSlow();
        sleep(10790);
        armstrong.lifterStop();
        armstrong.collectServoLeftStop();
        armstrong.collectServoRightStop();
        armstrong.unlatch();
        armstrong.armup();
        sleep(250);
        armstrong.armstop();

        armstrong.forward(0.5);
        sleep(1300);
        armstrong.stop();

        armstrong.markDown();
        sleep(1000);
        telemetry.addData("Status", "WallDown");

        armstrong.backward(0.3);
        sleep(150);
        armstrong.stop();

        armstrong.imuTurnR(92, 0.3);
        sleep(808);
        armstrong.armup();
        sleep(250);
        armstrong.armstop();

//
        armstrong.forward(0.5);
        sleep(2200);

        armstrong.markDown();


        armstrong.stop();





    }
}
