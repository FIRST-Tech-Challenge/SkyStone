package org.firstinspires.ftc.teamcode.gamecode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodesupport.AutoOpMode;
import org.firstinspires.ftc.teamcode.robots.Armstrong;
@Autonomous
public class ArmstrongLand extends AutoOpMode {
    @Override

    public void runOp() throws InterruptedException {
        //init phase
        Armstrong armstrong = new Armstrong();
        telemetry.addData("Status", "Initialized");
        waitForStart();
        //this is after the driver presses play
        armstrong.lifterUp();
        sleep(8000);
        armstrong.lifterStop();
        armstrong.unlatch();
        sleep(1000);
        armstrong.forward(0.3);
        sleep(1000);
        armstrong.stop();


        //armstrong.backwardsDistance(1000,9);
        //sleep(90);





    }
}
