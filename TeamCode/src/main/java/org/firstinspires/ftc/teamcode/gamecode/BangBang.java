package org.firstinspires.ftc.teamcode.gamecode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodesupport.AutoOpMode;
import org.firstinspires.ftc.teamcode.robots.Armstrong;

@Autonomous
public class BangBang extends AutoOpMode {

    private int followdist;
    private int slantamount;
    @Override
    public void runOp() throws InterruptedException {
        Armstrong armstrong = new Armstrong();
        followdist = 3000;
        waitForStart();
        while (opModeIsActive()) {

            if (armstrong.ultrasonic.getDistance() > 270){
                slantamount = 1;
                armstrong.slantforward(0.3, 0.2);
            }
            else if (armstrong.ultrasonic.getDistance() < 260){
                armstrong.slantforward(0.2, 0.3);
            }
            else {
                armstrong.forward(0.3);
                sleep(100);

            }

        }
    }
}
