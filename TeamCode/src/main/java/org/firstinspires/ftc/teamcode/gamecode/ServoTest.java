package org.firstinspires.ftc.teamcode.gamecode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.opmodesupport.AutoOpMode;
import org.firstinspires.ftc.teamcode.robots.Felix;

/**
 * Created by Aila on 2018-01-22.
 */

@Autonomous
@Disabled
public class ServoTest extends AutoOpMode {

    private Felix green = null;

    @Override
    public void runOp() throws InterruptedException {

        green = new Felix();


        double i = 1;

        while (i > 0 && opModeIsActive()){
            green.jewelL.setPosition(i);
            sleep(1000);
            i = i - 0.1;
        }

        green.jewelL.setPosition(0);
        sleep(1000);

    }
}
