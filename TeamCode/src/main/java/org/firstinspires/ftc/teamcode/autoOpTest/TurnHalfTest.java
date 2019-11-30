package org.firstinspires.ftc.teamcode.autoOpTest;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.auto.ChassisStandard;
import org.firstinspires.ftc.teamcode.auto.ChassisConfig;

/**
 *
 */
@Autonomous(name="Turn Test Half", group="ZZTesting")
public class TurnHalfTest extends ChassisStandard {

    /**
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop () {

        if (madeTheRun == false) {

            turnRight(180);
            sleep(500);
            turnLeft(180);

            madeTheRun = true;
        }

        printStatus();
    }
}

