package org.firstinspires.ftc.teamcode.autoOpTest;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.auto.ChassisStandard;
import org.firstinspires.ftc.teamcode.auto.ChassisConfig;

import java.util.List;

/**
 *
 */
@Autonomous(name="Turn Test Quarter", group="ZZTesting")
public class TurnQuarterTest extends ChassisStandard {

    /**a
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop () {

        if (madeTheRun == false) {

            turnRight(90);
            sleep(500);
            turnLeft(90);

            madeTheRun = true;
        }

        printStatus();
    }
}

