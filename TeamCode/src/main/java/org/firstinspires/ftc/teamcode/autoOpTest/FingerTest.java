package org.firstinspires.ftc.teamcode.autoOpTest;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.auto.ChassisConfig;
import org.firstinspires.ftc.teamcode.auto.ChassisStandard;

/**
 *
 */
@Autonomous(name="Finger Test", group="ZZTesting")
public class FingerTest extends ChassisStandard {

    /**
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop () {

        if (madeTheRun == false) {

            raiseBackFinger();

            raiseFrontFinger();

            dropBackFinger();

            dropFrontFinger();


           raiseBackFinger();

           raiseFrontFinger();

            madeTheRun = true;
        }

        printStatus();
    }
}

