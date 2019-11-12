package org.firstinspires.ftc.teamcode.autoOp;

import org.firstinspires.ftc.teamcode.auto.ChassisConfig;
import org.firstinspires.ftc.teamcode.auto.ChassisStandard;

/**
 *
 */
public abstract class FingerTest extends ChassisStandard {


    public FingerTest(ChassisConfig config) {
        super(config);
    }

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
    }
}

