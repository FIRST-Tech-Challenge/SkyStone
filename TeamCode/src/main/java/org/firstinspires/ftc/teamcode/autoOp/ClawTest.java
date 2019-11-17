package org.firstinspires.ftc.teamcode.autoOp;

import org.firstinspires.ftc.teamcode.auto.ChassisConfig;
import org.firstinspires.ftc.teamcode.auto.ChassisStandard;

/**
 *
 */
public abstract class ClawTest extends ChassisStandard {


    public ClawTest(ChassisConfig config) {
        super(config);
    }

    /**
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop () {

        if (madeTheRun == false) {

            raiseCrab();
            sleep(2000);

            dropCrab();
            sleep(2000);

            madeTheRun = true;
        }

        printStatus();
    }
}

