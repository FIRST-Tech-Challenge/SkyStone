package org.firstinspires.ftc.teamcode.autoOp;

import org.firstinspires.ftc.teamcode.auto.ChassisConfig;
import org.firstinspires.ftc.teamcode.auto.ChassisStandard;

/**
 * This just runs from the position closest to the crater, into the crater.
 */
public abstract class ShortTrayTurnTest extends ChassisStandard {


    public ShortTrayTurnTest(ChassisConfig config) {
        super(config);
    }

    /**
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */

    public void loop () {

        if (madeTheRun == false) {

            encoderDrive(-8);
            turnRight(50);
            encoderDrive(10);

            //raiseCrab();

            madeTheRun = true;
        }
    }
}

