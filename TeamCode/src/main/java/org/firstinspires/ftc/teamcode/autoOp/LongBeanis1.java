package org.firstinspires.ftc.teamcode.autoOp;

import org.firstinspires.ftc.teamcode.auto.ChassisStandard;
import org.firstinspires.ftc.teamcode.auto.ChassisConfig;

/**
 * This just runs from the position closest to the crater, into the crater.
 */
public abstract class LongBeanis1 extends ChassisStandard {

    public LongBeanis1(ChassisConfig config) {
        super(config);
    }


    /**a
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop () {

        if (madeTheRun == false) {

            encoderDrive(27);

            turnLeft(90);
            encoderDrive(13);
            madeTheRun = true;
        }
    }
}

