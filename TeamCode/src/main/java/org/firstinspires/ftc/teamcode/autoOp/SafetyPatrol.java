package org.firstinspires.ftc.teamcode.autoOp;

import org.firstinspires.ftc.teamcode.auto.ChassisStandard;
import org.firstinspires.ftc.teamcode.auto.ChassisConfig;

/**
 * This just runs from the position closest to the crater, into the crater.
 */
public abstract class SafetyPatrol extends ChassisStandard {

    public SafetyPatrol(ChassisConfig config) {
        super(config);
    }

    /**
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop () {

        if (madeTheRun == false) {

            /*encoderDrive(36); */

            raiseCrab();
            sleep(1000);
            encoderDrive(34);
            encoderDrive(4, 4, 0.5);
            dropCrab();
            sleep(2000);
            encoderDrive(-15, -15, 1.0);
            turnRight(50);
            encoderDrive(24);
            raiseCrab();
            sleep(1000);
            encoderDrive(-5);
            turnLeft(55);
            encoderDrive(-30);

            madeTheRun = true;
        }

        printStatus();
    }
}

