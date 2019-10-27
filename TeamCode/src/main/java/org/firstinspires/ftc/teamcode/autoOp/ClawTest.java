package org.firstinspires.ftc.teamcode.autoOp;

import org.firstinspires.ftc.teamcode.TylerController;
import org.firstinspires.ftc.teamcode.auto.ChassisConfig;
import org.firstinspires.ftc.teamcode.auto.ChassisStandard;

/**
 * This just runs from the position closest to the crater, into the crater.
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

            // encoderDrive(24);
            raiseCrab();
            sleep(2000);
            /*while (crab.getPosition() < 1.0f) {
                if (useCrab) {
                    telemetry.addData("Crab", "Angle =%f", crab.getPosition());
                    telemetry.update();
                }
            }*/

            //encoderDrive(-12);
            dropCrab();
            sleep(2000);
           /* while (crab.getPosition() > 0.0f) {
                if (useCrab) {
                         telemetry.addData("Crab", "Angle =%f", crab.getPosition());
                         telemetry.update();
                     }
            }*/

            encoderDrive(-24);

            madeTheRun = true;
        }
    }
}

