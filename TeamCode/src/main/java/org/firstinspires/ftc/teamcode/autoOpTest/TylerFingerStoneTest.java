package org.firstinspires.ftc.teamcode.autoOpTest;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.auto.ChassisStandard;

@Autonomous(name="FingerStoneTest", group="ZZTesting")
public class TylerFingerStoneTest extends ChassisStandard {

    /**a
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop () {

        if (madeTheRun == false) {
            /*dropBackFinger();
            sleep(1000);
            dropFrontFinger();
            /*raiseBackFinger();
            sleep(1000);
            raiseFrontFinger();*/

            encoderDrive(-30);
            sleep(500);
            dropBackFinger();
            sleep(2000);
            encoderDrive(6);
            turnRight(75);
            encoderDrive(-60);
            raiseBackFinger();
            sleep(2000);
            encoderDrive(24);

            madeTheRun = true;
        }

        printStatus();
    }
}