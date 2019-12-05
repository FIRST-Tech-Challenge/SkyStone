package org.firstinspires.ftc.teamcode.autoOp;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.auto.ChassisStandard;

/**
 *
 */
@Autonomous(name="FoundationSnatch", group="ZZTesting")
public class FoundationSnatch extends ChassisStandard {

    /**
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop () {

        if (madeTheRun == false) {

            /*encoderDrive(36); */

            raiseCrab();
            sleep(1000);
            encoderDrive(45);
            turnRight(90);
            encoderDrive(10);
            dropCrab();
            sleep(1000);
            turnRight(75);
            encoderDrive(22);

            raiseCrab();
            sleep(1000);

            encoderDrive(-5);
            turnRight(80);
            encoderDrive(20);
            sleep(1000);

            turnRight(30);
            encoderDrive(25);



            //encoderDrive(-5);
            //turnLeft(55);
            //encoderDrive(-30);

            madeTheRun = true;
        }

        printStatus();
    }
}

