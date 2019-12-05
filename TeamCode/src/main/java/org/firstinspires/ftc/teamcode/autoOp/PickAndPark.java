package org.firstinspires.ftc.teamcode.autoOp;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.auto.ChassisConfig;
import org.firstinspires.ftc.teamcode.auto.ChassisStandard;

/**
 *
 */
@Autonomous(name="Pick and Park", group="OpMode")
public class     PickAndPark extends ChassisStandard {

    public PickAndPark() {
        // override the default of vuforia being off.
        useVuforia = true;

        // need this for tyler 2 chassis
       // switchMotorDirection();
    }

    /**
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        if (madeTheRun == false) {

            scanStones();
            if (stoneconfig == "LEFT") {

                encoderDrive(30);
                sleep(500);
                dropFrontFinger();
                sleep(2000);
                encoderDrive(-6);
                sleep(500);
                turnRight(95);
                sleep(500);

                encoderDrive(60);
                sleep(500);
                raiseFrontFinger();
                sleep(2000);
                encoderDrive(-24);


            } else if (stoneconfig =="CENTER") {

                encoderDrive(6);
                turnRight(45);
                encoderDrive(8);
                turnLeft(42);
                encoderDrive(19);


                sleep(500);

                dropFrontFinger();
                 sleep(2000);

                encoderDrive(-5);
                sleep(500);
                turnRight(85);
                sleep(500);
                encoderDrive(60);
                sleep(500);
                raiseFrontFinger();
                sleep(2000);
                encoderDrive(-24);

            } else if (stoneconfig == "RIGHT") {

                encoderDrive(6);
                turnRight(45);
                encoderDrive(20);
                turnLeft(38);
                encoderDrive(13);


                sleep(500);
                dropFrontFinger();
                sleep(2000);

                encoderDrive(-5);
                sleep(500);
                turnRight(80);
                sleep(500);
                encoderDrive(45);
                sleep(500);
                raiseFrontFinger();
                sleep(2000);
                encoderDrive(-24);
            }

            madeTheRun = true;
        }

        printStatus();
    }
}

