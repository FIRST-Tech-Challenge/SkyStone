package org.firstinspires.ftc.teamcode.autoOpTest;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.auto.ChassisConfig;
import org.firstinspires.ftc.teamcode.auto.ChassisStandard;

/**
 *
 */
@Autonomous(name="Short Tray Turn Test", group="ZZTesting")
public class ShortTrayTurnTest extends ChassisStandard {


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

        printStatus();
    }
}

