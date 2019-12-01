package org.firstinspires.ftc.teamcode.autoOp;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.auto.ChassisConfig;
import org.firstinspires.ftc.teamcode.auto.ChassisStandard;

/**
 *
 */
@Autonomous(name="Pick and Park", group="OpMode")
public class PickAndPark extends ChassisStandard {

    public PickAndPark() {
        // override the default of vuforia being off.
        useVuforia = true;
    }

    /**
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        scanStones();
        if (stoneconfig == "LEFT") {
            if (madeTheRun == false) {

                encoderDrive(-28);
                sleep(500);
                dropBackFinger();
                sleep(2000);
                encoderDrive(8);
                sleep(500);
                turnLeft(80);
                sleep(500);
                encoderDrive(-60);
                sleep(500);
                raiseBackFinger();
                sleep(2000);
                encoderDrive(36);

                madeTheRun = true;}
        } else if (stoneconfig == "CENTER") {
            if (madeTheRun == false) {


                madeTheRun = true;}
        } else if (stoneconfig == "RIGHT") {
            if (madeTheRun == false) {




                madeTheRun = true;}
        }
    }
}

