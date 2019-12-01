package org.firstinspires.ftc.teamcode.autoOpTest;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.auto.ChassisConfig;
import org.firstinspires.ftc.teamcode.auto.ChassisStandard;

/**
 *
 */
@Autonomous(name="Magnet Test", group="ZZTesting")
public class MagnetTest extends ChassisStandard {

    /**
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        if (madeTheRun == false) {

            telemetry.addData("Magnet's epic status", "loc=%s", isElevatorMagnetOn());
            //madeTheRun = true;

            printStatus();
        }
    }
}

