package org.firstinspires.ftc.teamcode.mainops;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.libraries.AutoLib;
import org.firstinspires.ftc.teamcode.libraries.Constants;

/*
 * Title: AutoBlueCraterBase
 * Date Created: 11/23/2018
 * Date Modified: 2/22/2019
 * Author: Rahul, Poorvi, Varnika
 * Type: Main
 * Description: Starts on blue crater latcher
 */

@Autonomous(name = "AutoCrater", group = "Concept")
public class AutoCraterBase extends LinearOpMode {
    private AutoLib autoLib;

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        // Vuforia
        Constants.Coordinates coordinates = autoLib.readCoordinates();

        if (coordinates.yPosition < 0) {
            autoLib.calcMove(5, .1f, Constants.Direction.RIGHT);
            autoLib.calcMove((float) (coordinates.yPosition / 10) + 5, .9f, Constants.Direction.RIGHT); //when decreased- moves to the left
            autoLib.calcMove((float) (-coordinates.xPosition / 10) + 5, .5f, Constants.Direction.FORWARD);   //when increased-moves back

        } else if (coordinates.yPosition < 10) {
            telemetry.addData("pos", "Right");
            Thread.sleep(1000);
            autoLib.calcMove(5, .1f, Constants.Direction.RIGHT);


        } else {
            telemetry.addData("pos", "Center");
            telemetry.update();
            Thread.sleep(1000);
            autoLib.calcMove(5, .1f, Constants.Direction.RIGHT);


        }
        telemetry.update();
    }

    private void initialize() {
        telemetry.addData("Status", "Initializing...");
        telemetry.update();

        autoLib = new AutoLib(this);

        telemetry.addData("Status", "Ready");
        telemetry.update();
        waitForStart();

        telemetry.addData("Status", "Running");
        telemetry.update();
    }
}
