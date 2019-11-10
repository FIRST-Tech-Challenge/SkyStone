package org.firstinspires.ftc.teamcode.mainops;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.libraries.TeleLib;

/*
 * Title: MainTeleOp
 * Date Created: 10/14/2018
 * Date Modified: 2/27/2019
 * Author: Poorvi, Sachin
 * Type: Main
 * Description: This is the main teleop program we will use
 */

@TeleOp(group = "Main")
public class MainTeleOp extends LinearOpMode {
    private TeleLib teleLib;

    @SuppressWarnings("RedundantThrows")
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        while (opModeIsActive()) {
            // Gamepad 1
            teleLib.processDrive();
            teleLib.processIntakeMinerals();
            teleLib.processMoveArmUp();
            teleLib.processStopIntake();
            teleLib.processIntakeGrab();
            teleLib.processScoreStone();

            // Gamepad 2
            teleLib.processFoundation();
            teleLib.processServoArm();
            teleLib.processServoGrab();
            idle();
        }
    }

    private void initialize() {
        telemetry.addData("Status", "Initializing...");
        telemetry.update();

        teleLib = new TeleLib(this);

        telemetry.addData("Status", "Ready");
        telemetry.update();
        waitForStart();

        telemetry.addData("Status", "Running");
    }
}