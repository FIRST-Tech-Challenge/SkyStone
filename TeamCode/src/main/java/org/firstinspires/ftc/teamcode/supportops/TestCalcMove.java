package org.firstinspires.ftc.teamcode.supportops;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.internal.android.dx.ssa.DomFront;
import org.firstinspires.ftc.teamcode.libraries.AutoLib;
import org.firstinspires.ftc.teamcode.libraries.Constants;

/*
 * Title: CalcTurn Test
 * Date Created: 2/13/2019
 * Date Modified: 2/22/2019
 * Author: Poorvi
 * Type: Support
 * Description: This will test if the robot can actually turn
 */

@Autonomous(group = "Support")
public class TestCalcMove extends LinearOpMode {
    private AutoLib autoLib;


    @SuppressWarnings("RedundantThrows")
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        telemetry.addData("about to move","initialized");
        telemetry.update();
        Thread.sleep(2000);

       autoLib.latchServoFoundation();
       autoLib.grabServo();

        telemetry.addData("Just moved","finished moving");
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
