package org.eastsideprep.eps8103;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.eastsideprep.eps9884.Hardware9884;


@Autonomous(name="Skystone Gang", group="8103")

public class Auto8103 extends LinearOpMode {

    /* Declare OpMode members. */
    Hardware8103 robot = new Hardware8103();   // Use a Pushbot's hardware


    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "started");    //
        telemetry.update();
    }
}

