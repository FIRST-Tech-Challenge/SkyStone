package org.eastsideprep.eps15203;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


@Autonomous(name="5 | TapePark Wait | Blue BZ / Red LZ", group="15203")

public class TapeParkBlueBZWait extends LinearOpMode {

    /* Declare OpMode members. */
    Hardware15203 robot = new Hardware15203();   // Use a Pushbot's hardware


    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "started");    //
        telemetry.update();

        waitForStart();

        sleep(20000);

        robot.allDrive(-0.5, 750);
        robot.turn(0.5, 785);
        robot.allDrive(-0.5, 1000);
        robot.allDrive(0.0, 1000);
    }
}

