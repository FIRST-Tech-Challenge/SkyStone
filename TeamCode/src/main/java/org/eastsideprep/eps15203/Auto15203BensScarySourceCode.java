package org.eastsideprep.eps15203;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


@Autonomous(name="Autonomous Template 15203", group="15203")

public class Auto15203BensScarySourceCode extends LinearOpMode {

    /* Declare OpMode members. */
    Hardware15203 robot = new Hardware15203();   // Use a Pushbot's hardware


    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "started");    //
        telemetry.addData("Written and Designed By:", "Benjamin");
        telemetry.update();
//This is my edited version for pulling the platform
        /*
        ________________________________________
        |       X           |                  |
        |       |-----------|--|               |
        |                   |  |               |
        |                   |  |         < B > |
        |                   |  |               |
        |                   |  |         ------|
        |                   |  |               |
        |                   |  |         < A > |
        |                   |  |----------|    |
        |                   |             |    |
        |___________________|__________X--|____|
         */
        robot.allDrive(0.5, 100); //Moves forward roughly (X)feet
        robot.turn(0.5, 785); //Turns left at a (X) degree angle
        robot.allDrive(0.5, 600); //move forward (X) feet
        robot.turn(-0.5, 785); //Turn turn right (X) degrees again
        robot.allDrive(0.5, 800); //Move forward (X) feet
        robot.turn(0.5, 785); //Turn left (X) degrees
        robot.allDrive(0.5, 4); //Move forward (X) feet
        robot.turn(-0.5, 785); //Turn right (X) degrees
        //Code to have finger come down.
        robot.allDrive(-0.5, 50); // Move forward (X) feet
        robot.allDrive(-0.5, -200); //Move backwards (X) feet
        //Code to have finger come up.
        robot.turn(0.5, 785); //Turn (X)degrees left.
        robot.allDrive(-0.5, 400); //Go backwards (X) feet.

    }
}

