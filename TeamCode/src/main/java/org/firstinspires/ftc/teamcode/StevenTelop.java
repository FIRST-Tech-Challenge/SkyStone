package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name="Steve Robot Test", group="Steve")
public class StevenTelop extends LinearOpMode {

    /* Declare OpMode members. */
    RobotHardware robot      = new RobotHardware();   // Use a Pushbot's hardware
    int counter       = 0;                     // Servo mid position

    @Override
    public void runOpMode() {

        robot.init(hardwareMap, telemetry);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            robot.clamp.setPosition(.5);
            sleep(1000);

            robot.clamp.setPosition(-0.5);
            sleep(1000);

            telemetry.addData("We be counting",  "Counter = %7d", counter);
            telemetry.update();


            counter += 1;

            if (counter == 5) {
                break;
            }

        }
    }
}