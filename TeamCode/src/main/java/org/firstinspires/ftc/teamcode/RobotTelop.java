package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name="Robot: Teleop", group="Steve")
public class RobotTelop extends LinearOpMode {

    /* Declare OpMode members. */
    RobotHardware robot      = new RobotHardware();   // Use a Pushbot's hardware
    int counter       = 0;                     // Servo mid position

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);
        telemetry.addData("Say", "Hi Steve");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {


            telemetry.addData("We be counting",  "Counter = %7d", counter);

            // Pace this loop so servo speed is reasonable.
            sleep(1000);

        }
    }
}