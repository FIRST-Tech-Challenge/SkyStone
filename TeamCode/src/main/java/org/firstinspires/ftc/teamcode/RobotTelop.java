package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name="Robot: Teleop", group="Steve")
public class RobotTelop extends LinearOpMode {

    /* Declare OpMode members. */
    RobotHardware robot      = new RobotHardware();   // Use a Pushbot's hardware
    double servoOffset       = 0;                     // Servo mid position
    final double SERVO_SPEED = 0.02;                  // sets rate to move servo

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);
        telemetry.addData("Say", "Hi Steve");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Use gamepad left & right Bumpers to move the servo
            if (gamepad1.right_bumper)
                servoOffset += SERVO_SPEED;
            else if (gamepad1.left_bumper)
                servoOffset -= SERVO_SPEED;

            servoOffset = Range.clip(servoOffset, -0.5, 0.5);
            robot.servoOne.setPosition(robot.MID_SERVO + servoOffset);

            telemetry.addData("servo_one",  "Offset = %.2f", servoOffset);

            // Pace this loop so servo speed is reasonable.
            sleep(50);

        }
    }
}