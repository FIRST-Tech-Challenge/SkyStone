//TO DO: better documentation and commenting lol xd kill me

package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.hardware.HardwarePushbot;

/**
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 * The code is structured as a LinearOpMode
 * <p>
 * This particular OpMode executes a POV Game style Teleop for a PushBot
 * In this mode the left stick moves the robot FWD and back, the Right stick turns left and right.
 * It raises and lowers the claw using the Gamepad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 * <p>
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name = "DAMN SON", group = "Pushbot")

public class KyleAndEduardoGoSuperSaiyan extends LinearOpMode
{

    private final double CLAW_SPEED = 0.02;                     // sets rate to move servo
    private final double CLAW_OPEN_MAX = -0.12;
    private final double CLAW_OPEN_MIN = -0.5;
    /* Declare OpMode members. */
    HardwarePushbot robot = new HardwarePushbot();
    // Use a Pushbot's hardware
    // could also use HardwarePushbotMatrix class.
    private double clawOffset = 0;                        // Servo mid position

    @Override
    public void runOpMode()
    {
        double left;
        double right;

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        //telemetry.addData("Say", "I AM A BAD ROBOT PUNISH ME FATHER");
        //telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive())
        {
            // Normalize the values so neither exceed +/- 1.0
            left = Range.clip(-gamepad1.left_stick_y, -1, 1);
            right = Range.clip(-gamepad1.right_stick_y, -1, 1);

            // Output the safe vales to the motor drives.
            robot.leftDrive.setPower(left);
            robot.rightDrive.setPower(right);

            if (gamepad2.b)
            {
                clawOffset += CLAW_SPEED;
            }

            else if (gamepad2.x)
            {
                clawOffset -= CLAW_SPEED;
            }



            // Use gamepad buttons to move arm up (Y) and down (A)
            if (gamepad2.y)
            {
                robot.arm.setPower(HardwarePushbot.ARM_UP_POWER);
            }
            else if (gamepad2.a)
            {
                robot.arm.setPower(HardwarePushbot.ARM_DOWN_POWER);
            }
            else
            {
                robot.arm.setPower(0.0);
            }

            clawOffset = Range.clip(clawOffset, CLAW_OPEN_MIN, CLAW_OPEN_MAX);

            // Send telemetry message to signify robot running;
            telemetry.addData("claw", "Offset = %.2f", clawOffset);
            telemetry.addData("left", "%.2f", left);
            telemetry.addData("right", "%.2f", right);
            telemetry.update();

            // Pace this loop so jaw action is reasonable speed.
            sleep(25);
        }
    }
}