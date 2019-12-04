package org.firstinspires.ftc.teamcode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a four wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name = "Tank Mode", group = "Tank Mode")
public class TeleOp_Tank extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor drive_FL, drive_FR, drive_RL, drive_RR;
    private DcMotor intake_L, intake_R;
    private Servo latch_L, latch_R;

    private boolean latched = false;
    private String servoStatus = "Pending";

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        drive_FL = hardwareMap.get(DcMotor.class, "front left");
        drive_FR = hardwareMap.get(DcMotor.class, "front right");
        drive_RL = hardwareMap.get(DcMotor.class, "rear left");
        drive_RR = hardwareMap.get(DcMotor.class, "rear right");

        intake_L = hardwareMap.get(DcMotor.class, "left intake");
        intake_R = hardwareMap.get(DcMotor.class, "right intake");

        latch_L = hardwareMap.get(Servo.class, "left servo");
        latch_R = hardwareMap.get(Servo.class, "right servo");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        drive_FR.setDirection(DcMotor.Direction.REVERSE);
        drive_RL.setDirection(DcMotor.Direction.REVERSE);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // Tank Mode uses both sticks to control each side.
            drive_FL.setPower(gamepad1.left_stick_y);
            drive_FR.setPower(gamepad1.right_stick_y);
            drive_RL.setPower(gamepad1.left_stick_y);
            drive_RR.setPower(gamepad1.right_stick_y);

            // Set D-Pad for strafing -> not used for Joe 2019-2020
//            if (gamepad1.dpad_left) {
//                drive_FL.setPower(-0.5);
//                drive_FR.setPower(0.5);
//                drive_RL.setPower(0.25);
//                drive_RR.setPower(-0.25);
//            } else if (gamepad1.dpad_right) {
//                drive_FL.setPower(0.5);
//                drive_FR.setPower(-0.5);
//                drive_RL.setPower(-0.25);
//                drive_RR.setPower(0.25);
//            }

            // Map triggers to intake motors
            float LT = gamepad1.left_trigger;
            float RT = gamepad1.right_trigger;

            if (LT > 0 && RT == 0) {
                intake_L.setPower(LT * -0.5);
                intake_R.setPower(LT * 0.5);
            } else if (RT > 0 && LT == 0) {
                intake_L.setPower(RT * 0.25);
                intake_R.setPower(RT * -0.25);
            } else {
                intake_L.setPower(0);
                intake_R.setPower(0);
            }

            // Map bumpers to foundation latches
            boolean LB = gamepad1.left_bumper;
            boolean RB = gamepad1.right_bumper;

            if (LB) {
                latch_L.setPosition(1);
                latch_R.setPosition(0);

                latched = true;
            } else if (RB) {
                latch_L.setPosition(0.35);
                latch_R.setPosition(0.65);

                latched = false;
            }

            if (latched) {
                servoStatus = "latched";
            } else {
                servoStatus = "unlatched";
            }

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", -gamepad1.left_stick_y, -gamepad1.right_stick_y);
            telemetry.addData("Servos", servoStatus);
            telemetry.update();
        }
    }
}