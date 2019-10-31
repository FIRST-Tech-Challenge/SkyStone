package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * This is based on the helpful write-up at
 * https://www.roboteq.com/index.php/component/easyblog/entry/driving-mecanum-wheels-omnidirectional-robots?Itemid=1208
 * which itself is pulled from the Simplistic Control of Mecanum Drive from Ian McInerney, FRC Team 2022:
 * https://forums.parallax.com/discussion/download/79828/ControllingMecanumDrive%5B1%5D.pdf
 *
 * The controls for this mecanum drive are on gamepad1:
 *   - The right stick controls the movement of the robot in a Cartesian plane, driving the bot in
 *     a compass direction matching the direction the stick is pushed, at a speed proportionate to
 *     the amount the stick has been pushed (push harder, go faster).
 *   - The left stick rotates the bot around its center point (staying in place).
 */
@TeleOp(name="Mecanum", group="Drive Systems")
public class EncoderDrive extends OpMode {

    private DcMotor front_left, front_right, back_left, back_right;

    /**
     * It may have beena wiring figment on our end, but we noticed that the rear motors were running
     * in the reverse of the expected direction, so we chose to empirically reverse their direction.
     * We have _not_ reasoned out _why_ this was so, so it may be just a wiring glitch on our end,
     * in which case this constant should be set to 1, not -1.
     */
    private final double INVERT_REAR_MOTORS = -1;

    @Override
    public void init() {
        telemetry.addData("Initializing Mecanum Drive", "Initializing motor controllers");
        front_left = hardwareMap.get(DcMotor.class, "left front");
        front_right = hardwareMap.get(DcMotor.class, "right front");
        back_left = hardwareMap.get(DcMotor.class, "left rear");
        back_right = hardwareMap.get(DcMotor.class, "right rear");

        front_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        front_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        back_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        back_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // TODO: PID Tuning
        // (Talk to Nate)
    }

    @Override
    public void loop() {
        mecanumDrive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
    }

    private void mecanumDrive(double x, double y, double r) {
        double
                /*
                 * Calculate desired speed based on the amount the right gamepad stick has been
                 * displaced
                 */
                speed = Math.hypot(x, y),

                /*
                 * Calculate the desired compass direction based on the direction the right gamepad
                 * stick has been displaced (note that Math.atan2 expects its parameters in (dy, dx)
                 * order, which often catches the unwary!
                 */
                heading = Math.atan2(y, x)

                /*
                 * Calculate the amount of rotation, based on the horizontal displacement of the
                 * left gamepad stick
                 */,

                /*
                 * The heading is rotated 45 degrees (pi/4 radians) to place the XY axes so that
                 * they pass _through_ the wheels, rather than through the front and side bumpers
                 * of the bot.
                 */
                headingX_adjusted = Math.cos(heading + Math.PI / 4.0),
                headingY_adjusted = Math.sin(heading + Math.PI / 4.0);

        /*
         * Adjust motor power to move at speed in heading with desired rotation
         */
        front_left.setPower(speed * headingY_adjusted + r);
        front_right.setPower(speed * headingX_adjusted - r);
        back_left.setPower(INVERT_REAR_MOTORS * (speed * headingX_adjusted + r));
        back_right.setPower(INVERT_REAR_MOTORS * (speed * headingY_adjusted - r));
    }
}