package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp
public class Mecanum_Drive extends LinearOpMode {
    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private DcMotor upperArm, lowerArm;
    private Servo leftClaw, rightClaw, grabber;
    float fwd, side, turn, power;
    double clawOffset = 0.0;
    double grabberOffset = 0.5;
    boolean dpad_up, dpad_down, btn_y, btn_a, btn_x, btn_b, left_bumper, right_bumper;

    public final static double deadzone = 0.2;          // Deadzone for bot movement
    public final static double claw_speed = 0.01;       // Claw movement rate
    public final static double grabber_speed = 0.01;    // Grabber rotation rate
    public final static double arm_up_power = 0.45;    // Grabber rotation rate
    public final static double arm_down_power = -0.45;    // Grabber rotation rate

    @Override
    public void runOpMode() {
        // Mecanum wheels
        frontLeft = hardwareMap.get(DcMotor.class, "front_left");
        frontRight = hardwareMap.get(DcMotor.class, "front_right");
        backLeft = hardwareMap.get(DcMotor.class, "back_left");
        backRight = hardwareMap.get(DcMotor.class, "back_right");

        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        // Arm motors
//        upperArm = hardwareMap.get(DcMotor.class, "upper_arm");
//        lowerArm = hardwareMap.get(DcMotor.class, "lower_arm");

        // Claw servos
        leftClaw = hardwareMap.get(Servo.class, "left_claw");
        rightClaw = hardwareMap.get(Servo.class, "right_claw");
        grabber = hardwareMap.get(Servo.class, "grabber");

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            getGamePadValues();

            // Bot movement with Mecanum wheels
            power = fwd; //this can be tweaked for exponential power increase

            frontLeft.setPower(Range.clip(power + side - turn, -1, 1));
            frontRight.setPower(Range.clip(power - side + turn, -1, 1));
            backLeft.setPower(Range.clip(power - side - turn, -1, 1));
            backRight.setPower(Range.clip(power + side + turn, -1, 1));


            // Arm movements
            /*
            if (dpad_up)
                upperArm.setPower(arm_up_power);
            else if (dpad_down)
                upperArm.setPower(arm_down_power);
            else
                upperArm.setPower(0.0);

            if (btn_y)
                lowerArm.setPower(arm_up_power);
            else if (dpad_down)
                lowerArm.setPower(arm_down_power);
            else
                lowerArm.setPower(0.0);
             */


            // Use gamepad left & right Bumpers to open and close the claw
            if (left_bumper)
                clawOffset += claw_speed;
            else if (right_bumper)
                clawOffset -= claw_speed;

            // Move both servos to new position.  Assume servos are mirror image of each other.
            clawOffset = Range.clip(clawOffset, -0.5, 0.5);
            leftClaw.setPosition(0.5 + clawOffset);
            rightClaw.setPosition(0.5 - clawOffset);


            // Use gamepad X & right B buttons to rotate the grabber
            if (btn_x)
                grabberOffset += grabber_speed;
            else if (btn_b)
                grabberOffset -= grabber_speed;

            // Move grabber servo to the desired position.
            grabberOffset = Range.clip(grabberOffset, -0.5, 0.5);
            grabber.setPosition(0.5 + grabberOffset);

//            telemetry.addData("Motor Power FL ", frontLeft.getPower());
//            telemetry.addData("Motor Power FR", frontRight.getPower());
//            telemetry.addData("Motor Power BL", backLeft.getPower());
//            telemetry.addData("Motor Power BR", backRight.getPower());
//            telemetry.addData("Status", "Running");
            telemetry.addData("DPad UP = ", dpad_up );
            telemetry.addData("DPad DOWN = ", dpad_up );
            telemetry.addData("Y = ", btn_y );
            telemetry.addData("A = ", btn_x );
            telemetry.update();
        }
    }

    void getGamePadValues() {
        fwd = gamepad1.left_stick_y;
        side = gamepad1.left_stick_x;
        turn = gamepad1.right_stick_x;

        dpad_up = gamepad1.dpad_up;
        dpad_down = gamepad1.dpad_down;
        btn_y = gamepad1.y;
        btn_a = gamepad1.a;

        btn_x = gamepad1.x;
        btn_b = gamepad1.b;
        left_bumper = gamepad1.left_bumper;
        right_bumper = gamepad1.right_bumper;

        //updates joystick values
        if( Math.abs(fwd) < deadzone ) fwd = 0;
        if( Math.abs(side) < deadzone ) side = 0;
        if( Math.abs(turn) < deadzone ) turn = 0;
        //checks deadzones
    }
}
