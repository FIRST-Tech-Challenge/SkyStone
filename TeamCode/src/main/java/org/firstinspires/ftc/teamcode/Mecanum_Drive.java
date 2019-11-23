package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp
public class Mecanum_Drive extends LinearOpMode {
    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private DcMotor lowerArm;
    private Servo leftClaw, rightClaw, linear, grabber, rotate, front_grab, side_grab;
    private DigitalChannel lower_sensor;
    private float fwd, side, turn, power, lower_arm_stick, grabber_stick;
    private double clawOffset = 0.5;
    private double rotation_offset = 0.0;
    private double linearOffset = 0.0;
    private double grabberOffset = 0.0;
    private boolean btn_y, btn_a, left_bumper_1, right_bumper_1, left_bumper_2, right_bumper_2, dpad_left, dpad_right, btn_x1, btn_b1, btn_x2, btn_b2;
    private boolean clawExpanded = false;
    private boolean clawOpen = true;
    private boolean turning = false;
    private boolean gyro_assist = false;
    private boolean front_grabber_up = true;
    private boolean side_grabber_up = true;
    private double target_grabber = 0.0;


    // Gyro related initialization
    private BNO055IMU imu;
    private Orientation lastAngles = new Orientation();
    private double globalAngle, correction;

    // Variables
    public final static double deadzone = 0.2;              // Deadzone for bot movement
    public final static double deadzone_hi = 0.9;           // Deadzone for straight movement
    public final static double turn_speed_max = 0.15;       // Max turn speed
    public final static double rotation_speed = 0.01;       // Claw rotation movement rate
    public final static double rotation_offset_max = 0.1;   // Max offset allowed to move rotation servo
    public final static double rotation_initial = 0.5;       // Claw rotation movement rate
    public final static double claw_speed = 0.01;           // Claw movement rate
    public final static double claw_left_initial = 0.0;    // Left claw initial position
    public final static double claw_right_initial = 0.0;   // Right claw initial position
    public final static double claw_mid_position = 0.65;    // Claw mid point position
    public final static double claw_move_span = 0.35;       // Claw movement span
    public final static double claw_grip_offset = 0.35;     // Grip offset for claws
    public final static double claw_release_offset = -0.05; // Release offset for claws
    public final static double linear_speed = 0.005;        // Linear servo rotation rate
    public final static double linear_lower = 0.32;         // Linear servo lower limit
    public final static double linear_upper = 0.615;        // Linear servo upper limit
    public final static double linear_initial = linear_lower;
    public final static double linear_offset_max = 0.01;    // Max offset allowed to move linear servo
    public final static double grabber_speed = 0.005;       // Grabber servo rotation rate
    public final static double arm_up_power = 1.0;          // Arm up movement rate
    public final static double arm_down_power = -1.0;       // Arm down movement rate
    public final static int arm_up_step = 30;               // Arm up movement position step
    public final static int arm_down_step = 30;             // Arm up movement position step

    @Override
    public void runOpMode() {
        // Gyro initialization and calibration
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // make sure the imu gyro is calibrated before continuing.
        while (!imu.isGyroCalibrated()) {
            sleep(50);
            idle();
        }

        // Reset angle
        resetAngle();

        // Mecanum wheels
        frontLeft = hardwareMap.get(DcMotor.class, "front_left");
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight = hardwareMap.get(DcMotor.class, "front_right");
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft = hardwareMap.get(DcMotor.class, "back_left");
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight = hardwareMap.get(DcMotor.class, "back_right");
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        // Arm motor
        lowerArm = hardwareMap.get(DcMotor.class, "lower_arm");
        lowerArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Claw servos
        leftClaw = hardwareMap.get(Servo.class, "left_claw");
        rightClaw = hardwareMap.get(Servo.class, "right_claw");
        linear = hardwareMap.get(Servo.class, "linear");
        grabber = hardwareMap.get(Servo.class,"grabber");
        rotate = hardwareMap.get(Servo.class, "rotate");

        // Front and side grabber servos
        front_grab = hardwareMap.get(Servo.class, "front_grab");
        side_grab = hardwareMap.get(Servo.class, "side_grab");

        // Sensor for lower arm
        lower_sensor = hardwareMap.get(DigitalChannel.class, "arm_lower_sensor");
        lower_sensor.setMode(DigitalChannel.Mode.INPUT);


        // Setting initial positions for claw, grabber, linear servo and arm
        initializePositions();


        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            getGamePadValues();

            // Bot movement with Mecanum wheels
            power = fwd; //this can be tweaked for exponential power increase

            if ( turn != 0 ) {
                // We are turing
                turning = true;

                frontLeft.setPower(Range.clip(power + side - turn, -turn_speed_max, turn_speed_max));
                frontRight.setPower(Range.clip(power - side + turn, -turn_speed_max, turn_speed_max));
                backLeft.setPower(Range.clip(power - side - turn, -turn_speed_max, turn_speed_max));
                backRight.setPower(Range.clip(power + side + turn, -turn_speed_max, turn_speed_max));
            } else {
                // Just finished turning and let it settled down
                if ( turning ) {
                    frontLeft.setPower(0);
                    frontRight.setPower(0);
                    backLeft.setPower(0);
                    backRight.setPower(0);
                    sleep(500);

                    resetAngle();
                    turning = false;
                }

                // Moving in straight line (forward, backward or sideway) without turning; Add gyro assistance
                if (gyro_assist)
                    correction = checkDirection();
                else
                    correction = 0;


                frontLeft.setPower(Range.clip(power + side + correction, -1, 1));
                frontRight.setPower(Range.clip(power - side - correction, -1, 1));
                backLeft.setPower(Range.clip(power - side + correction, -1, 1));
                backRight.setPower(Range.clip(power + side - correction, -1, 1));

            }


            // Arm movements
            if (lower_arm_stick < 0 && lowerArm.getCurrentPosition() < 3000) {
                lowerArm.setTargetPosition(lowerArm.getCurrentPosition() + arm_up_step);
                lowerArm.setPower(arm_up_power);
            } else if (lower_arm_stick > 0 && lower_sensor.getState()) {
                lowerArm.setTargetPosition(lowerArm.getCurrentPosition() - arm_down_step);
                lowerArm.setPower(arm_down_power);
            } else
                lowerArm.setPower(0.0);


            // Use gamepad left & right Bumpers to open and close the claw
            if (left_bumper_2) {
                clawOffset += claw_speed;
//                clawOffset = claw_grip_offset;
                clawOpen = true;
                clawExpanded = true;
            } else if (right_bumper_2) {
                clawOffset -= claw_speed;
//                clawOffset = claw_release_offset;
                clawOpen = false;
                clawExpanded = true;
            }

            // Move both claw servos to new position.  Assume servos are mirror image of each other.
            if ( clawExpanded ) {
//                clawOffset = Range.clip(clawOffset, -claw_move_span, claw_move_span);
//                leftClaw.setPosition(claw_mid_position + clawOffset);
//                rightClaw.setPosition(claw_mid_position - clawOffset);
                if (clawOpen) {
                    leftClaw.setPosition(0.6);
                    rightClaw.setPosition(0.1);
                } else {
                    leftClaw.setPosition(0.2);
                    rightClaw.setPosition(0.4);
                }
            }


            // User gamepad left bumper to move up and down the front grabber
            if (left_bumper_1) {
                front_grabber_up = !front_grabber_up;
                sleep(300);
            }

            if ( front_grabber_up )
                front_grab.setPosition(0.0);
            else
                front_grab.setPosition(1.0);

            // User gamepad right bumper to move up and down the side grabber
            if (right_bumper_1) {
                side_grabber_up = !side_grabber_up;
                sleep(300);
            }

            if ( side_grabber_up )
                side_grab.setPosition(0.0);
            else
                side_grab.setPosition(1.0);


            // User DPad left and right to rotate claw
            if (dpad_left)
                rotation_offset -= rotation_speed;
            else if (dpad_right)
                rotation_offset += rotation_speed;

            rotation_offset = Range.clip( rotation_offset, -0.5, 0.5 );
            rotate.setPosition( rotation_initial + rotation_offset );


            // Use gamepad Y & right A buttons to rotate the linear servo
            if (btn_y)
                linearOffset += linear_speed;
            else if (btn_a)
                linearOffset -= linear_speed;

            // Move linear servo to the desired position.
            linearOffset = Range.clip(linearOffset, -linear_offset_max, linear_offset_max);       // At most move certain limit amount for linear servo

            // Only set position when linear action buttons are not pressed OR linearOffset is at max allowed value
            if (( !btn_y && !btn_a ) || Math.abs(linearOffset) == linear_offset_max ) {
                double target = linear.getPosition() + linearOffset;
                if (target < linear_lower)
                    target = linear_lower;
                if (target > linear_upper)
                    target = linear_upper;
                linear.setPosition(target);

                if ( grabber.getPosition() > 0.5 ) {
                    target_grabber = 0.77 + (target - linear_lower) / (linear_upper - linear_lower) * (1.0 - 0.77);
                    grabberOffset = target_grabber;
                }

                linearOffset = 0.0;
            }


            // Grabber movement
            if ( grabber_stick < 0 )
                grabberOffset += grabber_speed;
            else if ( grabber_stick > 0 )
                grabberOffset -= grabber_speed;

            // Move grabber servo to the desired position.
            grabberOffset = Range.clip(grabberOffset, 0.0, 1.0);
            grabber.setPosition(grabberOffset);


            // Check gyro support toggle
            if ( btn_x1 && btn_b1 ) {
                gyro_assist = !gyro_assist;
                resetAngle();
            }

            // Check request for initialize positions
            if ( btn_x2 && btn_b2 )
                initializePositions();

            // Output Telemetry information
//            telemetry.addData("FWD=", fwd);
//            telemetry.addData("SIDE=", side);
//            telemetry.addData("TURN=", turn);
//            telemetry.addData("RCLAW=", rightClaw.getPosition());
//            telemetry.addData("LCLAW", leftClaw.getPosition());
//            telemetry.addData("GRAB=", grabber.getPosition());
//            telemetry.addData("TGRAB=", target_grabber );
//            telemetry.addData("Linear= ", linear.getPosition());
//            telemetry.addData("ARMSTK=", lower_arm_stick);
//            telemetry.addData("Arm= ", lowerArm.getCurrentPosition());
//            telemetry.addData("Gyro= ", gyro_assist);
            telemetry.addData("LF", frontLeft.getCurrentPosition());
            telemetry.addData("RF", frontRight.getCurrentPosition());
            telemetry.addData("LB", backLeft.getCurrentPosition());
            telemetry.addData("RB", backRight.getCurrentPosition());

            telemetry.update();
        }
    }

    void getGamePadValues() {
        fwd = gamepad1.left_stick_y;                // For mecanum drive forward and backward
        side = gamepad1.left_stick_x;               // For mecanum drive sideway
        turn = gamepad1.right_stick_x;              // For bot turns

        btn_x1 = gamepad1.x;                        // For toggle of gyro assistance
        btn_b1 = gamepad1.b;                        // For toggle of gyro assistance

        lower_arm_stick = gamepad2.left_stick_y;    // For lower arm movement
        grabber_stick = gamepad2.right_stick_y;     // For grabber

        btn_y = gamepad2.y;                         // For linear servo - up
        btn_a = gamepad2.a;                         // For linear servo - down
        btn_x2 = gamepad2.x;                        // For initialize positions
        btn_b2 = gamepad2.b;                        // For initialize positions

        left_bumper_1 = gamepad1.left_bumper;       // For front grabber
        right_bumper_1 = gamepad1.right_bumper;     // For right grabber
        left_bumper_2 = gamepad2.left_bumper;         // For claw release
        right_bumper_2 = gamepad2.right_bumper;       // For claw grip

        dpad_left = gamepad2.dpad_left;             // For rotation servo
        dpad_right = gamepad2.dpad_right;           // For rotation servo

        //updates joystick values
        if( Math.abs(fwd) < deadzone_hi ) fwd = 0;
        if( Math.abs(side) < deadzone_hi ) side = 0;
        if( Math.abs(turn) < deadzone ) turn = 0;
        if( Math.abs(lower_arm_stick) < deadzone ) lower_arm_stick = 0;
        if( Math.abs(grabber_stick) < deadzone ) grabber_stick = 0;
        //checks deadzones
    }

    void initializePositions() {
        // Init claw and grabber positions
        leftClaw.setPosition(claw_left_initial);
        rightClaw.setPosition(claw_right_initial);
        clawExpanded = false;
        grabber.setPosition(0.0);
        grabberOffset = 0.0;
        clawOffset = 0.0;
        sleep(2000);

        // Init linear servo position
        linear.setPosition(linear_initial);

        // Initialize lower arm position
        while (lower_sensor.getState()) {
            lowerArm.setTargetPosition(lowerArm.getCurrentPosition() - arm_down_step);
            lowerArm.setPower(arm_down_power);
        }
        lowerArm.setPower(0);
        lowerArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lowerArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Resets the cumulative angle tracking to zero.
     */
    private void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right.
     */
    private double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    /**
     * See if we are moving in a straight line and if not return a power correction value.
     * @return Power adjustment, + is adjust left - is adjust right.
     */
    private double checkDirection()
    {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction, angle, gain = 0.02;

        angle = getAngle();

        if (angle < 5 && angle > -5)
            correction = 0;             // no adjustment.
        else
            correction = -angle;        // reverse sign of angle for correction.

        correction = correction * gain;

        return correction;
    }
}

