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
    private DcMotor upperArm, lowerArm;
    private Servo leftClaw, rightClaw, grabber, grabber2;
    private DigitalChannel upper_sensor, lower_sensor;
    private float fwd, side, turn, power, lower_arm_stick, grabber_stick;
    private double clawOffset = 0.5;
    private double grabberOffset = 0.0;
    private double grabberOffset2 = 0.0;
    private boolean dpad_up, dpad_down, btn_y, btn_a, btn_x, btn_b, left_bumper, right_bumper;

    // Gyro related initialization
    private BNO055IMU imu;
    private Orientation lastAngles = new Orientation();
    private double globalAngle, correction;

    public final static double deadzone = 0.2;          // Deadzone for bot movement
    public final static double claw_speed = 0.01;       // Claw movement rate
    public final static double grabber_speed = 0.005;    // Grabber rotation rate
    public final static double grabber_lower = 0.3;    // Grabber lower limit
    public final static double grabber_upper = 0.615;      // Grabber upper limit
    public final static double grabber_initial = grabber_lower;
    public final static double arm_up_power = 1.0;    // Grabber rotation rate
    public final static double arm_down_power = -1.0;    // Grabber rotation rate
    public final static int arm_up_step = 30;           // Arm up movement position step
    public final static int arm_down_step = 30;           // Arm up movement position step

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

        // Arm motors
//        upperArm = hardwareMap.get(DcMotor.class, "upper_arm");
        lowerArm = hardwareMap.get(DcMotor.class, "lower_arm");

//        upperArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        upperArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        upperArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lowerArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        lowerArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        lowerArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        lowerArm.setDirection(DcMotor.Direction.REVERSE);

        // Claw servos
        leftClaw = hardwareMap.get(Servo.class, "left_claw");
        rightClaw = hardwareMap.get(Servo.class, "right_claw");
        grabber = hardwareMap.get(Servo.class, "linear");
        grabber2 = hardwareMap.get(Servo.class,"grabber");

        leftClaw.setPosition(0.07);
        rightClaw.setPosition(0.55);
        grabber2.setPosition(grabberOffset2);
        boolean clawExpanded = false;

        telemetry.addData("LClaw= ", leftClaw.getPosition());
        telemetry.addData("RClaw= ", rightClaw.getPosition());
        telemetry.addData("Grabber2= ", grabber2.getPosition());
        telemetry.update();

        sleep(2000);
        grabber.setPosition(grabber_initial);

        // Touch sensors
        /*
        upper_sensor = hardwareMap.get(DigitalChannel.class, "arm_upper_sensor");
        upper_sensor.setMode(DigitalChannel.Mode.INPUT);
        */
        lower_sensor = hardwareMap.get(DigitalChannel.class, "arm_lower_sensor");
        lower_sensor.setMode(DigitalChannel.Mode.INPUT);

        // Initialize lower arm
        while (lower_sensor.getState()) {
            lowerArm.setTargetPosition(lowerArm.getCurrentPosition() - arm_down_step);
            lowerArm.setPower(arm_down_power);
        }
        lowerArm.setPower(0);
        lowerArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lowerArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        boolean turning = false;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            getGamePadValues();

            // Bot movement with Mecanum wheels
            power = fwd; //this can be tweaked for exponential power increase

            if ( turn != 0 ) {
                // We are turing
                turning = true;

                frontLeft.setPower(Range.clip(power + side - turn, -0.2, 0.2));
                frontRight.setPower(Range.clip(power - side + turn, -0.2, 0.2));
                backLeft.setPower(Range.clip(power - side - turn, -0.2, 0.2));
                backRight.setPower(Range.clip(power + side + turn, -0.2, 0.2));
            } else {
                // Just finished turning and let it settled down
                if ( turning ) {
                    sleep(500);

                    resetAngle();
                    turning = false;
                }

                // Moving in straight line (forward, backward or sideway) without turning; Add gyro assistance
//                correction = checkDirection();
                correction = 0;

                frontLeft.setPower(Range.clip(power + side + correction, -1, 1));
                frontRight.setPower(Range.clip(power - side - correction, -1, 1));
                backLeft.setPower(Range.clip(power - side + correction, -1, 1));
                backRight.setPower(Range.clip(power + side - correction, -1, 1));

            }


            // Arm movements
/*            if (dpad_up) {
                upperArm.setTargetPosition(upperArm.getCurrentPosition() + arm_up_step);
                upperArm.setPower(arm_up_power);
                upperArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            } else if (dpad_down) {
                upperArm.setTargetPosition(upperArm.getCurrentPosition() - arm_down_step);
                upperArm.setPower(arm_down_power);
                upperArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            } else
                upperArm.setPower(0.0);
*/
            if (lower_arm_stick < 0 && lowerArm.getCurrentPosition() < 3000) {
                lowerArm.setTargetPosition(lowerArm.getCurrentPosition() + arm_up_step);
                lowerArm.setPower(arm_up_power);
  //              lowerArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            } else if (lower_arm_stick > 0 && lower_sensor.getState()) {
                lowerArm.setTargetPosition(lowerArm.getCurrentPosition() - arm_down_step);
                lowerArm.setPower(arm_down_power);
  //              lowerArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            } else
                lowerArm.setPower(0.0);


            // Use gamepad left & right Bumpers to open and close the claw
            if (left_bumper) {
                clawOffset += claw_speed;
                clawExpanded = true;
            } else if (right_bumper) {
                clawOffset -= claw_speed;
                clawExpanded = true;
            }

            // Move both servos to new position.  Assume servos are mirror image of each other.
            if ( clawExpanded ) {
                clawOffset = Range.clip(clawOffset, -0.35, 0.35);
                leftClaw.setPosition(0.65 + clawOffset);
                rightClaw.setPosition(0.65 - clawOffset);
            }


            // Use gamepad X & right B buttons to rotate the grabber
            if (/* grabber_stick < 0*/ btn_y)
                grabberOffset += grabber_speed;
            else if (/*grabber_stick > 0*/ btn_a)
                grabberOffset -= grabber_speed;

            // Move grabber servo to the desired position.
            grabberOffset = Range.clip(grabberOffset, -0.02, 0.02);

            // Only set position when grabber stick is not pushed OR grabberOffset is 0.05
            if (( !btn_y && !btn_a )|| Math.abs(grabberOffset) == 0.02 ) {
                double target = grabber.getPosition() + grabberOffset;
                if (target < grabber_lower)
                    target = grabber_lower;
                if (target > grabber_upper)
                    target = grabber_upper;
                grabber.setPosition(target);
                grabberOffset = 0.0;
            }

            if ( grabber_stick < 0 )
                grabberOffset2 += grabber_speed * 5;
            else if ( grabber_stick > 0 )
                grabberOffset2 -= grabber_speed * 5;

            // Move grabber servo to the desired position.
            grabberOffset2 = Range.clip(grabberOffset2, 0.0, 1.0);

            // Only set position when grabber stick is not pushed OR grabberOffset is 0.05
            grabber2.setPosition(grabberOffset2);

//            telemetry.addData("Motor Power FL ", frontLeft.getPower());
//            telemetry.addData("Motor Power FR", frontRight.getPower());
//            telemetry.addData("Motor Power BL", backLeft.getPower());
//            telemetry.addData("Motor Power BR", backRight.getPower());
//            telemetry.addData("Status", "Running");
            telemetry.addData("LClaw= ", leftClaw.getPosition());
            telemetry.addData("RClaw= ", rightClaw.getPosition());
            telemetry.addData("CLOFF= ", clawOffset);
            telemetry.addData("BR= ", backRight.getCurrentPosition());
            telemetry.addData("ARM=", lowerArm.getCurrentPosition());
            telemetry.update();
        }
    }

    void getGamePadValues() {
        fwd = gamepad1.left_stick_y;
        side = gamepad1.left_stick_x;
        turn = gamepad1.right_stick_x;

        lower_arm_stick = gamepad2.left_stick_y;
        grabber_stick = gamepad2.right_stick_y;
/*
        dpad_up = gamepad1.dpad_up;
        dpad_down = gamepad1.dpad_down;
*/
        btn_y = gamepad2.y;
        btn_a = gamepad2.a;

/*
        btn_x = gamepad1.x;
        btn_b = gamepad1.b;
*/

        left_bumper = gamepad2.left_bumper;
        right_bumper = gamepad2.right_bumper;

        //updates joystick values
        if( Math.abs(fwd) < deadzone ) fwd = 0;
        if( Math.abs(side) < deadzone ) side = 0;
        if( Math.abs(turn) < deadzone ) turn = 0;
        if( Math.abs(lower_arm_stick) < deadzone ) lower_arm_stick = 0;
        if( Math.abs(grabber_stick) < deadzone ) grabber_stick = 0;
        //checks deadzones
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
        double correction, angle, gain = 0.03;

        angle = getAngle();

        if (angle < 5 && angle > -5)
            correction = 0;             // no adjustment.
        else
            correction = -angle;        // reverse sign of angle for correction.

        correction = correction * gain;

        return correction;
    }
}
