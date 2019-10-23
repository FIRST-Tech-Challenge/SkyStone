package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@Autonomous(name = "Vuforia Tester")
public class VuforiaTester extends LinearOpMode {

    DcMotor motorFL;
    DcMotor motorFR;
    DcMotor motorBL;
    DcMotor motorBR;

    final double calibFL = 1.00f;
    final double calibFR = 1.00f;
    final double calibBL = 1.00f;
    final double calibBR = 1.00f;

    private ElapsedTime runtime = new ElapsedTime();

    final double COUNTS_PER_MOTOR_REV = 1440;    // eg: TETRIX Motor Encoder
    final double DRIVE_GEAR_REDUCTION = 2.0;     // This is < 1.0 if geared UP
    final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    final double DRIVE_SPEED = 0.7;
    final double TURN_SPEED = 0.4;

    private final String[] actions = {"forward", "backward", "left", "right", "rotate"};

    private enum Action {
        FORWARD,
        BACKWARD,
        LEFT,
        RIGHT,
        ROTATE
    }

    VuforiaSensorSkystone vSensor;

    public VuforiaTester() {
        super();
    }

    @Override
    public void runOpMode() {
        vSensor = new VuforiaSensorSkystone(telemetry, hardwareMap);
        vSensor.setUpVuforia();
        vSensor.activate();
        initMotors();

        waitForStart();

        //move up to the bricks
        doAction(Action.FORWARD, 4.0);

        //find a skystone
        int brickCount = 0;
        while (brickCount < 7 || !vSensor.listenerSkystone.isVisible() || Math.abs(vSensor.getTransX(vSensor.lastKnownLocationSkystone)) > 15.0) {
            vSensor.loop();
            straifLeftRaw(0.4);
            sleep(100);
            brickCount++;
            telemetry.update();
        }

        //grab the skystone
        /*
        missing code for grabbing mechanism
         */

        //move to the building zone side of the field.
        doAction(Action.BACKWARD, 1.0);
        doAction(Action.RIGHT, 7.0 + (0.5 * brickCount));

        //release stone
        doAction(Action.LEFT, 7.0 + (0.5 * brickCount));
        doAction(Action.FORWARD, 1.0);






        stop();

    }

    public void moveForwardRaw(double power) {
        motorFL.setPower(Range.clip(calibFL * power, -1, 1));
        motorFR.setPower(Range.clip(calibFR * power, -1, 1));
        motorBL.setPower(Range.clip(calibBL * power, -1, 1));
        motorBR.setPower(Range.clip(calibBR * power, -1, 1));
    }

    public void rotateLeftRaw(double power) {
        motorFL.setPower(calibFL * -power);
        motorFR.setPower(calibFR * power);
        motorBL.setPower(calibBL * -power);
        motorBR.setPower(calibBR * power);
    }

    public void straifLeftRaw(double power) {
        motorFL.setPower(calibFL * power);
        motorFR.setPower(calibFR * power);
        motorBL.setPower(calibBR * -power);
        motorBR.setPower(calibBR * -power);
    }

    public void doAction(Action action, double distance) {
        switch (action) {
            case FORWARD:
                moveForward(distance);
                break;
            case BACKWARD:
                moveForward(-distance);
                break;
            case LEFT:
                straifLeft(distance);
                break;
            case RIGHT:
                straifLeft(-distance);
                break;
            case ROTATE:
                rotateLeft(distance);
                break;
        }
    }

    public void moveForward(double distance) {
        encoderDrive(DRIVE_SPEED, distance, distance, 10, false);
    }

    public void rotateLeft(double distance) {
        encoderDrive(TURN_SPEED, -distance, distance, 10, false);
    }

    public void straifLeft(double distance) {
        encoderDrive(DRIVE_SPEED, -distance, -distance, 10, true);
    }

    public void initMotors() {
        motorFL = hardwareMap.get(DcMotor.class, "frontLeft"); // frontLeft
        motorFR = hardwareMap.get(DcMotor.class, "frontRight"); // frontRight
        motorBL = hardwareMap.get(DcMotor.class, "backLeft"); // backLeft
        motorBR = hardwareMap.get(DcMotor.class, "backRight"); // backRight

        motorFL.setDirection(DcMotorSimple.Direction.FORWARD);
        motorFR.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBL.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBR.setDirection(DcMotorSimple.Direction.REVERSE);

        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS,
                             boolean straif) {
        int newFL;
        int newFR;
        int newBL;
        int newBR;

        int straifCoef = 1;
        if (straif) {
            straifCoef = -1;
        }

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newFL = motorFL.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH * straifCoef);
            newFR = motorFR.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH * straifCoef);
            newBL = motorBL.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newBR = motorBR.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);

            motorFL.setTargetPosition(newFL);
            motorFR.setTargetPosition(newFR);
            motorBL.setTargetPosition(newBL);
            motorBR.setTargetPosition(newBR);

            // Turn On RUN_TO_POSITION
            motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            motorFL.setPower(Math.abs(speed) * calibFL);
            motorFR.setPower(Math.abs(speed) * calibFR);
            motorBL.setPower(Math.abs(speed) * calibBL);
            motorBR.setPower(Math.abs(speed) * calibBR);

            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (motorFL.isBusy() || motorFR.isBusy() || motorBL.isBusy() || motorBR.isBusy())) {
                vSensor.loop();
            }

            // Stop all motion;
            motorFL.setPower(0.0);
            motorFR.setPower(0.0);
            motorBL.setPower(0.0);
            motorBR.setPower(0.0);

            // Turn off RUN_TO_POSITION
            motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


            sleep(1000);   // optional pause after each move
        }
    }

}