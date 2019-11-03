package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.lang.Thread;

// THIS IS NOT AN OPMODE - IT IS A DEFINING CLASS

public class Robot {

    // Motors
    public DcMotor rearLeft;
    public DcMotor rearRight;
    public DcMotor frontLeft;
    public DcMotor frontRight;
    public DcMotor waffleMover;
    public DcMotor armRotate1;
    public DcMotor armRotate2;
    public DcMotor liftMotor;

    // Servos
    private Servo gripperRotateServo1;
    private Servo gripperRotateServo2;
    private Servo grabServo;

    // Constants
    private int CORE_HEX_TICKS_PER_REV = 288; // ticks / rev
    private int ANGLE_OF_GRIPPER_WHEN_GRABBING = 45; // in degrees
    private double ANDYMARK_TICKS_PER_REV = 537.6; // ticks / rev
    private double WHEEL_DIAMETER = 4;
    private double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI; // in / rev
    private double TICKS_PER_INCH = ANDYMARK_TICKS_PER_REV / WHEEL_CIRCUMFERENCE; // ticks / in
    double ROBOT_EXTENDED_LENGTH = 36.0; // in
    double ROBOT_RETRACTED_LENGTH = 18.0; // in

    // info
    private int wafflePosition = -1; // 1 = Up, -1 = Down Waffle mover starts down
    private double wafflePower = 0.5;

    private int gripperRotatePosition = 1; // 1 = at a 90 degree angle, -1 = whatever angle it will be

    private enum gripperPosition {OPEN, CLOSED}
    private gripperPosition gripperPos = gripperPosition.OPEN;

    private enum armPosition {REST, ACTIVE}
    private armPosition armPos = armPosition.REST;

    private HardwareMap hwMap = null;

    public Robot () {
        // Constructor
    }

    void init (HardwareMap ahwMap) {
        /* Initializes the robot */

        hwMap = ahwMap;

        // Motor mapping
        this.rearLeft = hwMap.dcMotor.get("rearLeft");
        this.frontLeft = hwMap.dcMotor.get("frontLeft");
        this.rearRight = hwMap.dcMotor.get("rearRight");
        this.frontRight = hwMap.dcMotor.get("frontRight");
        this.waffleMover = hwMap.dcMotor.get("waffleMover");
        this.armRotate1 = hwMap.dcMotor.get("armRotate1");
        this.armRotate2 = hwMap.dcMotor.get("armRotate2");
        this.liftMotor = hwMap.dcMotor.get("liftMotor");

        // Drive Motor Direction
        this.rearLeft.setDirection(DcMotor.Direction.FORWARD);
        this.frontLeft.setDirection(DcMotor.Direction.FORWARD);
        this.rearRight.setDirection(DcMotor.Direction.REVERSE);
        this.frontRight.setDirection(DcMotor.Direction.REVERSE);
        this.waffleMover.setDirection(DcMotor.Direction.FORWARD);
        this.armRotate1.setDirection(DcMotor.Direction.REVERSE); // positive makes arm go forward
        this.armRotate2.setDirection(DcMotor.Direction.REVERSE);
        this.liftMotor.setDirection(DcMotor.Direction.FORWARD);

        // set motor powers to 0 so they don't cause problems
        this.stopDrive();
        this.waffleMover.setPower(0);
        this.armRotate1.setPower(0);
        this.armRotate2.setPower(0);
        this.liftMotor.setPower(0);

        // Zero power behavior
        this.armRotate1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.armRotate2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Servo mapping
        this.gripperRotateServo1 = hwMap.get(Servo.class, "gripperRotateServo1");
        this.gripperRotateServo2 = hwMap.get(Servo.class, "gripperRotateServo2");
        this.grabServo = hwMap.get(Servo.class, "grabServo");

        // Servo direction
        this.gripperRotateServo1.setDirection(Servo.Direction.REVERSE);
        this.gripperRotateServo2.setDirection(Servo.Direction.FORWARD);
        this.grabServo.setDirection(Servo.Direction.FORWARD);

    }

    void setDrivePower(double power) {
        /* sets all drive motors to a certain power */
        this.rearLeft.setPower(power);
        this.frontLeft.setPower(power);
        this.rearRight.setPower(power);
        this.frontRight.setPower(power);
    }

    void setDriveMode(DcMotor.RunMode runMode) {
        /* sets all drive motors to a certain mode */
        this.rearLeft.setMode(runMode);
        this.frontLeft.setMode(runMode);
        this.rearRight.setMode(runMode);
        this.frontRight.setMode(runMode);
    }

    void stopDrive() {
        /* stops all the drive motors */
        this.setDrivePower(0);
    }


    void driveForwardDistance(double distance, double power, LinearOpMode opmode) {
        /* drives forward a certain distance(in) using encoders */

        // calculate ticks
        long NUM_TICKS_LONG = StrictMath.round(this.TICKS_PER_INCH * distance);
        int NUM_TICKS = (int) NUM_TICKS_LONG;

        // reset encoders
        this.setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // set mode
        this.setDriveMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // set power
        this.setDrivePower(power);

        // drive
        while (opmode.opModeIsActive() && Math.abs(this.rearLeft.getCurrentPosition()) < NUM_TICKS && Math.abs(this.frontLeft.getCurrentPosition()) < NUM_TICKS
        && Math.abs(this.rearRight.getCurrentPosition()) < NUM_TICKS && Math.abs(this.frontRight.getCurrentPosition()) < NUM_TICKS) {
            // wait until target position is reached
            opmode.telemetry.addData("Target Position", NUM_TICKS);
            opmode.telemetry.addData("Rear Left", this.rearLeft.getCurrentPosition());
            opmode.telemetry.addData("Rear Right", this.rearRight.getCurrentPosition());
            opmode.telemetry.addData("Front Left", this.frontLeft.getCurrentPosition());
            opmode.telemetry.addData("Front Right", this.frontRight.getCurrentPosition());
            opmode.telemetry.update();
        }

        // stop driving
        this.stopDrive();

    }

    void setStrafe(double power) {
        /* strafes at certain power
        positive power goes to the right
        negative power goes to the left */
        this.rearLeft.setPower(-power);
        this.frontRight.setPower(-power);

        this.frontLeft.setPower(power);
        this.rearRight.setPower(power);
    }

    void strafeTime(double power, long milliseconds) throws InterruptedException {
        /* strafes for a certain amount of milliseconds */
        this.setStrafe(power);
        Thread.sleep(milliseconds);
        this.stopDrive();
    }

    void turnRight(double power, long milliseconds) throws InterruptedException {
        this.rearLeft.setPower(power);
        this.frontLeft.setPower(power);

        this.rearRight.setPower(-power);
        this.frontRight.setPower(-power);
        Thread.sleep(milliseconds);
        this.stopDrive();
    }

    void moveWaffleMover(char floatOrHold) throws InterruptedException {
        if (floatOrHold != 'f' && floatOrHold != 'h') { // make sure floatOrHold is 'f' or 'h' or else the cookie monster will come for you :)
            return;
        } else if (floatOrHold == 'h') {
            this.waffleMover.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        } else {
            this.waffleMover.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }

        this.waffleMover.setPower(this.wafflePower * this.wafflePosition);
        Thread.sleep(250);
        this.waffleMover.setPower(0);

        this.wafflePosition *= -1;
    }

    private void moveArmRotate(int targetPosition, double power, OpMode opmode) {
        // reset encoders
        this.armRotate1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.armRotate2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // set mode
        this.armRotate1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.armRotate2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // set power
        this.setArmRotatePower(power);

        // wait for the armRotate motors to reach the position or else things go bad bad
        while (Math.abs(this.armRotate1.getCurrentPosition()) <  targetPosition &&
                Math.abs(this.armRotate2.getCurrentPosition()) < targetPosition) {
            this.setArmRotatePower(power * Math.pow(1 - (Math.abs(this.armRotate1.getCurrentPosition()) / targetPosition), 2));
            opmode.telemetry.addData("Gripper", "#1: " + this.armRotate1.getCurrentPosition() + " #2: " + this.armRotate2.getCurrentPosition());
            opmode.telemetry.update();
        }

        // stop the armRotate motors
        this.stopArmRotate();
    }

    void rotateGripper(double angle) {
        this.gripperRotateServo1.setPosition(angle / 360);
        this.gripperRotateServo2.setPosition(angle / 360);
    }

    void bringArmDown(OpMode opmode) throws InterruptedException {
        if (armPos == armPosition.REST) { // we only bring the arm down if the arm is resting
            // we rotate the arm 180 + ANGLE_OF_GRIPPER_WHEN_GRABBING degrees
            this.moveArmRotate(CORE_HEX_TICKS_PER_REV * (150) / 360, 0.7, opmode);
            this.setArmRotatePower(-0.4); // since gravity is pushing on the arm, we fight it so the arm gradually goes down
            this.armPos = armPosition.ACTIVE;
        }
    }

    void foldArmBack(OpMode opmode) throws InterruptedException {
        if (this.armPos == armPosition.ACTIVE) { // we only do something if the arm is active
            if (this.gripperRotatePosition == -1) {
                // we rotate the gripper so it is perpendicular to the ground
                this.rotateGripper(this.ANGLE_OF_GRIPPER_WHEN_GRABBING - 90);
            }

            // we rotate the arm 110 degrees
            this.moveArmRotate(this.CORE_HEX_TICKS_PER_REV * (150) / 360, -0.7, opmode);
            this.setArmRotatePower(-0.4); // since gravity is pushing on the arm, we fight it so the arm gradually goes down
            Thread.sleep(500);
            this.stopArmRotate();
            this.armPos = armPosition.REST;
        }
    }

    void gripBlock() {
        this.grabServo.setPosition(0.25);
        this.gripperPos = gripperPosition.CLOSED;
    }

    void releaseBlock(OpMode opmode) {
        this.grabServo.setPosition(0);
        this.gripperPos = gripperPosition.OPEN;
    }

    void pickUpBlock(OpMode opmode) throws InterruptedException { // for autonomous
        this.bringArmDown(opmode); // bring arm down
        Thread.sleep(500);
        // we rotate the gripper so it is parallel to the ground
        this.rotateGripper(90 - this.ANGLE_OF_GRIPPER_WHEN_GRABBING);
        this.gripBlock(); // grab the block
    }

    void liftUp() { this.liftMotor.setPower(0.5); }

    void liftDown() { this.liftMotor.setPower(-0.5); }

    void stopLift() { this.liftMotor.setPower(0); }

    int detectSkystone(LinearOpMode opmode) {
        WebcamTest detector = new WebcamTest();
        return detector.detectSkystonePosition(opmode);
    }

    void setArmRotatePower(double power) {
        this.armRotate1.setPower(power);
        this.armRotate2.setPower(power);
    }

    void stopArmRotate() { this.setArmRotatePower(0); }

    String getInfo() {
        String output = "Arm Position: " + this.armPos + ", Waffle Position: ";
        if (this.wafflePosition == -1) {
            output += "Down. Wrist Position: ";
        } else {
            output += "Up, Wrist Position: ";
        }

        if (this.gripperRotatePosition == 1) {
            output += "Up, Gripper Position: ";
        } else {
            output += "Down, Gripper Position: ";
        }

        output += this.gripperPos;

        return output;
    }

    void toggleArmRotate() {
        this.rotateGripper(this.gripperRotatePosition * (90 - this.ANGLE_OF_GRIPPER_WHEN_GRABBING));
        this.gripperRotatePosition *= -1;
    }
}