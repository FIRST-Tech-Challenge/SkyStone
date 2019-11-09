package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

// THIS IS NOT AN OPMODE - IT IS A DEFINING CLASS

public class Robot {

    // Motors
    public DcMotor rearLeft;
    public DcMotor rearRight;
    public DcMotor frontLeft;
    public DcMotor frontRight;
    public DcMotor waffleMover;
    public DcMotor armRotate;
    public DcMotor liftMotor;

    // Servos
    Servo gripperRotateServo1;
    Servo gripperRotateServo2;
    private Servo grabServo;

    // Sensors
    NavxMicroNavigationSensor navxMicro;

    // Constants
    private int TORQUENADO60TICKS_PER_REV = 1440; // ticks / rev
    private int ANGLE_OF_GRIPPER_WHEN_GRABBING = 30; // in degrees
    private double TORQUENADO20_TICKS_PER_REV = 480; // ticks / rev
    private double WHEEL_DIAMETER = 4;
    private double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI; // in / rev
    private double TICKS_PER_INCH = TORQUENADO20_TICKS_PER_REV / WHEEL_CIRCUMFERENCE; // ticks / in
    double ROBOT_EXTENDED_LENGTH = 36.0; // in
    double ROBOT_RETRACTED_LENGTH = 18.0; // in

    // PID Controller
    double p = 0.0;
    double i = 0.0;
    double d = 0.0;
    PIDController PIDWrist1 = new PIDController(p, i, d);
    PIDController PIDWrist2 = new PIDController(p, i, d);

    // info
    private int wafflePosition = -1; // 1 = Up, -1 = Down Waffle mover starts down
    private double wafflePower = 0.5;

    private double gripperRotatePosition = 0.4; // 0.4 = at a 90 degree angle, 0.6 = parallel to ground

    private enum gripperPosition {OPEN, CLOSED}
    private gripperPosition gripperPos = gripperPosition.OPEN;

    public enum armPosition {REST, ACTIVE}
    public armPosition armPos = armPosition.REST; // will change back to REST later

    private HardwareMap hwMap = null;

    public Robot () {
        // Constructor
    }

    void init (OpMode opmode) throws InterruptedException {
        /* Initializes the robot */

        hwMap = opmode.hardwareMap;

        // Motor mapping
        this.rearLeft = hwMap.dcMotor.get("rearLeft");
        this.frontLeft = hwMap.dcMotor.get("frontLeft");
        this.rearRight = hwMap.dcMotor.get("rearRight");
        this.frontRight = hwMap.dcMotor.get("frontRight");
        this.waffleMover = hwMap.dcMotor.get("waffleMover");
        this.armRotate = hwMap.dcMotor.get("armRotate");
        this.liftMotor = hwMap.dcMotor.get("liftMotor");

        // Drive Motor Direction
        this.rearLeft.setDirection(DcMotor.Direction.FORWARD);
        this.frontLeft.setDirection(DcMotor.Direction.FORWARD);
        this.rearRight.setDirection(DcMotor.Direction.REVERSE);
        this.frontRight.setDirection(DcMotor.Direction.REVERSE);
        this.waffleMover.setDirection(DcMotor.Direction.FORWARD);
        this.armRotate.setDirection(DcMotor.Direction.REVERSE); // positive makes arm go forward
        this.liftMotor.setDirection(DcMotor.Direction.FORWARD);

        // set motor powers to 0 so they don't cause problems
        this.stopDrive();
        this.waffleMover.setPower(0);
        this.armRotate.setPower(0);
        this.liftMotor.setPower(0);

        // Servo mapping
        this.gripperRotateServo1 = hwMap.get(Servo.class, "gripperRotateServo1");
        this.gripperRotateServo2 = hwMap.get(Servo.class, "gripperRotateServo2");
        this.grabServo = hwMap.get(Servo.class, "grabServo");

        // Servo direction
        this.gripperRotateServo1.setDirection(Servo.Direction.FORWARD);
        this.gripperRotateServo2.setDirection(Servo.Direction.REVERSE);
        this.grabServo.setDirection(Servo.Direction.FORWARD);

        // NavX Gyro Init
        //this.initNavXGyro(opmode);

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


    void driveForwardDistance(double distance, double power, LinearOpMode opmode) { // make power negative to go backwards
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
        Thread.sleep(600);
        this.waffleMover.setPower(0);

        this.wafflePosition *= -1;
    }

    private void moveArmRotate(int targetPosition, double power, OpMode opmode) {
        // reset encoders
        this.armRotate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // set mode
        this.armRotate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // set power
        this.setArmRotatePower(power);

        // PID parameters
        PIDWrist1.setSetpoint(targetPosition);
        PIDWrist1.setOutputRange(0, 0.5);
        PIDWrist1.setInputRange(0, 500);
        PIDWrist1.enable();

        PIDWrist2.setSetpoint(targetPosition);
        PIDWrist2.setOutputRange(0, 0.5);
        PIDWrist2.setInputRange(0, 500);
        PIDWrist2.enable();

        long timestart = System.nanoTime();
        // wait for the armRotate motors to reach the position or else things go bad bad
        while (Math.abs(this.armRotate.getCurrentPosition()) <  targetPosition) {

            // if it takes more than 2 seconds, something is wrong so we exit the loop
            if (System.nanoTime() - timestart > 2000000000) {
                opmode.telemetry.addData("Error", "Gripper movement took too long");
                opmode.telemetry.update();
                break;
            }

            double correction1 = PIDWrist1.performPID(Math.abs(this.armRotate.getCurrentPosition()));
            double correction2 = PIDWrist2.performPID(Math.abs(this.armRotate.getCurrentPosition()));

            this.armRotate.setPower(this.armRotate.getPower() + correction1);
            opmode.telemetry.addData("Gripper", "#1: " + correction1 + " #2: " + correction2);
            opmode.telemetry.update();
        }

        PIDWrist1.reset();
        PIDWrist2.reset();

        // stop the armRotate motors
        this.stopArmRotate();
    }

    void rotateGripper(double position) {
        this.gripperRotateServo1.setPosition(position);
        //this.gripperRotateServo2.setPosition(position);
    }

    void bringArmDown(OpMode opmode) throws InterruptedException {
        if (armPos == armPosition.REST) { // we only bring the arm down if the arm is resting
            // we rotate the arm 180 + ANGLE_OF_GRIPPER_WHEN_GRABBING degrees
            this.moveArmRotate(TORQUENADO60TICKS_PER_REV * (160) / 360, 0.6, opmode);
            // since gravity is pushing on the arm, we fight it so the arm gradually goes down and holds its position
            this.setArmRotatePower(-0.5);
            Thread.sleep(500);
            this.stopArmRotate();
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
            this.moveArmRotate(this.TORQUENADO60TICKS_PER_REV * (150) / 360, -0.7, opmode);
            this.setArmRotatePower(-0.4); // since gravity is pushing on the arm, we fight it so the arm gradually goes down
            Thread.sleep(500);
            this.stopArmRotate();
            this.armPos = armPosition.REST;
        }
    }

    void gripBlock() {
        this.grabServo.setPosition(0);
        this.gripperPos = gripperPosition.CLOSED;
    }

    void releaseBlock(OpMode opmode) {
        this.grabServo.setPosition(1);
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
        this.armRotate.setPower(power);
    }

    void stopArmRotate() { this.setArmRotatePower(0); }

    String getInfo() {
        String output = "Arm Position: " + this.armPos + "\nWaffle Position: ";
        if (this.wafflePosition == -1) {
            output += "Down\nWrist Position: ";
        } else {
            output += "Up\nWrist Position: ";
        }

        if (this.gripperRotatePosition == 1) {
            output += "Up\nGripper Position: ";
        } else {
            output += "Down\nGripper Position: ";
        }

        output += this.gripperPos;

        return output;
    }

    void toggleArmRotate() {
        this.rotateGripper(this.gripperRotatePosition);
        this.gripperRotatePosition = 1 - this.gripperRotatePosition;
    }

    void initNavXGyro(OpMode opmode) throws InterruptedException {
        // A timer helps provide feedback while calibration is taking place
        ElapsedTime timer = new ElapsedTime();

        navxMicro = opmode.hardwareMap.get(NavxMicroNavigationSensor.class, "navx");
        // If you're only interested in the IntegratingGyroscope interface, the following will suffice.
        // gyro = hardwareMap.get(IntegratingGyroscope.class, "navx");

        // The gyro automatically starts calibrating. This takes a few seconds.
        opmode.telemetry.log().add("Gyro Calibrating. DO NOT MOVE or else the Cookie Monster will come for your soul!");

        // Wait until the gyro calibration is complete
        timer.reset();
        while (navxMicro.isCalibrating())  {
            opmode.telemetry.addData("calibrating", "%s", Math.round(timer.seconds())%2==0 ? "|.." : "..|");
            opmode.telemetry.update();
            Thread.sleep(50);
        }
        opmode.telemetry.log().clear(); opmode.telemetry.log().add("Gyro Calibrated. Press Start.");
        opmode.telemetry.clear(); opmode.telemetry.update();
    }

    double getAngle() {
        Orientation angles = navxMicro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }
}