package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.concurrent.TimeUnit;

// THIS IS NOT AN OPMODE - IT IS A DEFINING CLASS
public class Robot {

    // armRotate encoder count: -3650

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
    IntegratingGyroscope gyro;
    NavxMicroNavigationSensor navxMicro;
    BNO055IMU imu;

    DistanceSensor frontDistance, leftDistance, rightDistance, rearDistance;
    ColorSensor leftColor, insideColor;
    TouchSensor rearTouch;

    // Vuforia
    WebcamTest detector;

    // Constants
    private int TORQUENADO60TICKS_PER_REV = 1440; // ticks / rev
    private int ANGLE_OF_GRIPPER_WHEN_GRABBING = 30; // in degrees
    private double TORQUENADO20_TICKS_PER_REV = 480; // ticks / rev
    private double WHEEL_DIAMETER = 4;
    private double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI; // in / rev
    private double TICKS_PER_INCH = TORQUENADO20_TICKS_PER_REV / WHEEL_CIRCUMFERENCE; // ticks / in
    double ROBOT_EXTENDED_LENGTH = 30.0; // in
    double ROBOT_RETRACTED_LENGTH = 18.0; // in

    // PID Wrist Controller
    double pWrist = 0.0;
    double iWrist = 0.0;
    double dWrist = 0.0;
    PIDController PIDWrist = new PIDController(pWrist, iWrist, dWrist);

    // PID Drive Controller
    double pDrive = 0.0;
    double iDrive = 0.0;
    double dDrive = 0.0;
    PIDController PIDDrive = new PIDController(pDrive, iDrive, dDrive);

    // info
    private int wafflePosition = -1; // 1 = Up, -1 = Down Waffle mover starts down
    private double wafflePower = 0.5;

    private double gripperRotatePosition = 0.8; // 0.8 = at a 90 degree angle, 0.5 = parallel to ground

    private enum gripperPosition {OPEN, CLOSED}
    private gripperPosition gripperPos = gripperPosition.OPEN;

    public enum armPosition {REST, ACTIVE}
    public armPosition armPos = armPosition.REST;
    Orientation angles;



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

        // Motor encoder reset
        this.armRotate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.armRotate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // set motor powers to 0 so they don't cause problems
        this.stopDrive();
        this.waffleMover.setPower(0);
        this.armRotate.setPower(0);
        this.liftMotor.setPower(0);

        // Servo mapping
        this.gripperRotateServo1 = hwMap.get(Servo.class, "gripperRotateServo1");
        this.grabServo = hwMap.get(Servo.class, "grabServo");

        // Servo direction
        this.gripperRotateServo1.setDirection(Servo.Direction.FORWARD);
        this.grabServo.setDirection(Servo.Direction.FORWARD);

        // Sensor init
        this.frontDistance = hwMap.get(DistanceSensor.class, "frontDistance");
        this.leftDistance = hwMap.get(DistanceSensor.class, "leftDistance");
        this.rightDistance = hwMap.get(DistanceSensor.class, "rightDistance");
        this.rearDistance = hwMap.get(DistanceSensor.class, "rearDistance");

        this.leftColor = hwMap.get(ColorSensor.class, "leftColor");
        this.insideColor = hwMap.get(ColorSensor.class, "insideColor");

        this.rearTouch = hwMap.get(TouchSensor.class, "rearTouch");

        // init imu
        this.initImu();

        // Vuforia init
        detector = new WebcamTest();
        detector.init(hwMap);

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
        double targetAngle = this.getHeading();

        // calculate ticks
        long NUM_TICKS_LONG = StrictMath.round(this.TICKS_PER_INCH * distance);
        int NUM_TICKS = (int) NUM_TICKS_LONG;

        // reset encoders
        this.setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // set mode
        this.setDriveMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // drive
        while (opmode.opModeIsActive() && Math.abs(this.rearLeft.getCurrentPosition()) < NUM_TICKS && Math.abs(this.frontLeft.getCurrentPosition()) < NUM_TICKS
        && Math.abs(this.rearRight.getCurrentPosition()) < NUM_TICKS && Math.abs(this.frontRight.getCurrentPosition()) < NUM_TICKS) {
            double currentAngle = this.getHeading();
            double error = Math.tanh((currentAngle - targetAngle) / 40);
            this.frontLeft.setPower(power + error);
            this.frontRight.setPower(power - error);
            this.rearLeft.setPower(power + error);
            this.rearRight.setPower(power - error);
            opmode.telemetry.addData("Angle of Robot", currentAngle);
            opmode.telemetry.update();
        }

        // stop driving
        this.stopDrive();

    }

    void driveWithDIstanceSensor(double distanceForSensor, double power, DistanceSensor distanceSensor, OpMode opmode) {
        this.setDrivePower(power);
        double distanceToBlock = distanceSensor.getDistance(DistanceUnit.INCH);
        while (distanceToBlock > 15) {
            opmode.telemetry.addData("Distance", distanceToBlock);
            opmode.telemetry.update();
            distanceToBlock = distanceSensor.getDistance(DistanceUnit.INCH);
        }
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
        double targetAngle = this.getHeading(); // you want to stay at this angle the whole time
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while (timer.time(TimeUnit.MILLISECONDS) < milliseconds) {
            double currentAngle = this.getHeading();
            double error = Math.tanh((currentAngle - targetAngle) / 20); // we have to constrain the error between -1 and 1
            this.frontLeft.setPower(power + error);
            this.frontRight.setPower(-power - error);
            this.rearLeft.setPower(-power - error);
            this.rearRight.setPower(power + error);
        }
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

    void turnWithImu(double power, double angle, OpMode opmode) {
        double currentAngle = this.getHeading();
        double targetAngle = currentAngle + angle;
        double sign = Math.signum(angle);
        this.rearLeft.setPower(-power * sign);
        this.frontLeft.setPower(-power * sign);
        this.rearRight.setPower(power * sign);
        this.frontRight.setPower(power * sign);
        while (currentAngle * sign <= targetAngle * sign) {
            currentAngle = this.getHeading();
            opmode.telemetry.addData("Angle of Robot", currentAngle);
            opmode.telemetry.update();
        }
        this.stopDrive();
    }

    void moveWaffleMover() throws InterruptedException {
        this.waffleMover.setPower(this.wafflePower * this.wafflePosition);
        Thread.sleep(600);
        this.waffleMover.setPower(0);
        this.wafflePosition *= -1;
    }

    private void moveArmRotate(int targetPosition, double power, OpMode opmode) {
        double sign = Math.signum(targetPosition);

        // reset encoders
        this.armRotate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // set mode
        this.armRotate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // set power
        this.setArmRotatePower(power);

        long timestart = System.nanoTime() / 1000000000;
        // wait for the armRotate motors to reach the position or else things go bad bad
        while (this.armRotate.getCurrentPosition() * sign <  targetPosition * sign) {
            // if it takes more than 2 seconds, something is wrong so we exit the loop
            if (System.nanoTime() / 1000000000 - timestart > 10) {
                opmode.telemetry.addData("Error", "Gripper movement took too long");
                opmode.telemetry.update();
                break;
            }

            opmode.telemetry.addData("Gripper", targetPosition + " " + this.armRotate.getCurrentPosition());
            opmode.telemetry.update();
        }

        // stop the armRotate motors
        this.stopArmRotate();
    }

    void rotateGripper(double position) {
        this.gripperRotateServo1.setPosition(position);
        //this.gripperRotateServo2.setPosition(position);
    }

    void bringArmDown(OpMode opmode) {
        if (armPos == armPosition.REST) { // we only bring the arm down if the arm is resting
            // we rotate the arm 180 + ANGLE_OF_GRIPPER_WHEN_GRABBING degrees
            this.moveArmRotate(-3100, 0.6, opmode);
            this.stopArmRotate();
            this.armPos = armPosition.ACTIVE;
        }
    }

    void foldArmBack(OpMode opmode) {
        if (this.armPos == armPosition.ACTIVE) { // we only do something if the arm is active
            if (this.gripperRotatePosition == -1) {
                // we rotate the gripper so it is perpendicular to the ground
                this.rotateGripper(0.5);
            }
            // we rotate the arm 225 degrees
            this.moveArmRotate(3100, -0.6, opmode);
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
        this.rotateGripper(0.5);
        this.gripBlock(); // grab the block
        Thread.sleep(500);
        // we rotate the gripper back
        this.rotateGripper(0.9);
    }

    void liftUp() { this.liftMotor.setPower(0.5); }

    void liftDown() { this.liftMotor.setPower(-0.5); }

    void stopLift() { this.liftMotor.setPower(0); }

    int detectSkystone(LinearOpMode opmode) {
        return detector.detectSkystonePosition(opmode);
    }

    void setArmRotatePower(double power) {
        this.armRotate.setPower(power);
    }

    void stopArmRotate() { this.setArmRotatePower(0); }

    String getInfo() {
        String output = "Arm Position: " + this.armRotate.getCurrentPosition() + "\nWaffle Position: ";
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

        output += "\nRobot Angle: " + this.getHeading();

        return output;
    }

    void toggleArmRotate() {
        this.rotateGripper(this.gripperRotatePosition);
        this.gripperRotatePosition = 1.3 - this.gripperRotatePosition;
    }

    void initNavXGyro(OpMode opmode) throws InterruptedException {
        // A timer helps provide feedback while calibration is taking place
        ElapsedTime timer = new ElapsedTime();

        // Get a reference to a Modern Robotics GyroSensor object. We use several interfaces
        // on this object to illustrate which interfaces support which functionality.
        navxMicro = opmode.hardwareMap.get(NavxMicroNavigationSensor.class, "navx");
        gyro = (IntegratingGyroscope)navxMicro;
        // If you're only interested in the IntegratingGyroscope interface, the following will suffice.
        // gyro = hardwareMap.get(IntegratingGyroscope.class, "navx");

        // The gyro automatically starts calibrating. This takes a few seconds.
        opmode.telemetry.log().add("Gyro Calibrating. DO NOT MOVE or else the Cookie Monster will eat your soul!");

        // Wait until the gyro calibration is complete
        timer.reset();
        while (navxMicro.isCalibrating())  {
            opmode.telemetry.addData("calibrating", "%s", Math.round(timer.seconds())%2==0 ? "|" : "-");
            opmode.telemetry.update();
            Thread.sleep(50);
        }
        opmode.telemetry.log().clear(); opmode.telemetry.log().add("Gyro Calibrated. Press Start.");
        opmode.telemetry.clear(); opmode.telemetry.update();

    }

    double getAngle() {
        Orientation angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }

    void grabBlockAuto() throws InterruptedException {
        // grab block
        this.rotateGripper(0.5);
        Thread.sleep(500);
        this.gripBlock();
        Thread.sleep(500);
        this.rotateGripper(0.8);
    }

    void stopEverything() {
        this.stopDrive();
        this.stopArmRotate();
        this.stopLift();
    }

    void initImu() {
        // imu init
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        this.imu = hwMap.get(BNO055IMU.class, "imu");
        this.imu.initialize(parameters);
    }

    double getHeading() {
        // get angles
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }

    void turnToGlobalPosition(double power, double angle, OpMode opmode) {
        double angleToTurn = angle - this.getHeading();
        this.turnWithImu(power, angleToTurn, opmode);
    }
}

