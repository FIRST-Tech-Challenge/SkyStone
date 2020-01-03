package org.firstinspires.ftc.teamcode.Control;

import com.qualcomm.hardware.bosch.BNO055IMUImpl;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.annotations.MotorType;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
import static org.firstinspires.ftc.teamcode.Control.Constants.COUNTS_PER_COREHEXMOTOR_INCH;
import static org.firstinspires.ftc.teamcode.Control.Constants.COUNTS_PER_GOBUILDA312RPM_INCH;
import static org.firstinspires.ftc.teamcode.Control.Constants.COUNTS_PER_GOBUILDA312RPM_ROT;
import static org.firstinspires.ftc.teamcode.Control.Constants.COUNTS_PER_INCH;
import static org.firstinspires.ftc.teamcode.Control.Constants.COUNTS_PER_MOTOR_GOBUILDA312RPM;
import static org.firstinspires.ftc.teamcode.Control.Constants.COUNTS_PER_MOTOR_REV;
import static org.firstinspires.ftc.teamcode.Control.Constants.backs;
import static org.firstinspires.ftc.teamcode.Control.Constants.colors;
import static org.firstinspires.ftc.teamcode.Control.Constants.extendos;
import static org.firstinspires.ftc.teamcode.Control.Constants.foundationServos1;
import static org.firstinspires.ftc.teamcode.Control.Constants.foundationServos2;
import static org.firstinspires.ftc.teamcode.Control.Constants.fronts;
import static org.firstinspires.ftc.teamcode.Control.Constants.imuS;
import static org.firstinspires.ftc.teamcode.Control.Constants.leftServos;
import static org.firstinspires.ftc.teamcode.Control.Constants.lefts;
import static org.firstinspires.ftc.teamcode.Control.Constants.leftsucks;
import static org.firstinspires.ftc.teamcode.Control.Constants.linearLimits;
import static org.firstinspires.ftc.teamcode.Control.Constants.motorBLS;
import static org.firstinspires.ftc.teamcode.Control.Constants.motorBRS;
import static org.firstinspires.ftc.teamcode.Control.Constants.motorFLS;
import static org.firstinspires.ftc.teamcode.Control.Constants.motorFRS;
import static org.firstinspires.ftc.teamcode.Control.Constants.rightLinears;
import static org.firstinspires.ftc.teamcode.Control.Constants.rightServos;
import static org.firstinspires.ftc.teamcode.Control.Constants.rights;
import static org.firstinspires.ftc.teamcode.Control.Constants.rightsucks;
import static org.firstinspires.ftc.teamcode.Control.Constants.rotationservos;
import static org.firstinspires.ftc.teamcode.Control.Constants.smallLSucks;
import static org.firstinspires.ftc.teamcode.Control.Constants.smallRSucks;

public class Crane {


    public Crane(HardwareMap hardwareMap, ElapsedTime runtime, Central central, setupType... setup) throws InterruptedException {
        this.hardwareMap = hardwareMap;
        this.runtime = runtime;
        this.central = central;

        StringBuilder i = new StringBuilder();

        for (setupType type: setup) {
            switch (type) {
                case imu:
                    setupIMU();
                    break;
                case ultrasoinc:
                    setupUltra();
                    break;
                case intake:
                    setupIntake();
                    break;
                case autonomous:
                    setupDrivetrain();
                    break;
                case drive:
                    setupDrivetrain();
                    break;
                case camera:
                    setupCamera();
                    break;
                case claw:
                    setupClaw();
                    break;
                case foundation:
                    setupFoundation();
                    break;
                case encoder:
                    setupEncoder();
                    break;
                case bSystem:
                    //setupRack();
                    //setupLinearSlides();
                    setupClaw();
                    break;
            }

            i.append(type.name()).append(" ");

        }
        central.telemetry.addLine(i.toString());
        central.telemetry.update();

    }

    // important non-configuration fields
    public ElapsedTime runtime;     //set in constructor to the runtime of running class
    public Central central;
    public HardwareMap hardwareMap;


    public int[] wheelAdjust = {1, 1, 1, 1};

    public static double speedAdjust = 20.0 / 41.0;
    public static double yToXRatio = 1.25;

    public void setWheelAdjust(int fr, int fl, int br, int bl) {
        wheelAdjust[0] = fr;
        wheelAdjust[1] = fl;
        wheelAdjust[2] = br;
        wheelAdjust[3] = bl;
    }
    //----specfic non-configuration fields
    //none rnh


    // Vuforia Variables
    public final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    public final boolean PHONE_IS_PORTRAIT = false;
    public final String VUFORIA_KEY =
             " AYzLd0v/////AAABmR035tu9m07+uuZ6k86JLR0c/MC84MmTzTQa5z2QOC45RUpRTBISgipZ2Aop4XzRFFIvrLEpsop5eEBl5yu5tJxK6jHbMppJyWH8lQbvjz4PAK+swG4ALuz2M2MdFXWl7Xh67s/XfIFSq1UJpX0DgwmZnoDCYHmx/MnFbyxvpWIMLZziaJqledMpZtmH11l1/AS0oH+mrzWQLB57w1Ur0FRdhpxcrZS9KG09u6I6vCUc8EqkHqG7T2Zm4QdnytYWpVBBu17iRNhmsd3Ok3w8Pn22blBYRo6dZZ8oscyQS1ZtilM1YT49ORQHc8mu/BMWh06LxdstWctSiGiBV0+Wn3Zk++xQ750c64lg3QLjNkXc";
    public final float mmPerInch        = 25.4f;
    public final float mmTargetHeight   = (6) * mmPerInch;
    public final float stoneZ = 2.00f * mmPerInch;
    public final float bridgeZ = 6.42f * mmPerInch;
    public final float bridgeY = 23 * mmPerInch;
    public final float bridgeX = 5.18f * mmPerInch;
    public final float bridgeRotY = 59;
    public final float bridgeRotZ = 180;
    public final float halfField = 72 * mmPerInch;
    public final float quadField  = 36 * mmPerInch;

    public OpenGLMatrix lastLocation = null;
    public VuforiaLocalizer vuforia = null;
    public WebcamName webcamName = null;

    //----------------CONFIGURATION FIELDS--------------------
    public DcMotor[] drivetrain;   //set in motorDriveMode() for drivetrain movement functions

    public DcMotor motorFR;
    public DcMotor motorFL;
    public DcMotor motorBR;
    public DcMotor motorBL;

    public DcMotor rightLinear;
    public DcMotor leftLinear;
    public DcMotor rack;

    public DcMotor rightSuck;
    public DcMotor leftSuck;

    public DcMotor encoderup;

    public  List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
    public VuforiaTrackables targetsSkyStone;
    public VuforiaTrackable stoneTarget;
    public boolean targetVisible = false;
    public OpenGLMatrix robotFromCamera;
    public int cameraMonitorViewId;
    public VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

    public Servo servo;

    public Servo rotationservo, rightServo, leftServo, foundationServo1, foundationServo2;
    public CRServo smallRSuck, smallLSuck;

    public DcMotor extend;

    public ModernRoboticsI2cRangeSensor front, back, left, right;

    public ModernRoboticsI2cColorSensor color;

    public DigitalChannel flimit;
    public DigitalChannel blimit;
    public DigitalChannel linearLimit, clawLimit;

    public double StrafetoTotalPower = 2.0/3.0;

    //----       IMU        ----

    public BNO055IMUImpl imu;
    public BNO055IMUImpl.Parameters imuparameters = new BNO055IMUImpl.Parameters();
    public Orientation current;
    public static boolean isnotstopped;
    public float initorient;

    public void setupIMU() throws InterruptedException{
        imuparameters.angleUnit = BNO055IMUImpl.AngleUnit.DEGREES;
        imuparameters.accelUnit = BNO055IMUImpl.AccelUnit.METERS_PERSEC_PERSEC;
        imuparameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        imuparameters.loggingEnabled = true; //copypasted from BNO055IMU sample code, no clue what this does
        imuparameters.loggingTag = "imu"; //copypasted from BNO055IMU sample code, no clue what this does
        imu = hardwareMap.get(BNO055IMUImpl.class, imuS);
        imu.initialize(imuparameters);
        initorient = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        central.telemetry.addData("IMU status", imu.getSystemStatus());
        central.telemetry.update();


    }

    public void setupDrivetrain() throws InterruptedException {
        motorFR = motor(motorFRS, DcMotorSimple.Direction.FORWARD, DcMotor.ZeroPowerBehavior.BRAKE);
        motorFL = motor(motorFLS, DcMotorSimple.Direction.FORWARD, DcMotor.ZeroPowerBehavior.BRAKE);
        motorBR = motor(motorBRS, DcMotorSimple.Direction.REVERSE, DcMotor.ZeroPowerBehavior.BRAKE);
        motorBL = motor(motorBLS, DcMotorSimple.Direction.REVERSE, DcMotor.ZeroPowerBehavior.BRAKE);

        motorDriveMode(EncoderMode.ON, motorFR, motorFL, motorBR, motorBL);
    }


    public void setupClaw() throws InterruptedException {
        //leftLinear = motor(leftLinears, DcMotorSimple.Direction.FORWARD, DcMotor.ZeroPowerBehavior.BRAKE);
        rightLinear = motor(rightLinears, DcMotorSimple.Direction.FORWARD, DcMotor.ZeroPowerBehavior.BRAKE);
        encoder(EncoderMode.ON, rightLinear);
        //leftServo = servo(leftServos, Servo.Direction.REVERSE,0,1,0.5);
       // linearLimit = hardwareMap.digitalChannel.get(linearLimits);
        extend = motor(extendos, DcMotorSimple.Direction.FORWARD, DcMotor.ZeroPowerBehavior.BRAKE);
        rightServo = servo(rightServos, Servo.Direction.FORWARD,0,1,.2);
        rotationservo = servo(rotationservos, Servo.Direction.FORWARD,0,1,1);
        clawLimit = hardwareMap.digitalChannel.get("clawLimit");


    }

    public void setupFoundation() throws InterruptedException{
        foundationServo1 = servo(foundationServos2, Servo.Direction.FORWARD,0,1,0);
        foundationServo2 = servo(foundationServos1, Servo.Direction.FORWARD,0,1,.6);
    }

    public void setupIntake() throws InterruptedException{
        rightSuck = motor(rightsucks, DcMotorSimple.Direction.FORWARD, DcMotor.ZeroPowerBehavior.BRAKE);
        leftSuck = motor(leftsucks, DcMotorSimple.Direction.FORWARD, DcMotor.ZeroPowerBehavior.BRAKE);

        smallRSuck = servo(smallRSucks, DcMotorSimple.Direction.FORWARD, 0);
        smallLSuck = servo(smallLSucks, DcMotorSimple.Direction.FORWARD, 0);

        encoder(EncoderMode.OFF, rightSuck, leftSuck);
    }

    public void setupEncoder() throws InterruptedException{
        encoderup = motor(rightsucks, DcMotorSimple.Direction.FORWARD, DcMotor.ZeroPowerBehavior.FLOAT);
        rightSuck = motor(rightsucks, DcMotorSimple.Direction.FORWARD, DcMotor.ZeroPowerBehavior.FLOAT);
        leftSuck = motor(leftsucks, DcMotorSimple.Direction.FORWARD, DcMotor.ZeroPowerBehavior.FLOAT);

        encoder(EncoderMode.ON, rightSuck, leftSuck, encoderup);
    }

    public void setupUltra() throws InterruptedException{
        front = ultrasonicSensor(fronts);
        back = ultrasonicSensor(backs);
        left = ultrasonicSensor(lefts);
        right = ultrasonicSensor(rights);
        color = MRColor(colors);

    }

    //-----------------------HARDWARE SETUP FUNCTIONS---------------------------------------
    public DcMotor motor(String name, DcMotor.Direction directionm, DcMotor.ZeroPowerBehavior zeroPowerBehavior) throws InterruptedException {
        DcMotor motor = hardwareMap.dcMotor.get(name);
        motor.setDirection(DcMotor.Direction.FORWARD);
        motor.setZeroPowerBehavior(zeroPowerBehavior);
        motor.setPower(0);
        return motor;
    }

    public Servo servo(String name, Servo.Direction direction, double min, double max, double start) throws InterruptedException {
        Servo servo = hardwareMap.servo.get(name);
        servo.setDirection(direction);
        servo.scaleRange(min, max);
        servo.setPosition(start);
        return servo;
    }
    public CRServo servo(String name, DcMotorSimple.Direction direction, double startSpeed) throws InterruptedException {
        CRServo servo = hardwareMap.crservo.get(name);
        servo.setDirection(direction);

        servo.setPower(startSpeed);
        return servo;
    }
    public ColorSensor colorSensor(String name, boolean ledOn) throws InterruptedException {
        ColorSensor sensor = hardwareMap.colorSensor.get(name);
        sensor.enableLed(ledOn);

        central.telemetry.addData("Beacon Red Value: ", sensor.red());
        central.telemetry.update();

        return sensor;
    }
    public ModernRoboticsI2cRangeSensor ultrasonicSensor(String name) throws InterruptedException {

        return hardwareMap.get(ModernRoboticsI2cRangeSensor.class, name);
    }

    public ModernRoboticsI2cColorSensor MRColor(String name) throws InterruptedException{
        return hardwareMap.get(ModernRoboticsI2cColorSensor.class, name);

    }

    public void encoder(EncoderMode mode, DcMotor... motor) throws InterruptedException {
        switch (mode) {
            case ON:
                for (DcMotor i : motor) {
                    i.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                }
                central.idle();
                for (DcMotor i : motor) {
                    i.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                }
                break;
            case OFF:
                break;
        }

    }

    public void motorDriveMode(EncoderMode mode, DcMotor... motor) throws InterruptedException {
        switch (mode) {
            case ON:
                for (DcMotor i : motor) {
                    i.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                }
                central.idle();
                for (DcMotor i : motor) {
                    i.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                }
                break;
            case OFF:
                break;
        }

        this.drivetrain = motor;

    }

    public void driveTrainEncoderMovement(double speed, double distance, double timeoutS, long waitAfter, movements movement) throws  InterruptedException{

        int[] targets = new int[drivetrain.length];
        double[] signs = movement.getDirections();

        // Ensure that the opmode is still active
        if (central.opModeIsActive()) {
            // Determine new target position, and pass to motor controller


            for (DcMotor motor : drivetrain){
                int x = Arrays.asList(drivetrain).indexOf(motor);
                targets[x] = motor.getCurrentPosition() + (int) (signs[x] * wheelAdjust[x] * distance * COUNTS_PER_GOBUILDA312RPM_INCH);
            }
            for (DcMotor motor: drivetrain){
                int x = Arrays.asList(drivetrain).indexOf(motor);
                motor.setTargetPosition(targets[x]);
            }
            for (DcMotor motor: drivetrain){
                motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            runtime.reset();

            for (DcMotor motor:drivetrain){
                motor.setPower(Math.abs(speed));
            }

            // keep looping while we are still active, and there is time left, and both motors are running.
            boolean x = true;
            while (central.opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (x)) {

                // Display it for the driver.
                // Allow time for other processes to run.
                central.idle();

                for (int i = 0; i < drivetrain.length; i++) {
                    DcMotor motor = drivetrain[i];
                    if (!motor.isBusy() && signs[i] != 0) {
                        x = false;
                    }
                }
            }

            // Stop all motion;
            for (DcMotor motor: drivetrain){
                motor.setPower(0);
            }

            // Turn off RUN_TO_POSITION
            for (DcMotor motor: drivetrain){
                motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            central.sleep(waitAfter);


        }
    }
    public void encoderMovement(double speed, double distance, double timeoutS, long waitAfter, movements movement, DcMotor... motors) throws  InterruptedException{

        int[] targets = new int[motors.length];
        double[] signs = movement.getDirections();

        // Ensure that the opmode is still active
        if (central.opModeIsActive()) {
            // Determine new target position, and pass to motor controller

            for (DcMotor motor : motors){
                int x = Arrays.asList(motors).indexOf(motor);
                targets[x] = motor.getCurrentPosition() + (int) (signs[x] * wheelAdjust[x] * distance * COUNTS_PER_GOBUILDA312RPM_INCH);
            }
            for (DcMotor motor: motors){
                int x = Arrays.asList(motors).indexOf(motor);
                motor.setTargetPosition(targets[x]);
            }
            for (DcMotor motor: motors){
                motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            runtime.reset();

            for (DcMotor motor:motors){
                motor.setPower(Math.abs(speed));
            }

            // keep looping while we are still active, and there is time left, and both motors are running.
            boolean x = true;
            while (central.opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (x)) {

                // Display it for the driver.
                // Allow time for other processes to run.
                central.idle();
                for (DcMotor motor: motors){
                    if (!motor.isBusy()){
                        x =false;
                    }
                }
            }

            // Stop all motion;
            for (DcMotor motor: motors){
                motor.setPower(0);
            }

            // Turn off RUN_TO_POSITION
            for (DcMotor motor: motors){
                motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            central.sleep(waitAfter);


        }
    }
    public void encodeCoreHexMovement(double speed, double distance, double timeoutS, long waitAfter, movements movement, DcMotor... motors) throws  InterruptedException{

        int[] targets = new int[motors.length];
        double[] signs = movement.getDirections();

        // Ensure that the opmode is still active
        if (central.opModeIsActive()) {
            // Determine new target position, and pass to motor controller

            for (DcMotor motor : motors){
                int x = Arrays.asList(motors).indexOf(motor);
                targets[x] = motor.getCurrentPosition() + (int) (signs[x] * wheelAdjust[x] * distance * COUNTS_PER_COREHEXMOTOR_INCH);
            }
            for (DcMotor motor: motors){
                int x = Arrays.asList(motors).indexOf(motor);
                motor.setTargetPosition(targets[x]);
            }
            for (DcMotor motor: motors){
                motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            runtime.reset();

            for (DcMotor motor:motors){
                motor.setPower(Math.abs(speed));
            }

            // keep looping while we are still active, and there is time left, and both motors are running.
            boolean x = true;
            while (central.opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (x)) {

                // Display it for the driver.
                // Allow time for other processes to run.
                central.idle();
                for (DcMotor motor: motors){
                    if (!motor.isBusy()){
                        x =false;
                    }
                }
            }

            // Stop all motion;
            for (DcMotor motor: motors){
                motor.setPower(0);
            }

            // Turn off RUN_TO_POSITION
            for (DcMotor motor: motors){
                motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            central.sleep(waitAfter);


        }
    }

    //------------------DRIVETRAIN TELEOP FUNCTIONS------------------------------------------------------------------------
    public void driveTrainMovement(double speed, movements movement) throws InterruptedException{
        double[] signs = movement.getDirections();
        for (DcMotor motor: drivetrain){
            int x = Arrays.asList(drivetrain).indexOf(motor);
            motor.setPower(signs[x] * wheelAdjust[x]* speed);

        }
    }
    public void driveTrainMovement(double... speed) throws InterruptedException{

        for (int i = 0; i < drivetrain.length; i++) {
            drivetrain[i].setPower(speed[i]);
        }
    }
    public void driveTrainTimeMovement(double speed, movements movement, long duration, long waitAfter) throws InterruptedException{
        double[] signs = movement.getDirections();
        for (DcMotor motor: drivetrain){
            int x = Arrays.asList(drivetrain).indexOf(motor);
            motor.setPower(signs[x] * wheelAdjust[x]* speed);

        }
        central.sleep(duration);
        stopDrivetrain();
        central.sleep(waitAfter);
    }

    public void anyMovement(double speed, movements movement, DcMotor... motors) throws InterruptedException{
        double[] signs = movement.getDirections();
        for (DcMotor motor: motors){
            int x = Arrays.asList(motors).indexOf(motor);
            motor.setPower(signs[x] * wheelAdjust[x]* speed);

        }
    }
    public void anyMovementTime(double speed, movements movement, long duration, DcMotor... motors) throws InterruptedException{
        double[] signs = movement.getDirections();
        for (DcMotor motor: motors){
            int x = Arrays.asList(motors).indexOf(motor);
            motor.setPower(signs[x] * wheelAdjust[x]* speed);

        }
        central.sleep(duration);
        for (DcMotor motor: motors){
            motor.setPower(0);

        }
    }
    public void stopDrivetrain() throws InterruptedException{
        for (DcMotor motor: drivetrain){
            motor.setPower(0);
        }
    }
    public void setupCamera() throws InterruptedException{

        float phoneXRotate    = 0;
        float phoneYRotate    = 0;
        float phoneZRotate    = 0;

        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");

        cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        parameters.vuforiaLicenseKey = VUFORIA_KEY;

        parameters.cameraName = webcamName;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        targetsSkyStone  = this.vuforia.loadTrackablesFromAsset("Skystone");

        // Load the data sets for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.

        stoneTarget = targetsSkyStone.get(0);
        stoneTarget.setName("Stone Target");
        VuforiaTrackable blueRearBridge = targetsSkyStone.get(1);
        blueRearBridge.setName("Blue Rear Bridge");
        VuforiaTrackable redRearBridge = targetsSkyStone.get(2);
        redRearBridge.setName("Red Rear Bridge");
        VuforiaTrackable redFrontBridge = targetsSkyStone.get(3);
        redFrontBridge.setName("Red Front Bridge");
        VuforiaTrackable blueFrontBridge = targetsSkyStone.get(4);
        blueFrontBridge.setName("Blue Front Bridge");
        VuforiaTrackable red1 = targetsSkyStone.get(5);
        red1.setName("Red Perimeter 1");
        VuforiaTrackable red2 = targetsSkyStone.get(6);
        red2.setName("Red Perimeter 2");
        VuforiaTrackable front1 = targetsSkyStone.get(7);
        front1.setName("Front Perimeter 1");
        VuforiaTrackable front2 = targetsSkyStone.get(8);
        front2.setName("Front Perimeter 2");
        VuforiaTrackable blue1 = targetsSkyStone.get(9);
        blue1.setName("Blue Perimeter 1");
        VuforiaTrackable blue2 = targetsSkyStone.get(10);
        blue2.setName("Blue Perimeter 2");
        VuforiaTrackable rear1 = targetsSkyStone.get(11);
        rear1.setName("Rear Perimeter 1");
        VuforiaTrackable rear2 = targetsSkyStone.get(12);
        rear2.setName("Rear Perimeter 2");


        allTrackables.addAll(targetsSkyStone);

        stoneTarget.setLocation(OpenGLMatrix
                .translation(0, 0, stoneZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        blueFrontBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, bridgeRotY, bridgeRotZ)));

        blueRearBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, -bridgeRotY, bridgeRotZ)));

        redFrontBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, -bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, -bridgeRotY, 0)));

        redRearBridge.setLocation(OpenGLMatrix
                .translation(bridgeX, -bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, bridgeRotY, 0)));

        red1.setLocation(OpenGLMatrix
                .translation(quadField, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        red2.setLocation(OpenGLMatrix
                .translation(-quadField, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        front1.setLocation(OpenGLMatrix
                .translation(-halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , 90)));

        front2.setLocation(OpenGLMatrix
                .translation(-halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90)));

        blue1.setLocation(OpenGLMatrix
                .translation(-quadField, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));

        blue2.setLocation(OpenGLMatrix
                .translation(quadField, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));

        rear1.setLocation(OpenGLMatrix
                .translation(halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , -90)));

        rear2.setLocation(OpenGLMatrix
                .translation(halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        if (CAMERA_CHOICE == BACK) {
            phoneYRotate = -90;
        } else {
            phoneYRotate = 90;
        }

        if (PHONE_IS_PORTRAIT) {
            phoneXRotate = 90 ;
        }

        final float CAMERA_FORWARD_DISPLACEMENT  = 4.0f * mmPerInch;   // eg: Camera is 4 Inches in front of robot-center
        final float CAMERA_VERTICAL_DISPLACEMENT = 8.0f * mmPerInch;   // eg: Camera is 8 Inches above ground
        final float CAMERA_LEFT_DISPLACEMENT     = 0;     // eg: Camera is ON the robot's center line


        robotFromCamera = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));
    }

    public void powerMotors(double speed, long time, DcMotor... motors) {
        for (DcMotor motor : motors) {
            motor.setPower(speed);
        }
        central.sleep(time);
        for (DcMotor motor : motors) {
            motor.setPower(0);
        }
    }

    // IMU Movements
    public void turn(float target, turnside direction, double speed, axis rotation_Axis) throws InterruptedException{

        central.telemetry.addData("IMU State: ", imu.getSystemStatus());
        central.telemetry.update();

        double start = getDirection();

        double end = (start + ((direction == turnside.cw) ? target : -target) + 360) % 360;

        isnotstopped = true;
        try {
            switch (rotation_Axis) {
                case center:
                    driveTrainMovement(speed, (direction == turnside.cw) ? movements.cw : movements.ccw);
                    break;
                case back:
                    driveTrainMovement(speed, (direction == turnside.cw) ? movements.cwback : movements.ccwback);
                    break;
                case front:
                    driveTrainMovement(speed, (direction == turnside.cw) ? movements.cwfront : movements.ccwfront);
                    break;
            }
        } catch (InterruptedException e) {
            isnotstopped = false;
        }

        while (((calculateDifferenceBetweenAngles(getDirection(), end) > 1 && turnside.cw == direction) || (calculateDifferenceBetweenAngles(getDirection(), end) < -1 && turnside.ccw == direction)) && central.opModeIsActive() ) {
            central.telemetry.addLine("First Try ");
            central.telemetry.addData("IMU Inital: ", start);
            central.telemetry.addData("IMU Final Projection: ", end);
            central.telemetry.addData("IMU Orient: ", getDirection());
            central.telemetry.addData("IMU Difference: ", end - getDirection());
            central.telemetry.update();
        }
        try {
            stopDrivetrain();
        } catch (InterruptedException e) {
        }

        while (calculateDifferenceBetweenAngles(end, getDirection()) > 1 && central.opModeIsActive()){
            driveTrainMovement(0.2, (direction == turnside.cw) ? movements.ccw : movements.cw);
            central.telemetry.addLine("Correctional Try ");
            central.telemetry.addData("IMU Inital: ", start);
            central.telemetry.addData("IMU Final Projection: ", end);
            central.telemetry.addData("IMU Orient: ", getDirection());
            central.telemetry.addData("IMU Diffnce: ", end - getDirection());
            central.telemetry.update();
        }
        stopDrivetrain();
        central.telemetry.addLine("Completed");
        central.telemetry.addData("IMU Inital: ", start);
        central.telemetry.addData("IMU Final Projection: ", end);
        central.telemetry.addData("IMU Orient: ", getDirection());
        central.telemetry.addData("IMU Diffnce: ", end - getDirection());
        central.telemetry.update();
    }
    public void absturn(float target, turnside direction, double speed, axis rotation_Axis) throws InterruptedException{

        central.telemetry.addData("IMU State: ", imu.getSystemStatus());
        central.telemetry.update();

        double start = 0;

        double end = (start + ((direction == turnside.cw) ? target : -target) + 360) % 360;

        isnotstopped = true;
        try {
            switch (rotation_Axis) {
                case center:
                    driveTrainMovement(speed, (direction == turnside.cw) ? movements.cw : movements.ccw);
                    break;
                case back:
                    driveTrainMovement(speed, (direction == turnside.cw) ? movements.cwback : movements.ccwback);
                    break;
                case front:
                    driveTrainMovement(speed, (direction == turnside.cw) ? movements.cwfront : movements.ccwfront);
                    break;
            }
        } catch (InterruptedException e) {
            isnotstopped = false;
        }

        while (((calculateDifferenceBetweenAngles(getDirection(), end) > 1 && turnside.cw == direction) || (calculateDifferenceBetweenAngles(getDirection(), end) < -1 && turnside.ccw == direction)) && central.opModeIsActive() ) {
            central.telemetry.addLine("First Try ");
            central.telemetry.addData("IMU Inital: ", start);
            central.telemetry.addData("IMU Final Projection: ", end);
            central.telemetry.addData("IMU Orient: ", getDirection());
            central.telemetry.addData("IMU Difference: ", end - getDirection());
            central.telemetry.update();
        }
        try {
            stopDrivetrain();
        } catch (InterruptedException e) {
        }

        while (calculateDifferenceBetweenAngles(end, getDirection()) > 1 && central.opModeIsActive()){
            driveTrainMovement(0.2, (direction == turnside.cw) ? movements.ccw : movements.cw);
            central.telemetry.addLine("Correctional Try ");
            central.telemetry.addData("IMU Inital: ", start);
            central.telemetry.addData("IMU Final Projection: ", end);
            central.telemetry.addData("IMU Orient: ", getDirection());
            central.telemetry.addData("IMU Diffnce: ", end - getDirection());
            central.telemetry.update();
        }
        stopDrivetrain();
        central.telemetry.addLine("Completed");
        central.telemetry.addData("IMU Inital: ", start);
        central.telemetry.addData("IMU Final Projection: ", end);
        central.telemetry.addData("IMU Orient: ", getDirection());
        central.telemetry.addData("IMU Diffnce: ", end - getDirection());
        central.telemetry.update();
    }

    public double calculateDifferenceBetweenAngles(double firstAngle, double secondAngle)
    {
        double difference = secondAngle - firstAngle;
        while (difference < -180) difference += 360;
        while (difference > 180) difference -= 360;
        return difference;
    }

    public double getDirection(){
        return (this.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle-initorient+720)%360;
    }


    public enum EncoderMode{
        ON, OFF;
    }
    public enum setupType{
        autonomous, teleop, endgame, drive, camera, claw, bSystem, foundation, yellow, encoder, intake, ultrasoinc, imu;
    }

    //-------------------SET FUNCTIONS--------------------------------
    public void setCentral(Central central) {
        this.central = central;
    }
    public void setHardwareMap(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }
    public void setRuntime(ElapsedTime runtime) {
        this.runtime = runtime;
    }

    //-------------------CHOICE ENUMS-------------------------
    public enum movements {
        right(1, 1, -1, -1),
        left(-1, -1, 1, 1),
        backward(1, -1, 1, -1),
        forward(-1, 1, -1, 1),
        tr(0, -1, 1, 0),
        tl(1, 0, 0, -1),
        bl(0, 1, -1, 0),
        br(-1, 0, 0, 1),
        cw(1, 1, 1, 1),
        ccw(-1, -1, -1, -1),
        cwback(-1, -1, 0, 0),
        ccwback(1, 1, 0, 0),
        cwfront(0, 0, -1, -1),
        ccwfront(0, 0, 1, 1),
        linearUp(1),
        linearDown(-1),
        clawOut(1),
        clawIn(-1);



        private final double[] directions;

        movements(double... signs) {
            this.directions = signs;
        }

        public double[] getDirections() {
            return directions;
        }
    }


    public enum turnside {
        ccw, cw
    }

    public enum axis {
        front, center, back
    }


}
