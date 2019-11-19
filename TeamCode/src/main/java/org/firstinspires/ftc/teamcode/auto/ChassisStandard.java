package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;


public abstract class ChassisStandard extends OpMode {

    //copied from tensorflow example
    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";
    private static final String VUFORIA_KEY =
            "AfgOBrf/////AAABmRjMx12ilksPnWUyiHDtfRE42LuceBSFlCTIKmmNqCn2EOk3I4NtDCSr0wCLFxWPoLR2qHKraX49ofQ2JknI76SJS5Hy8cLbIN+1GlFDqC8ilhuf/Y1yDzKN6a4n0fYWcEPlzHRc8C1V+D8vZ9QjoF3r//FDDtm+M3qlmwA7J/jNy4nMSXWHPCn2IUASoNqybTi/CEpVQ+jEBOBjtqxNgb1CEdkFJrYGowUZRP0z90+Sew2cp1DJePT4YrAnhhMBOSCURgcyW3q6Pl10XTjwB4/VTjF7TOwboQ5VbUq0wO3teE2TXQAI53dF3ZUle2STjRH0Rk8H94VtHm9u4uitopFR7zmxVl3kQB565EUHwfvG";

    //vision detection variables/state
    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    protected VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    protected TFObjectDetector tfod;

    //is sound playing?
    boolean soundPlaying = false;

    int bruhSoundID = -1;

    protected ChassisConfig config;
    protected boolean madeTheRun = false;


    // Elapsed time since the opmode started.
    protected ElapsedTime runtime = new ElapsedTime();
    protected ElapsedTime dropTime = new ElapsedTime();

    // Motors connected to the hub.
    private DcMotor motorBackLeft;
    private DcMotor motorBackRight;
    private DcMotor motorFrontLeft;
    private DcMotor motorFrontRight;
    private DcMotor extender;
    private DcMotor shoulder;
    private DcMotor elevator;
    private DcMotor crane;

    //Crab
    protected Servo crab;

    //fingers
    protected Servo fingerFront;
    protected Servo fingerBack;

    // Team Marker Servo
    private Servo flagHolder;
    private Servo bull;
    private Servo dozer;
    private double angleHand;
    private double angleAnkle;
    private double ffAngleHand;
    private double bfAngleHand;

    // Walle state management
    int wasteAllocationLoadLifterEarthBegin;
    private DcMotor wasteAllocationLoadLifterEarth;

    //Screw
    int extraterrestrialVegetationEvaluatorBegin;
    private DcMotor extraterrestrialVegetationEvaluator;

    //gyroscope built into hub
    private BNO055IMU bosch;

    // Hack stuff.
    protected boolean useGyroScope = true;
    protected boolean useMotors = true;
    protected boolean hackTimeouts = true;
    protected boolean useArm = false;
    protected boolean useEve = false;
    protected boolean useCrab = true;
    protected boolean useElevator = true;
    protected boolean useFingers = true;
    protected boolean useVuforia = false;


    protected ChassisStandard() {
        this(ChassisConfig.forDefaultConfig());
    }

    protected ChassisStandard(ChassisConfig config) {
        this.config = config;
    }

     /*
        Robot Controller Callbacks
     */

    @Override
    public void start () {
        // Reset the game timer.
        runtime.reset();

    }

    @Override
    public void stop (){
    }

    @Override
    public void init() {
        initMotors();
        initTimeouts();
        initGyroscope();
        initCrab();
        initFingers();
        initVuforia();
    }

    /**
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop () {
        printStatus();
    }


    /**
     *
     */
    protected void printStatus() {
        if(useGyroScope) {
            telemetry.addData("Gyro", "angle: " + this.getGyroscopeAngle());
        }
        if(useCrab) {
            telemetry.addData("Crab", "Angle =%f", crab.getPosition());
        }
        telemetry.addData("Status", "time: " + runtime.toString());
        telemetry.addData("Run", "madeTheRun=%b", madeTheRun);
    }

    /*
        MOTOR SUBSYTEM
     */

    protected void initMotors() {

        // Initialize the motors.
        if (useMotors) {
            try  {
                motorBackLeft = hardwareMap.get(DcMotor.class, "motor0");
                motorBackRight = hardwareMap.get(DcMotor.class, "motor1");

                // Most robots need the motor on one side to be reversed to drive forward
                // Reverse the motor that runs backwards when connected directly to the battery
                motorBackLeft.setDirection(config.isLeftMotorReversed() ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);
                motorBackRight.setDirection(config.isRightMotorReversed() ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);

                // initilize the encoder
                motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                if (config.getUseFourWheelDrive()) {
                    motorFrontLeft = hardwareMap.get(DcMotor.class, "motor2");
                    motorFrontRight = hardwareMap.get(DcMotor.class, "motor3");

                    motorFrontLeft.setDirection(config.isLeftMotorReversed() ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);
                    motorFrontRight.setDirection(config.isRightMotorReversed() ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);

                    motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                }


                // init the lifter arm,
                if (config.getHasWalle()) {
                    if (useEve == true) {
                        extraterrestrialVegetationEvaluator = hardwareMap.get(DcMotor.class, "motor6");
                        extraterrestrialVegetationEvaluator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        extraterrestrialVegetationEvaluator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        extraterrestrialVegetationEvaluatorBegin = extraterrestrialVegetationEvaluator.getCurrentPosition();
                    } else {
                        wasteAllocationLoadLifterEarth = hardwareMap.get(DcMotor.class, "motor6");
                        wasteAllocationLoadLifterEarth.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        wasteAllocationLoadLifterEarth.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        wasteAllocationLoadLifterEarthBegin = wasteAllocationLoadLifterEarth.getCurrentPosition();
                    }
                }
            } catch (Exception e) {
                telemetry.addData("motors", "exception on init: " + e.toString());
                useMotors = false;
            }
        }
    }

    protected void initCrab(){
        if(useCrab){
            try {
                crab = hardwareMap.get(Servo.class, "servoCrab");
            } catch (Exception e) {
                telemetry.addData("crab", "exception on init: " + e.toString());
                useCrab = false;
            }
        }
    }

    protected void initElevator() {
        if (useElevator) {
            try {
                elevator = hardwareMap.get(DcMotor.class, "elevator");
            } catch (Exception e) {
                telemetry.addData("elevator", "exception on init: " + e.toString());
                useElevator = false;
            }
        }
    }

    protected void initFingers(){
        if(useFingers){
            try {
                fingerFront = hardwareMap.get(Servo.class, "servoFrontFinger");

                fingerBack = hardwareMap.get(Servo.class, "servoBackFinger");

            } catch (Exception e) {
                telemetry.addData("finger", "exception on init: " + e.toString());
                useFingers = false;
            }
        }
    }


    protected void initTimeouts() {
        // This code prevents the OpMode from freaking out if you go to sleep for more than a second.
        if (hackTimeouts) {
            this.msStuckDetectInit = 30000;
            this.msStuckDetectInitLoop = 30000;
            this.msStuckDetectStart = 30000;
            this.msStuckDetectLoop = 30000;
            this.msStuckDetectStop = 30000;
        }
    }

   /* protected void initArm() {
        if (useArm) {
            //shoulder = hardwareMap.get(DcMotor.class, "motor4");
            extender = hardwareMap.get(DcMotor.class, "motorExtender");
            crane = hardwareMap.get(DcMotor.class, "motorCrane");
        }
    }

    */

    protected boolean initGyroscope() {
        if (useGyroScope) {
            bosch = hardwareMap.get(BNO055IMU.class, "imu0");
            telemetry.addData("Gyro", "class:" + bosch.getClass().getName());

            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.mode = BNO055IMU.SensorMode.IMU;
            parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.loggingEnabled = false;
            parameters.loggingTag = "bosch";
            //parameters.calibrationDataFile = "MonsieurMallahCalibration.json"; // see the calibration sample opmode
            boolean boschInit = bosch.initialize(parameters);
            return boschInit;
        } else {
            return true;
        }
    }


    public void dropFrontFinger() {
        if (useFingers) {
            angleHand = 0.0;
            fingerFront.setPosition(ffAngleHand);

        }
    }

    public void raiseFrontFinger() {
        if (useFingers) {
            angleHand = 1.0;
            fingerFront.setPosition(ffAngleHand);
        }
    }
    
    public void dropBackFinger() {
        if (useFingers) {
            angleHand = 0.0;
            fingerBack.setPosition(bfAngleHand);

        }
    }

    public void raiseBackFinger() {
        if (useFingers) {
            angleHand = 1.0;
            fingerBack.setPosition(bfAngleHand);
        }
    }


    public void dropCrab() {
        if (useCrab) {
            angleHand = 0.0;
            crab.setPosition(angleHand);
        }
    }

    public void raiseCrab() {
        if (useCrab) {
            angleHand = 1.0;
            crab.setPosition(angleHand);
        }
    }

    public void raiseElevator() {
        if(useElevator) {
            angleAnkle = 1.0;
            elevator.setPower(angleAnkle);
        }
    }

    public void dropElevator() {
        if(useElevator) {
            angleAnkle = 0.0;
            elevator.setPower(angleAnkle);
        }
    }

    protected void encoderDrive(double inches) {
        encoderDrive(inches, inches);
    }

    protected void encoderDrive(double leftInches, double rightInches) {
        double speed = config.getMoveSpeed();
        encoderDrive(leftInches, rightInches, speed);
    }

    protected void encoderDrive(double leftInches, double rightInches, double speed) {

        double countsPerInch = config.getRearWheelSpeed() / (config.getRearWheelDiameter() * Math.PI);

        // Get the current position.
        int leftBackStart = motorBackLeft.getCurrentPosition();
        int rightBackStart = motorBackRight.getCurrentPosition();
        int leftFrontStart = 0;
        int rightFrontStart = 0;
        if (config.getUseFourWheelDrive()) {
            leftFrontStart = motorFrontLeft.getCurrentPosition();
            rightFrontStart = motorFrontRight.getCurrentPosition();
        }
        telemetry.addData("encoderDrive", "Starting %7d, %7d, %7d, %7d",
                leftBackStart, rightBackStart, leftFrontStart, rightFrontStart);

        // Determine new target position, and pass to motor controller
        int leftBackTarget = leftBackStart + (int) (leftInches * countsPerInch);
        int rightBackTarget = rightBackStart + (int) (rightInches * countsPerInch);
        int leftFrontTarget = 0;
        int rightFrontTarget = 0;
        if (config.getUseFourWheelDrive()) {
            leftFrontTarget = leftFrontStart + (int) (leftInches * countsPerInch);
            rightFrontTarget = rightFrontStart + (int) (rightInches * countsPerInch);
        }

        motorBackLeft.setTargetPosition(leftBackTarget);
        motorBackRight.setTargetPosition(rightBackTarget);
        if (config.getUseFourWheelDrive()) {
            motorFrontLeft.setTargetPosition(leftFrontTarget);
            motorFrontRight.setTargetPosition(rightFrontTarget);
        }
        telemetry.addData("encoderDrive", "Target %7d, %7d, %7d, %7d",
                leftBackTarget, rightBackTarget, leftFrontTarget, rightFrontTarget);

        // Turn On RUN_TO_POSITION
        motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackLeft.setPower(Math.abs(speed));
        motorBackRight.setPower(Math.abs(speed));
        if (config.getUseFourWheelDrive()) {
            motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorFrontLeft.setPower(Math.abs(speed));
            motorFrontRight.setPower(Math.abs(speed));
        }

        // keep looping while we are still active, and there is time left, and both motors are running.
        // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
        // its target position, the motion will stop.  This is "safer" in the event that the robot will
        // always end the motion as soon as possible.
        // However, if you require that BOTH motors have finished their moves before the robot continues
        // onto the next step, use (isBusy() || isBusy()) in the loop test.
        ElapsedTime motorOnTime = new ElapsedTime();
        boolean keepGoing = true;
        while ((motorOnTime.seconds() < 30) && keepGoing) {

            if (config.getUseFourWheelDrive()) {
                telemetry.addData("encoderDrive1", "Running at %7d, %7d, %7d, %7d",
                        motorBackLeft.getCurrentPosition(),
                        motorBackRight.getCurrentPosition(),
                        motorFrontLeft.getCurrentPosition(),
                        motorFrontRight.getCurrentPosition());
                telemetry.addData("encoderDrive2", "Running to %7d, %7d, %7d, %7d",
                        leftBackTarget,
                        rightBackTarget,
                        leftFrontTarget,
                        rightFrontTarget);
                keepGoing = motorBackRight.isBusy() && motorBackLeft.isBusy() && motorFrontLeft.isBusy() && motorFrontRight.isBusy();
            } else {
                telemetry.addData("encoderDrive1", "Running at %7d, %7d",
                        motorBackLeft.getCurrentPosition(),
                        motorBackRight.getCurrentPosition());
                telemetry.addData("encoderDrive2", "Running to %7d, %7d",
                        leftBackTarget,
                        rightBackTarget);
                keepGoing = motorBackRight.isBusy() && motorBackLeft.isBusy();
            }

            telemetry.update();
            sleep(100);
        }

        // Turn off RUN_TO_POSITION
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if (config.getUseFourWheelDrive()) {
            motorFrontLeft.setPower(0);
            motorFrontRight.setPower(0);
            motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        /*telemetry.addData("encoderDrive", "Finished (%s) at %7d,%7d,%7d,%7d to [%7d,%7d,%7d,%7d] (%7d,%7d,%7d,%7d)",
                motorOnTime.toString(),
                leftBackStart,
                rightBackStart,
                leftFrontStart,
                rightFrontStart,
                motorBackLeft.getCurrentPosition(),
                motorBackRight.getCurrentPosition(),
                motorFrontLeft.getCurrentPosition(),
                motorFrontRight.getCurrentPosition(),
                leftBackTarget,
                rightBackTarget,
                leftFrontTarget,
                rightFrontTarget); */
        sleep(1000);
    }


    protected void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();

        }
    }

    // Always returns a number from 0-359.9999
    protected float getGyroscopeAngle() {
        if (useGyroScope && bosch != null) {
            Orientation exangles = bosch.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
            float gyroAngle = exangles.thirdAngle;
            //exangles.
            telemetry.addData("angle", "angle: " + exangles.thirdAngle);
            float calculated = CrazyAngle.normalizeAngle(CrazyAngle.reverseAngle(gyroAngle));
            telemetry.addData("angle2","calculated:" + calculated);
            return calculated;
        } else {
            return 0.0f;
        }
    }


    /**
     * @param deltaAngle
     */
    protected void turnLeft(float deltaAngle) {
        assert (deltaAngle > 0.0);
        assert (deltaAngle <= 360.0);

        // does it wrap at all? (go around the zero mark?)
        float currentAngle = getGyroscopeAngle();
        float destinationAngle = currentAngle - deltaAngle;
        boolean doesItWrapAtAll = (destinationAngle < 0.0);
        destinationAngle = CrazyAngle.normalizeAngle(destinationAngle);

        // Get it past the zero mark.
        if (doesItWrapAtAll) {
            boolean keepGoing = true;
            while (keepGoing) {
                float oldAngle = currentAngle;
                nudgeLeft();
                currentAngle = getGyroscopeAngle();

                float justMoved = oldAngle - currentAngle;
                float stillNeed = currentAngle;
                telemetry.addData("turnLeft1", "current=%.0f, old=%.0f, dst=%.0f, moved=%.0f, need=%.0f", currentAngle, oldAngle, destinationAngle, justMoved, stillNeed);
                telemetry.update();

                keepGoing = (justMoved > -50.0);
            }
        }

        // turn the last part
        while ((currentAngle - destinationAngle) > 5.0) {

            float oldAngle = currentAngle;
            nudgeLeft();
            currentAngle = getGyroscopeAngle();

            float justMoved = oldAngle - currentAngle;
            float stillNeed = currentAngle - destinationAngle;
            telemetry.addData("turnLeft2", "current = %.0f, destination = %.0f, moved=%.0f, need=%.0f", currentAngle, destinationAngle, justMoved, stillNeed);
            telemetry.update();
        }

        // turn off motor.
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);
        if (config.getUseFourWheelDrive()) {
            motorFrontLeft.setPower(0);
            motorFrontRight.setPower(0);
        }
    }

    /**
     * @param deltaAngle
     */
    protected void turnRight(float deltaAngle) {
        assert (deltaAngle > 0.0);
        assert (deltaAngle <= 360.0);

        // does it wrap at all?
        float currentAngle = getGyroscopeAngle();
        float destinationAngle = currentAngle + deltaAngle;
        boolean doesItWrapAtAll = (destinationAngle > 360.0);
        destinationAngle = CrazyAngle.normalizeAngle(destinationAngle);

        // Get it past the zero mark.
        if (doesItWrapAtAll) {
            boolean keepGoing = true;
            while (keepGoing) {
                float oldAngle = currentAngle;
                nudgeRight();
                currentAngle = getGyroscopeAngle();

                float justMoved = currentAngle - oldAngle;
                float stillNeed = 360.0f - currentAngle;
                telemetry.addData("turRight1", "current=%.0f, old=%.0f, dst=%.0f, moved=%.0f, need=%.0f", currentAngle, oldAngle, destinationAngle, justMoved, stillNeed);
                telemetry.update();

                keepGoing = (justMoved > -50.0);
            }
        }

        // turn the last part
        while ((destinationAngle - currentAngle) > 5.0) {

            float oldAngle = currentAngle;
            nudgeRight();

            currentAngle = getGyroscopeAngle();

            float justMoved = currentAngle - oldAngle;
            float stillNeed = destinationAngle - currentAngle;
            telemetry.addData("turnRight2", "current = %.0f, destination = %.0f, moved=%.0f, need=%.0f", currentAngle, destinationAngle, justMoved, stillNeed);
            telemetry.update();
        }

        //turn off the motor
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);
        if (config.getUseFourWheelDrive()) {
            motorFrontLeft.setPower(0);
            motorFrontRight.setPower(0);

        }

    }

    // This nudges over about 2 degrees.
    protected void nudgeRight() {
        float power = config.getTurnSpeed();

        motorBackLeft.setPower(power);
        motorBackRight.setPower(-power);
        if (config.getUseFourWheelDrive()) {
            motorFrontLeft.setPower(power);
            motorFrontRight.setPower(-power);
        }
        sleep(5);
    }

    // This nudges over about 2 degrees.
    protected void nudgeLeft() {
        float power = config.getTurnSpeed();

        motorBackLeft.setPower(-power);
        motorBackRight.setPower(power);
        if (config.getUseFourWheelDrive()) {
            motorFrontLeft.setPower(-power);
            motorFrontRight.setPower(power);
        }
        sleep(5);
    }


    protected void nudgeBack() {
        float power = config.getTurnSpeed();

        motorBackLeft.setPower(-power);
        motorBackRight.setPower(-power);
        if (config.getUseFourWheelDrive()) {
            motorFrontLeft.setPower(-power);
            motorFrontRight.setPower(-power);
        }
        sleep(500);

        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);
        if (config.getUseFourWheelDrive()) {
            motorFrontLeft.setPower(0);
            motorFrontRight.setPower(0);
        }
    }


    protected void pointToZero() {

        float currentAngle = getGyroscopeAngle();
        float destinationAngle = 0;
        boolean keepGoing = true;
        while (keepGoing) {
            float oldAngle = currentAngle;
            nudgeRight();
            currentAngle = getGyroscopeAngle();

            float justMoved = currentAngle - oldAngle;
            float stillNeed = 360.0f - currentAngle;
            telemetry.addData("turRight1", "current=%.0f, old=%.0f, dst=%.0f, moved=%.0f, need=%.0f", currentAngle, oldAngle, destinationAngle, justMoved, stillNeed);
            telemetry.update();

            keepGoing = (justMoved > -50.0);
        }
    }

     protected void slideUpExtender(int extenderCounts) {
        double speed = 0.25;

        // Get the current position.
        int extenderStart = extender.getCurrentPosition();
        telemetry.addData("encoderDrive", "Starting %7d", extenderStart);

        // Determine new target position, and pass to motor controller
        int extenderTarget = extenderStart + extenderCounts;
        extender.setTargetPosition(extenderTarget);
        telemetry.addData("encoderDrive", "Target %7d", extenderTarget);

        // Turn On RUN_TO_POSITION
        extender.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        extender.setPower(speed);

        ElapsedTime motorOnTime = new ElapsedTime();
        while ((motorOnTime.seconds() < 30) && extender.isBusy()) {
            telemetry.addData("slideUpExtender", "Running at %7d to %7d", extender.getCurrentPosition(), extenderTarget);
            telemetry.update();
            sleep(10);
        }

        // Turn off RUN_TO_POSITION
        extender.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        extender.setPower(0);
    }

    protected void shiftShoulderDown(int shoulderCounts) {
        double speed = 0.5;

        // Get the current position.
        int shoulderStart = shoulder.getCurrentPosition();
        telemetry.addData("shoulderShifting", "Starting %7d", shoulderStart);

        // Determine new target position, and pass to motor controller
        int shoulderTarget = shoulderStart + shoulderCounts;
        shoulder.setTargetPosition(shoulderTarget);
        telemetry.addData("shoulderShifting", "Target %7d", shoulderTarget);

        // Turn On RUN_TO_POSITION
        shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        shoulder.setPower(speed);

        ElapsedTime motorOnTime = new ElapsedTime();
        while ((motorOnTime.seconds() < 30) && shoulder.isBusy()) {
            telemetry.addData("shiftShoulderDown", "Running at %7d to %7d", shoulder.getCurrentPosition(), shoulderTarget);
            telemetry.update();
            telemetry.update();
            sleep(10);
        }

        // Turn off RUN_TO_POSITION
        shoulder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shoulder.setPower(0);
    }


    protected void strafeLeft(int numberOfMillis) {
        float power = config.getTurnSpeed();

        motorBackLeft.setPower(power);
        motorBackRight.setPower(-power);
        if (config.getUseFourWheelDrive()) {
            motorFrontLeft.setPower(-power);
            motorFrontRight.setPower(power);
        }
        sleep(numberOfMillis);

        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);
        if (config.getUseFourWheelDrive()) {
            motorFrontLeft.setPower(0);
            motorFrontRight.setPower(0);
        }
    }


    protected void strafeRight(int numberOfMillis) {
        float power = config.getTurnSpeed();

        motorBackLeft.setPower(-power);
        motorBackRight.setPower(power);
        if (config.getUseFourWheelDrive()) {
            motorFrontLeft.setPower(power);
            motorFrontRight.setPower(-power);
        }
        sleep(numberOfMillis);

        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);
        if (config.getUseFourWheelDrive()) {
            motorFrontLeft.setPower(0);
            motorFrontRight.setPower(0);
        }
    }

    protected void descendFromLander() {
        if (!config.getlyftStrategy()) {
            // go down.
            lyftDownEve(13930);
            encoderDrive(10);
            lyftDownEve(-13930);
        } else {
            lyftDownEve(-1449);
            //write new phatswipe descend strategy
            turnLeft(25);
            encoderDrive(2, 2);
            turnRight(25);
        }
    }



    protected void lyftDownWalle(int howManySpins) {
        double speed = 0.5f;

        // Get the current position.
        int lyftBegin = wasteAllocationLoadLifterEarth.getCurrentPosition();
        telemetry.addData("lyftDownWalle", "Starting %7d", lyftBegin);

        // Determine new target position, and pass to motor controller
        int lyftTarget = lyftBegin + howManySpins;
        wasteAllocationLoadLifterEarth.setTargetPosition(lyftTarget);
        telemetry.addData("lyftDownWalle", "Target %7d", lyftTarget);

        // Turn On RUN_TO_POSITION
        wasteAllocationLoadLifterEarth.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wasteAllocationLoadLifterEarth.setPower(speed);

        ElapsedTime motorOnTime = new ElapsedTime();
        while ((motorOnTime.seconds() < 30) && wasteAllocationLoadLifterEarth.isBusy()) {
            telemetry.addData("lyftDownWalle", "Running at %7d to %7d", wasteAllocationLoadLifterEarth.getCurrentPosition(), lyftTarget);
            telemetry.update();
            sleep(10);
        }

        // Turn off RUN_TO_POSITION
        wasteAllocationLoadLifterEarth.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wasteAllocationLoadLifterEarth.setPower(0);

        //sleep(5000);
    }


    protected void lyftDownEve (int howManySpins) {
        double speed = 0.5f;

        // Get the current position.
        int lyftBegin = extraterrestrialVegetationEvaluator.getCurrentPosition();
        telemetry.addData("lyftDownWalle", "Starting %7d", lyftBegin);

        // Determine new target position, and pass to motor controller
        int lyftTarget = lyftBegin + howManySpins;
        extraterrestrialVegetationEvaluator.setTargetPosition(lyftTarget);
        telemetry.addData("lyftDownWalle", "Target %7d", lyftTarget);

        // Turn On RUN_TO_POSITION
        extraterrestrialVegetationEvaluator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        extraterrestrialVegetationEvaluator.setPower(speed);

        ElapsedTime motorOnTime = new ElapsedTime();
        while ((motorOnTime.seconds() < 30) && extraterrestrialVegetationEvaluator.isBusy()) {
            telemetry.addData("lyftDownWalle", "Running at %7d to %7d", extraterrestrialVegetationEvaluator.getCurrentPosition(), lyftTarget);
            telemetry.update();
            sleep(10);
        }

        // Turn off RUN_TO_POSITION
        extraterrestrialVegetationEvaluator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        extraterrestrialVegetationEvaluator.setPower(0);

        //sleep(5000);
    }


    /* VUFORIA */

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {

        if (useVuforia) {
            try {
                /*
                 * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
                 */
                VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
                parameters.vuforiaLicenseKey = VUFORIA_KEY;
                parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

                //  Instantiate the Vuforia engine
                vuforia = ClassFactory.getInstance().createVuforia(parameters);

                // Loading trackables is not necessary for the TensorFlow Object Detection engine.
                if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
                    initTfod();
                } else {
                    telemetry.addData("Sorry!", "This device is not compatible with TFOD");
                }

                /**
                 * Activate TensorFlow Object Detection before we wait for the start command.
                 * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
                 **/
                if (tfod != null) {
                    tfod.activate();
                }
            } catch (Exception e) {
                telemetry.addData("vuforia", "exception on init: " + e.toString());
                useVuforia = false;
            }
        }
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.8;

        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }
}
