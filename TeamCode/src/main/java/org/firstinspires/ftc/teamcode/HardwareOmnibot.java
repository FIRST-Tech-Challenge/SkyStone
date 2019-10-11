package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.ArrayList;
import java.util.List;
import java.util.Arrays;

import static java.lang.Math.*;

/**
 *Created by Ethan
 */
public class HardwareOmnibot extends HardwareOmnibotDrive
{
    public enum LiftPosition {
        STOWED(10),
        STONE1(149),
        ROTATE(300),
        STONE2(412),
        STONE3(660),
        STONE4(908),
        STONE5(1156),
        STONE6(1403),
        STONE7(1651),
        STONE8(1899),
        STONE9(2147),
        STONE10(2395),
        STONE11(2643),
        STONE12(2890);

        private final int encoderCount;
        LiftPosition(int encoderCount) { this.encoderCount = encoderCount; }
        public int getEncoderCount() { return encoderCount; }
    }

    /* Public OpMode members. */
    public static double RIGHT_FINGER_DOWN = 0.82;
    public static double LEFT_FINGER_DOWN = 0.82;
    public static double RIGHT_FINGER_UP = 0.25;
    public static double LEFT_FINGER_UP = 0.25;
    public static double CLAW_OPEN = 0.0;
    public static double CLAW_PINCHED = 1.0;
    public static double CLAWDRICOPTER_FRONT = 0.0;
    public static double CLAWDRICOPTER_BACK = 1.0;
    public static int MAX_STONE_HEIGHT = 13;
    public static int MIN_STONE_HEIGHT = 0;

    // Robot Controller Config Strings
    public final static String RIGHT_FINGER = "RightFinger";
    public final static String LEFT_FINGER = "LeftFinger";
    public final static String CLAW = "Claw";
    public final static String CLAWDRICTOPTER = "Clawdricopter";
    public final static String LEFT_INTAKE = "LeftIntake";
    public final static String RIGHT_INTAKE = "RightIntake";
    public final static String LIFTER = "Lifter";
    public final static String EXTENDER = "Extender";

    // Hardware objects
    protected Servo rightFinger = null;
    protected Servo leftFinger = null;
    protected Servo claw = null;
    protected Servo clawdricopter = null;
    protected DcMotor leftIntake = null;
    protected DcMotor rightIntake = null;
    protected DcMotor lifter = null;
    protected DcMotor extender = null;

    /* LEDs: Use this line if you drive the LEDs using an I2C/SPI bridge. */
    private DotStarBridgedLED leds;
    private IDotStarPattern robotDisplay;
    private IDotStarPattern ftcTimer;
    private IDotStarPattern halfAndHalf;
    private List<Integer> colors;

    // Tracking variables
    private boolean fingersUp = true;
    private boolean clawPinched = false;
    private boolean clawdricopterBack = false;
    // These are the heights of the stone levels to auto set the lift to
    protected LiftPosition lastLiftHeight = LiftPosition.STOWED;
    protected int liftZero = 0;

    /* Constructor */
    public HardwareOmnibot(){
        super();
    }

    public void initGroundEffects()
    {
        // Use ModernRoboticsDIM if using Modern Robotics hardware.
        leds.setController(DotStarBridgedLED.Controller.RevExpansionHub);
        leds.setLength(60);
        colors = new ArrayList<>();
        colors.add(0, 0x0);
        colors.add(1, 0x0);
        halfAndHalf = new DSPatternHalfAndHalf(leds);
        halfAndHalf.setPatternColors(colors);
        ftcTimer = new DSPatternFtcTimer(leds);
        robotDisplay = halfAndHalf;
    }

    public void updateTimerGroundEffects() {
        robotDisplay.update();
    }

    public void startTimerGroundEffects() {
        robotDisplay = ftcTimer;
        robotDisplay.update();
    }

    public void stopGroundEffects() {
    }

    public void toggleFingers() {
        if(fingersUp) {
            rightFinger.setPosition(RIGHT_FINGER_DOWN);
            leftFinger.setPosition(LEFT_FINGER_DOWN);
            fingersUp = false;
        } else {
            rightFinger.setPosition(RIGHT_FINGER_UP);
            leftFinger.setPosition(LEFT_FINGER_UP);
            fingersUp = true;
        }
    }

    public void toggleClaw() {
        if(clawPinched) {
            claw.setPosition(CLAW_OPEN);
            clawPinched = false;
        } else {
            claw.setPosition(CLAW_PINCHED);
            clawPinched = true;
        }
    }

    public void rotateClawdricopter() {
        if(clawdricopterBack) {
            clawdricopter.setPosition(CLAWDRICOPTER_FRONT);
            clawdricopterBack = false;
        } else {
            clawdricopter.setPosition(CLAWDRICOPTER_BACK);
            clawdricopterBack = true;
        }
    }

    public void setLiftZero(int value) {
        liftZero = value;
    }

    public void setLiftHeight(LiftPosition liftHeight) {
        if(liftHeight != lastLiftHeight) {
            lastLiftHeight = liftHeight;
            lifter.setTargetPosition(liftHeight.getEncoderCount() + liftZero);
            lifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lifter.setPower(1.0);
        }
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        super.init(ahwMap);

        rightFinger = hwMap.get(Servo.class, RIGHT_FINGER);
        leftFinger = hwMap.get(Servo.class, LEFT_FINGER);
        claw = hwMap.get(Servo.class, CLAW);
        clawdricopter = hwMap.get(Servo.class, CLAWDRICTOPTER);
        leftIntake = hwMap.get(DcMotor.class, LEFT_INTAKE);
        rightIntake = hwMap.get(DcMotor.class, RIGHT_INTAKE);
        lifter = hwMap.get(DcMotor.class, LIFTER);
        extender = hwMap.get(DcMotor.class, EXTENDER);

        // Set motor rotation
        // This makes lift go up with positive encoder values and power
        lifter.setDirection(DcMotor.Direction.REVERSE);
        // This makes extender go out with positive encoder values and power
        extender.setDirection(DcMotor.Direction.REVERSE);
        // This makes intake pull in with positive encoder values and power
        leftIntake.setDirection(DcMotor.Direction.REVERSE);
        rightIntake.setDirection(DcMotor.Direction.FORWARD);

        // Set motor encoder usage
        // Lifter encoder allows us to go to specific heights.
        lifter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // Extender encoder allows us to extend to specific positions.
        extender.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // The intake should just run as fast as it can.
        leftIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Reset the encoders that are used.
        int sleepTime = 0;
        int encoderCount = lifter.getCurrentPosition();

        lifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extender.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        while((encoderCount != 0) && (sleepTime < 1000)) {
            try {
                Thread.sleep(10);
            } catch (InterruptedException e) { break; }
            sleepTime += 10;
            encoderCount = lifter.getCurrentPosition();
        }

        lifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        extender.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Set the stop mode
        lifter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extender.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set up the LEDs. Change this to your configured name.
        leds = hwMap.get(DotStarBridgedLED.class, "leds");

        initGroundEffects();
    }

    /***
     *
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs  Length of wait cycle in mSec.
     * @throws InterruptedException
     */
    /*
    public void waitForTick(long periodMs) throws InterruptedException {

        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0)
            Thread.sleep(remaining);

        // Reset the cycle clock for the next pass.
        period.reset();
    }
    */
}

