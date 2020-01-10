package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a K9 robot.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Servo channel:  Servo to raise/lower arm: "arm"
 * Servo channel:  Servo to open/close claw: "claw"
 *
 * Note: the configuration of the servos is such that:
 *   As the arm servo approaches 0, the arm position moves up (away from the floor).
 *   As the claw servo approaches 0, the claw opens up (drops the game element).
 */
public class HardwareK9bot
{
    /* Public OpMode members. */

    public DcMotor frontLeftMotor   = null;
    public DcMotor frontRightMotor  = null;
    public DcMotor backLeftMotor   = null;
    public DcMotor backRightMotor  = null;
    public Servo hookOne = null;
    public Servo hookTwo = null;

    public DcMotor slide = null;
    public Servo grab = null;

    //public DcMotor leftMotor   = null;
    //public DcMotor rightMotor  = null;

    //public DcMotor fingerMotor = null;
    //public DcMotor armMotor    = null;
    //public DcMotor armSlide    = null;
    //public DcMotor slideMotor = null;

   // public DcMotor linSlideLeft = null;
   // public DcMotor linSlideRight = null;

    //public Servo claw        = null;
    //public Servo lobster     = null;
    //public Servo clawLeft = null;
    //public Servo clawRight = null;
    //public Servo colourStick = null;

    public Servo hookLeft = null;
    public Servo hookRight = null;
    public Servo markerStick = null;

    //public final static double ARM_HOME = 0.2;

    public final static double LEFT_CLAW_HOME = 0;
    public final static double RIGHT_CLAW_HOME = 1;
    public final static double COLOURSTICK_HOME = 0.9;

    //public final static double ARM_MIN_RANGE  = 0.20;
    //public final static double ARM_MAX_RANGE  = 0.90;
    public final static double CLAW_MIN_RANGE  = 0.20;
    public final static double CLAW_MAX_RANGE  = 0.7;

    /* Local OpMode members. */
    HardwareMap hwMap  = null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwareK9bot() {
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // save reference to HW Map
        hwMap = ahwMap;

        // Define and Initialize Motors
        frontLeftMotor   = hwMap.dcMotor.get("front_left_drive");
        frontRightMotor  = hwMap.dcMotor.get("front_right_drive");
        backLeftMotor   = hwMap.dcMotor.get("back_left_drive");
        backRightMotor  = hwMap.dcMotor.get("back_right_drive");

        slide = hwMap.dcMotor.get("slide");





//        leftMotor   = hwMap.dcMotor.get("left_drive");
//        rightMotor  = hwMap.dcMotor.get("right_drive");
        //fingerMotor = hwMap.dcMotor.get("chicken_fingers");
        //armSlide = hwMap.dcMotor.get("armSlide");
        //slideMotor = hwMap.dcMotor.get("slide");
//        linSlideLeftF = hwMap.dcMotor.get("linSlideLeft");
//        linSlideRight = hwMap.dcMotor.get("linSlideRight");

//        leftMotor.setDirection(DcMotor.Direction.REVERSE);

        // Set all motors to zero power
//        leftMotor.setPower(0);
//        rightMotor.setPower(0);

        //fingerMotor.setPower(0);
        //armMotor.setPower(0);
        //slideMotor.setPower(0);

//      linSlideLeft.setPower(0);
//      linSlideRight.setPower(0);

        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



//        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Define and initialize ALL installed servos. //
        hookOne = hwMap.servo.get("hook_one");
        hookTwo = hwMap.servo.get("hook_two");
        grab = hwMap.servo.get("grab");
        //claw = hwMap.servo.get("claw");
        //lobster = hwMap.servo.get("lobster");
        //colourStick = hwMap.servo.get("colourStick");
        //clawLeft = hwMap.servo.get("clawLeft");
        //clawRight = hwMap.servo.get("clawRight");

//        hookLeft = hwMap.servo.get("hookLeft");
//        hookRight = hwMap.servo.get("hookRight");
//        markerStick = hwMap.servo.get("marker_stick");


        //markerStick.setDirection(Servo.Direction.REVERSE);

        //clawLeft.setPosition(LEFT_CLAW_HOME);
        //clawRight.setPosition(RIGHT_CLAW_HOME);
        //colourStick.setPosition(COLOURSTICK_HOME);
    }

    /***
     *
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs  Length of wait cycle in mSec.
     */
    public void waitForTick(long periodMs) {

        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0) {
            try {
                Thread.sleep(remaining);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }

        // Reset the cycle clock for the next pass.
        period.reset();
    }
}
