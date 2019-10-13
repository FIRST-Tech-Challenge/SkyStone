package org.firstinspires.ftc.teamcode;

import android.content.Context;

import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


/**
 *
 */
@TeleOp(name="Tyler TeleOp", group="AAA")
public class TylerController extends OpMode {



    //is sound playing?
    boolean soundPlaying = false;

    int bruhSoundID = -1;
    int oofSoundID = -1;

    // Motors connected to the hub.
    private DcMotor motorBackLeft;
    private DcMotor motorBackRight;
    private DcMotor motorFrontLeft;
    private DcMotor motorFrontRight;

    private Servo crab;

    private DigitalChannel digitalTouch;  // Hardware Device Object


    // Hack stuff.
    private boolean useMotors = true;
    private boolean useEncoders = true;
    private boolean useArm = true; // HACK
    private boolean useCrab = true;
    private boolean useLifter = true; // HACL
    private boolean useDropper = true;
    private boolean useTouch = true;
    private boolean useRange = true;

    //Movement State
    private int armState;
    private int extenderTarget;
    private int lifterState;
    private int lifterExtenderTarget;
    private int extenderStartPostion = 0;
    private int lifterStartPosition = 0;
    private int shoulderTarget;
    private int shoulderStartPosition = 0;

   // Claw state
    private double angleHand;

    //Drive State
    private boolean switchFront = false;

    //distance sensors
    private DistanceSensor rangeFront;
    private DistanceSensor rangeBack;
    private DistanceSensor rangeLeft;
    private DistanceSensor rangeRight;

    /**
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {



        // Initialize the motors.
        if (useMotors) {
            motorBackLeft = hardwareMap.get(DcMotor.class, "motor0");
            motorBackRight = hardwareMap.get(DcMotor.class, "motor1");
            motorFrontLeft = hardwareMap.get(DcMotor.class, "motor2");
            motorFrontRight = hardwareMap.get(DcMotor.class, "motor3");

            // Most robots need the motor on one side to be reversed to drive forward
            // Reverse the motor that runs backwards when connected directly to the battery
            motorBackLeft.setDirection(DcMotor.Direction.FORWARD);
            motorBackRight.setDirection(DcMotor.Direction.REVERSE);
            motorFrontLeft.setDirection(DcMotor.Direction.FORWARD);
            motorFrontRight.setDirection(DcMotor.Direction.REVERSE);

            if (useEncoders) {
                motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
        }

        if (useCrab) {
            crab = hardwareMap.get(Servo.class, "servoCrab");

            if (crab == null){
                telemetry.addData("Crab", "You forgot to set up crab, set up servoCrab");
                useCrab = false;
            }
        }

        if (useTouch) {
            try {
                digitalTouch = hardwareMap.get(DigitalChannel.class, "sensorTouch");
            } catch (Exception e) {
                telemetry.addData("Touch", "exception on init: " + e.toString());
                digitalTouch = null;
            }

            if (digitalTouch == null) {
                telemetry.addData("Touch", "You forgot to set up sensorTouch, set up ");
                useTouch = false;
            }else {
                digitalTouch.setMode(DigitalChannel.Mode.INPUT);
            }
          
        if (useRange) {
            //initialize the four lidar sensors
            rangeFront = hardwareMap.get(DistanceSensor.class, "range_front");
            rangeBack = hardwareMap.get(DistanceSensor.class, "range_back");
            rangeLeft = hardwareMap.get(DistanceSensor.class, "range_left");
            rangeRight = hardwareMap.get(DistanceSensor.class, "range_right");

            Context myApp = hardwareMap.appContext;

            //load sound file
            bruhSoundID = myApp.getResources().getIdentifier("bruh", "raw", myApp.getPackageName());
            oofSoundID = myApp.getResources().getIdentifier("oof", "raw", myApp.getPackageName());
        }
    }



    /**
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /**
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {

    }

    /**
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

    /**
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        if (useRange) {

            Context myApp = hardwareMap.appContext;
            //getting the ranges from the lidar sensors on to the phones
            telemetry.addData("range: f,b", String.format("%.01f, %.01f ", rangeFront.getDistance(DistanceUnit.CM), rangeBack.getDistance(DistanceUnit.CM)));
            telemetry.addData("range: r,l", String.format("%.01f, %.01f", rangeLeft.getDistance(DistanceUnit.CM), rangeRight.getDistance(DistanceUnit.CM)));

            if (rangeFront.getDistance(DistanceUnit.CM) < 5) {

                if (bruhSoundID != 0) {

                    SoundPlayer.PlaySoundParams params = new SoundPlayer.PlaySoundParams();

                    // Signal that the sound is now playing.
                    soundPlaying = true;

                    // Start playing, and also Create a callback that will clear the playing flag when the sound is complete.

                    SoundPlayer.getInstance().startPlaying(myApp, bruhSoundID, params, null,
                            new Runnable() {
                                public void run() {
                                    soundPlaying = false;
                                }
                            });
                }
            } else if (rangeBack.getDistance(DistanceUnit.CM) < 5) {

                if (oofSoundID != 0) {

                    SoundPlayer.PlaySoundParams params = new SoundPlayer.PlaySoundParams();

                    // Signal that the sound is now playing.
                    soundPlaying = true;

                    // Start playing, and also Create a callback that will clear the playing flag when the sound is complete.

                    SoundPlayer.getInstance().startPlaying(myApp, oofSoundID, params, null,
                            new Runnable() {
                                public void run() {
                                    soundPlaying = false;
                                }
                            });
                }
            }
        }

        if (useMotors) {
            // Switch the directions for driving!
            if (gamepad1.b) {
                switchFront = !switchFront;
                sleep(500);
            }

            // Control the wheel motors.
            // POV Mode uses left stick to go forw ard, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            double driveNormal = -gamepad1.left_stick_y;
            if (Math.abs(driveNormal) < 0.1)
                driveNormal = 0.0; // Prevent the output from saying "-0. 0".

            double driveStrafe = -gamepad1.left_stick_x;
            if (Math.abs(driveStrafe) < 0.1)
                driveStrafe = 0.0; // Prevent the output from saying "-0.0".

            double turn = gamepad1.right_stick_x;

            if (switchFront) {
                driveNormal = -driveNormal;
                driveStrafe = -driveStrafe;
            }
            telemetry.addData("Motor", "n:%02.1f, s:%02.1f, t:%02.1f", driveNormal, driveStrafe, turn);

            float cap = 1.0f;
            // float backScale = 0.5f;
            double leftBackPower = Range.clip(driveNormal + turn + (driveStrafe), -cap, cap);
            double rightBackPower = Range.clip(driveNormal - turn - (driveStrafe), -cap, cap);
            double leftFrontPower = Range.clip(driveNormal + turn - driveStrafe, -cap, cap);
            double rightFrontPower = Range.clip(driveNormal - turn + driveStrafe, -cap, cap);

            double halfLeftBackPower = Range.clip(driveNormal + turn + driveStrafe, -0.25, 0.25);
            double halfRightBackPower = Range.clip(driveNormal - turn - driveStrafe, -0.25, 0.25);
            double halfLeftFrontPower = Range.clip(driveNormal + turn - driveStrafe, -0.25, 0.25);
            double halfRightFrontPower = Range.clip(driveNormal - turn + driveStrafe, -0.25, 0.25);

            boolean halfSpeed = gamepad1.right_stick_button;
            if (halfSpeed) {
                motorBackLeft.setPower(halfLeftBackPower);
                motorBackRight.setPower(halfRightBackPower);
                motorFrontLeft.setPower(halfLeftFrontPower);
                motorFrontRight.setPower(halfRightFrontPower);
                telemetry.addData("Motor", "half lb:%02.1f, rb:%02.1f, lf:%02.1f, rf:%02.1f", halfLeftBackPower, halfRightBackPower, halfLeftFrontPower, halfRightFrontPower);
            } else {
                motorBackLeft.setPower(leftBackPower);
                motorBackRight.setPower(rightBackPower);
                motorFrontLeft.setPower(leftFrontPower);
                motorFrontRight.setPower(rightFrontPower);
                telemetry.addData("Motor: f,b", "full left-back:%02.1f, (%d), rigt-back: %02.1f, (%d)", leftBackPower, motorBackLeft.getCurrentPosition(), rightBackPower, motorBackRight.getCurrentPosition());
                telemetry.addData("Motor: l,r", "full left-frnt:%02.1f, (%d), rigt-frnt: %02.1f, (%d)", leftFrontPower, motorFrontLeft.getCurrentPosition(), rightFrontPower, motorFrontRight.getCurrentPosition());
                //telemetry.addData("Motor", "SwitchFront ;%b", switchFront);
            }
        }

        if (useCrab) {
            if (gamepad1.b) {
                dropLock();
            }
            if (gamepad1.a) {
                raiseLock();
            }
        }

        if (useTouch) {
            if (digitalTouch.getState() == false) {
                telemetry.addData("Digital Touch", "we touching");
            }
        }
      
        telemetry.update();
    }
            
    protected void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    public void dropLock() {
            if (useCrab) {
                angleHand = 0.0;
            }
            crab.setPosition(angleHand);
    }

    public void raiseLock() {
        if (useCrab) {
            angleHand = 1.0;
        }
        crab.setPosition(angleHand);
    }
}