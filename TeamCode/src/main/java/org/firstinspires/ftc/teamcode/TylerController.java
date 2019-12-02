package org.firstinspires.ftc.teamcode;

import android.content.Context;

import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.skystone.SkystoneDetector;
import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaSkyStone;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

/**
 *
 */
@TeleOp(name="Tyler TeleOp", group="a")
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

    //elevator
    private DcMotor elevator;

    //Arm
    private DcMotor crane;
    //private DcMotor extender;
    private Servo hand;

    //Succ
    private DcMotor leftSucc;
    private DcMotor rightSucc;

    // Crab state.
    private Servo crab;
    private DigitalChannel digitalTouch;

    // Hardware Device Object
    private double angleHand;
    private double angleAnkle;

    // Hack stuff.
    private boolean useMotors = true;
    private boolean useEncoders = true;
    private boolean useArm = true; // HACK
    private boolean useLifter = false;
    private boolean useCrab = true;
    private boolean useDropper = false;
    private boolean useTouch = true;
    private boolean useRange = true;
    private boolean useStoneDetector = false;
    private boolean useOpenCvDisplay = false;
    private boolean useElevator = true;
    private boolean useSucc = true;


    //Movement State
    private int armState;
    private int extenderTarget;
    private int lifterState;
    private int lifterExtenderTarget;
    private int elevatorExtenderTarget;
    private int extenderStartPostion = 0;
    private int elevatorStartPosition = 0;
    private int shoulderTarget;
    private int shoulderStartPosition = 0;

    //Drive State
    private boolean switchFront = false;

    //distance sensors
    private DistanceSensor rangeFront;
    private DistanceSensor rangeBack;
    private DistanceSensor rangeLeft;
    private DistanceSensor rangeRight;

    // stone detector state.
    private SkystoneDetector detector;
    OpenCvCamera phoneCam;

    //elevator
    private DigitalChannel elevatorMagnet;
    protected boolean useMagnets = true;

    /**
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        // Initialize the motors.
        if (useMotors) {
            try {
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
            } catch (Exception e) {
                telemetry.addData("Motor", "exception on init: " + e.toString());
                motorBackLeft = null;
                motorBackRight = null;
                motorFrontLeft = null;
                motorFrontRight = null;
            }

            if (motorBackLeft == null) {
                telemetry.addData("Motor", "You forgot to set up the back left motor, set up the motors:");
                useMotors = false;
            }
            if (motorBackRight == null) {
                telemetry.addData("Motor", "You forgot to set up the back right motor, set up the motors:");
                useMotors = false;
            }
            if (motorFrontLeft == null) {
                telemetry.addData("Motor", "You forgot to set up the front left motors, set up the motors:");
                useMotors = false;
            }
            if (motorFrontRight == null) {
                telemetry.addData("Motor", "You forgot to set up the front right motors, set up the motors:");
                useMotors = false;
            }

            if (useMotors && useEncoders) {
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

        if (useArm) {
            try {
                crane = hardwareMap.get(DcMotor.class, "motorCrane");
                hand = hardwareMap.get(Servo.class, "servoGripper");
              //  extender = hardwareMap.get(DcMotor.class, "motorExtend");

            } catch (Exception e) {
                telemetry.addData("Arm", "exception on init: " + e.toString());
                crane = null;
                hand = null;
                //extender = null;
            }
            if (crane == null) {
                telemetry.addData("Arm", "You forgot to set up crane, set up Crane");
                useArm = false;
            }
            if (hand == null) {
                telemetry.addData("Arm", "You forgot to set up hand, set up the hand");
                useArm = false;
            }
            /*if (extender == null) {
                telemetry.addData("Arm", "You forgot to set up extender, set up the extender");
                useArm = false;
            }*/
        }


        if (useCrab) {
            try {
                crab = hardwareMap.get(Servo.class, "servoCrab");
            } catch (Exception e) {
                telemetry.addData("Crab", "exception on init: " + e.toString());
                crab = null;
            }

            if (crab == null) {
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
            } else {
                digitalTouch.setMode(DigitalChannel.Mode.INPUT);
            }
        }

        if (useElevator) {
            try {
                elevator = hardwareMap.get(DcMotor.class, "elevator");
            } catch (Exception e) {
                telemetry.addData("elevator", "exception on init: " + e.toString());
                useElevator = false;
            }
        }

        if (useSucc) {
            try {
                leftSucc = hardwareMap.get(DcMotor.class, "leftSucc");
                rightSucc = hardwareMap.get(DcMotor.class, "rightSucc");

            } catch (Exception e) {
                telemetry.addData("Succ", "exception on init: " + e.toString());
                useElevator = false;
            }
        }


        if (useRange) {
            try {
                //initialize the four lidar sensors
                rangeFront = hardwareMap.get(DistanceSensor.class, "range_front");
                rangeBack = hardwareMap.get(DistanceSensor.class, "range_back");
                rangeLeft = hardwareMap.get(DistanceSensor.class, "range_left");
                rangeRight = hardwareMap.get(DistanceSensor.class, "range_right");
            } catch (Exception e) {
                telemetry.addData("Range", "exception on init: " + e.toString());
                rangeFront = null;
                rangeBack = null;
                rangeLeft = null;
                rangeRight = null;
            }
            if (rangeFront == null) {
                telemetry.addData("Range", "You forgot to set up rangeFront, set it up ");
                useRange = false;
            }
            if (rangeBack == null) {
                telemetry.addData("Range", "You forgot to set up rangeBack, set it up ");
                useRange = false;
            }
            if (rangeLeft == null) {
                telemetry.addData("Range", "You forgot to set up rangeLeft, set it up ");
                useRange = false;
            }
            if (rangeRight == null) {
                telemetry.addData("Range", "You forgot to set up rangeRight, set it up ");
                useRange = false;
            }

            //load sound file
            Context myApp = hardwareMap.appContext;
            bruhSoundID = myApp.getResources().getIdentifier("bruh", "raw", myApp.getPackageName());
            oofSoundID = myApp.getResources().getIdentifier("oof", "raw", myApp.getPackageName());
        }


        if (useStoneDetector) {

            if (useOpenCvDisplay) {
                int cameraMonitorViewId =
                        hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id",
                                hardwareMap.appContext.getPackageName());
                phoneCam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
                phoneCam.openCameraDevice();
                phoneCam.setPipeline(new SamplePipeline());
                phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            telemetry.addData("Stone", "initing");

            // Set up detector
            detector = new SkystoneDetector(); // Create detector
            //detector.
            //detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance()); // Initialize it with the app context and camera
            //detector.useDefaults(); // Set detector to use default settings

            // Optional tuning
           /* detector.downscale = 0.4; // How much to downscale the input frames

            detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
            //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
            detector.maxAreaScorer.weight = 0.005; //

            detector.ratioScorer.weight = 5; //
            detector.ratioScorer.perfectRatio = 1.0; // Ratio adjustment */


            //detector.enable(); // Start the detector!
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
        if (useStoneDetector) {
            // TODO
            // this.detector.disable();
        }
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
            if (gamepad1.start) {
                switchFront = !switchFront;
                sleep(500);
            }

            // Control the wheel motors.
            // POV Mode uses left stick to go forw ard, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            double driveNormal = -gamepad1.left_stick_y;
            if (Math.abs(driveNormal) < 0.1)
                driveNormal = 0.0; // Prevent the output from saying "-0. 0".

            double driveStrafe = gamepad1.left_stick_x;
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

        if (useArm) {
            // shoulder MANUAL CONTROL
           /* boolean pullUpOne = gamepad1.dpad_up;
            boolean pullOutOne = gamepad1.dpad_down;
            boolean pullUpTwo = gamepad2.dpad_up;
            boolean pullOutTwo = gamepad2.dpad_down;
            double pullPower = 0.0;
            if (pullUpOne||pullUpTwo) {
                pullPower = -0.8;
            } else if (pullOutOne || pullOutTwo) {
                pullPower = 0.8;
            }else if(pullUpOne && pullOutTwo) {
                pullPower = 0.8;
            }else if(pullOutOne && pullUpTwo){
                pullPower = -0.8;
            }
            shoulder.setPower(pullPower);

            if (pullPower != 0.0 || armState == 0) {
                // if anyone uses manual reset presets and turn everything off
                if (armState != 0) {
                    armState = 0;
                    extenderTarget = 0;
                    extender.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    extender.setPower(0);
                    shoulderTarget = 0;
                    shoulder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                }
                shoulder.setPower(pullPower);
            }*/

            // Control the extender: MANUAL CONTROL
            /*boolean extendOut = gamepad1.a;
            boolean extendIn = gamepad1.y;
            double extendManualPower = 0.0;
            if (extendOut) {
                extendManualPower = -0.5;
            }
            if (extendIn) {
                extendManualPower = 0.5;
            }
            if (extendManualPower != 0.0 || armState == 0) {
                if (armState != 0) {
                    // if anyone uses manual reset presets and turn everything off
                    armState = 0;
                    extenderTarget = 0;
                    extender.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    shoulderTarget = 0;
                    //shoulder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    //shoulder.setPower(0);
                }
                extender.setPower(extendManualPower);
            }*/

            // Control the crane.
            boolean suckOut = gamepad1.dpad_up;
            boolean suckIn = gamepad1.dpad_down;
            if (suckOut != false) {
                crane.setPower(1.0);
            } else if (suckIn != false) {
                crane.setPower(-1.0);
            } else {
                crane.setPower(0);
            }


            /*if(gamepad1.right_trigger){
                crane.setPower(1);
            }else if (gamepad1.left_trigger){
                crane.setPower(-1);
            }else{
                crane.setPower(0);
            }*/

            boolean closeHand = gamepad1.dpad_right;
            boolean openHand = gamepad1.dpad_left;
            if (closeHand) {
                hand.setPosition(1);
            } else if (openHand) {
                hand.setPosition(-1);
            }

            //telemetry.addData("Extender", "start: %d, curr: %d, target: %d, armState: %d", extenderStartPostion, extender.getCurrentPosition(), extenderTarget, armState);
        }

       if (useElevator) {
           boolean liftAllIn = gamepad1.a;
           boolean liftAllOut = gamepad1.y;
           if (liftAllOut) {
               startElevatorMoving(elevatorStartPosition + 4500);
           } else if (liftAllIn) {
               startElevatorMoving(elevatorStartPosition);
           }
       }

       if (useSucc){
           float succIn = gamepad1.right_trigger;
           boolean succOut = gamepad1.right_bumper;
           if (succIn == 1) {
               leftSucc.setPower(-1.0);
               rightSucc.setPower(1.0);

           } else if (succOut) {
               leftSucc.setPower(1.0);
               rightSucc.setPower(-1.0);

           }
       }

        if (useCrab) {
            if (gamepad1.b) {
                raiseCrab();
            }
            else{
                dropCrab();
            }
        }



        if (useTouch) {
            if (digitalTouch.getState() == false) {
                telemetry.addData("Digital Touch", "we touching");
            }
        }

        if (useStoneDetector) {
            /*
             * Send some stats to the telemetry
             */
            telemetry.addData("Frame Count", phoneCam.getFrameCount());
            telemetry.addData("FPS", String.format("%.2f", phoneCam.getFps()));
            telemetry.addData("Total frame time ms", phoneCam.getTotalFrameTimeMs());
            telemetry.addData("Pipeline time ms", phoneCam.getPipelineTimeMs());
            telemetry.addData("Overhead time ms", phoneCam.getOverheadTimeMs());
            telemetry.addData("Theoretical max FPS", phoneCam.getCurrentPipelineMaxFps());
            telemetry.update();

            /*
             * NOTE: stopping the stream from the camera early (before the end of the OpMode
             * when it will be automatically stopped for you) *IS* supported. The "if" statement
             * below will stop streaming from the camera when the "A" button on gamepad 1 is pressed.
             */
            if (gamepad1.a) {
                /*
                 * IMPORTANT NOTE: calling stopStreaming() will indeed stop the stream of images
                 * from the camera (and, by extension, stop calling your vision pipeline). HOWEVER,
                 * if the reason you wish to stop the stream early is to switch use of the camera
                 * over to, say, Vuforia or TFOD, you will also need to call closeCameraDevice()
                 * (commented out below), because according to the Android Camera API documentation:
                 *         "Your application should only have one Camera object active at a time for
                 *          a particular hardware camera."
                 *
                 * NB: calling closeCameraDevice() will internally call stopStreaming() if applicable,
                 * but it doesn't hurt to call it anyway, if for no other reason than clarity.
                 *
                 * NB2: if you are stopping the camera stream to simply save some processing power
                 * (or battery power) for a short while when you do not need your vision pipeline,
                 * it is recommended to NOT call closeCameraDevice() as you will then need to re-open
                 * it the next time you wish to activate your vision pipeline, which can take a bit of
                 * time. Of course, this comment is irrelevant in light of the use case described in
                 * the above "important note".
                 */
                phoneCam.stopStreaming();
                //webcam.closeCameraDevice();
            }

            /*
             * The viewport (if one was specified in the constructor) can also be dynamically "paused"
             * and "resumed". The primary use case of this is to reduce CPU, memory, and power load
             * when you need your vision pipeline running, but do not require a live preview on the
             * robot controller screen. For instance, this could be useful if you wish to see the live
             * camera preview as you are initializing your robot, but you no longer require the live
             * preview after you have finished your initialization process; pausing the viewport does
             * not stop running your pipeline.
             *
             * The "if" statements below will pause the viewport if the "X" button on gamepad1 is pressed,
             * and resume the viewport if the "Y" button on gamepad1 is pressed.
             */
            else if (gamepad1.x) {
                phoneCam.pauseViewport();
            } else if (gamepad1.y) {
                phoneCam.resumeViewport();
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

    protected void startElevatorMoving(int lTarget){
        // Get the current position.
        int elevatorStart = elevator.getCurrentPosition();
        telemetry.addData("ElevatorStartingPos.", "Starting %7d", elevatorStart);

        // Turn On RUN_TO_POSITION
        elevator.setTargetPosition(lTarget);
        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevator.setPower(0.75);

        //Set global Movement State
        elevatorExtenderTarget = lTarget;
    }
    protected boolean initMagnets() {
        if (useMagnets) {
            elevatorMagnet = hardwareMap.get(DigitalChannel.class, "elevatorMagnet");
            telemetry.addData("Magnet", "class:" + elevatorMagnet.getClass().getName());
            return true;


        } else {
            useMagnets = false;
            return false;

        }
    }

    protected boolean isElevatorMagnetOn() {
        return !elevatorMagnet.getState();
    }



    /*
     * An example image processing pipeline to be run upon receipt of each frame from the camera.
     * Note that the processFrame() method is called serially from the frame worker thread -
     * that is, a new camera frame will not come in while you're still processing a previous one.
     * In other words, the processFrame() method will never be called multiple times simultaneously.
     *
     * However, the rendering of your processed image to the viewport is done in parallel to the
     * frame worker thread. Thzat is, the amount of time it takes to render the image to the
     * viewport does NOT impact the amount of frames per second that your pipeline can process.
     *
     * IMPORTANT NOTE: this pipeline is NOT invoked on your OpMode thread. It is invoked on the
     * frame worker thread. This should not be a problem in the vast majority of cases. However,
     * if you're doing something weird where you do need it synchronized with your OpMode thread,
     * then you will need to account for that accordingly.
     */
    class SamplePipeline extends OpenCvPipeline {
        /*
         * NOTE: if you wish to use additional Mat objects in your processing pipeline, it is
         * highly recommended to declare them here as instance variables and re-use them for
         * each invocation of processFrame(), rather than declaring them as new local variables
         * each time through processFrame(). This removes the danger of causing a memory leak
         * by forgetting to call mat.release(), and it also reduces memory pressure by not
         * constantly allocating and freeing large chunks of memory.
         */

        @Override
        public Mat processFrame(Mat input) {
            /*
             * IMPORTANT NOTE: the input Mat that is passed in as a parameter to this method
             * will only dereference to the same image for the duration of this particular
             * invocation of this method. That is, if for some reason you'd like to save a copy
             * of this particular frame for later use, you will need to either clone it or copy
             * it to another Mat.
             */

            /*
             * Draw a simple box around the middle 1/2 of the entire frame
             */
            Imgproc.rectangle(
                    input,
                    new Point(
                            input.cols() / 4,
                            input.rows() / 4),
                    new Point(
                            input.cols() * (3f / 4f),
                            input.rows() * (3f / 4f)),
                    new Scalar(0, 255, 0), 4);

            /**
             * NOTE: to see how to get data from your pipeline to your OpMode as well as how
             * to change which stage of the pipeline is rendered to the viewport when it is
             * tapped, please see {@link PipelineStageSwitchingExample}
             */
            return input;
        }
    }
}

