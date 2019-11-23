/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.content.Context;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

@Autonomous(name="DM: Auto Mecanum", group="Pushbot")
//@Disabled
public class DM_Auto_Mecanum extends LinearOpMode {

    /* Declare OpMode members. */
//    HardwarePushbot         robot   = new HardwarePushbot();   // Use a Pushbot's hardware
    private DcMotor frontLeft, frontRight, backLeft, backRight;
    ColorSensor colorSensor;    // Hardware Device Object
    ColorSensor colorSensor2;
    float hsvValues[] = {0F,0F,0F};
    final float values[] = hsvValues;
    int relativeLayoutId;
    View relativeLayout;

    private ElapsedTime     runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 1440 / 2 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 / 3 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 3.937 ;   // For figuring circumference - 100mm
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED_SLOW        = 0.4;
    static final double     DRIVE_SPEED             = 1.0;
    static final double     TURN_SPEED              = 0.15;
    boolean                 soundPlaying            = false;

    public final static int     COLOR_RED = 1;
    public final static int     COLOR_BLUE = 2;
    public final static int     COLOR_MIN = 50;
    public final static int     COLOR_MIN2 = 500;

    // Gyro related initialization
    BNO055IMU               imu;
    Orientation             lastAngles = new Orientation();
    double                  globalAngle, correction;

    // Vuforia and Tensorflow related initialization
    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";

    private static final String VUFORIA_KEY =
            "ATg5zNT/////AAABmQhNspqC40DCvUSavEYlElMBRSLlK9rhCxs+Jd4rqsraiBijuWYBeupEoJrKGOgaTaP2AuF8RIBHvXtJLaf6jEffF6Jfi0wxmwKfxCegMt4YezZ22wpiK2WfnvvjolMxcVFpKLo38wrA8n88Dy8G2Rg2HAu2HILsg+Sq6dfKpynpbQs8ycs46zHvZUWVp+BVdifSxoKC4RT9zwPtyykIUhiw2Nr1ueaHQKMYTda2EbhgZ/1LP4/fqSNHqZhcqbFTSL3Fcsup+a449TPBlERNWgJDoInJ4lT9iyopclF5tVKqS01xpbmEwAaDp/v5e/aV4HPupgGdRbQCVdIvHQv8XtS7VNT6+Y1wy9QX1MlonqGk";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    @Override
    public void runOpMode() {
        // Vuforia and Tensorflow related initialization
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();

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

        // Initialization for sound
        Context myApp = hardwareMap.appContext;
        int soundID = myApp.getResources().getIdentifier("ss_darth_vader", "raw", myApp.getPackageName());

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
//        robot.init(hardwareMap);

        frontLeft = hardwareMap.get(DcMotor.class, "front_left");
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight = hardwareMap.get(DcMotor.class, "front_right");
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft = hardwareMap.get(DcMotor.class, "back_left");
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight = hardwareMap.get(DcMotor.class, "back_right");
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        // Gyro
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        // Color Sensor related initialization
        // get a reference to the RelativeLayout so we can change the background
        // color of the Robot Controller app to match the hue detected by the RGB sensor.
        relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

        // get a reference to our ColorSensor object.
        colorSensor = hardwareMap.get(ColorSensor.class, "color_sensor");
        colorSensor.enableLed(true);
        colorSensor2 = hardwareMap.get(ColorSensor.class, "color_sensor2");
        colorSensor2.enableLed(true);


        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Status",  "Encoder Reset Done");
        telemetry.update();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        telemetry.addData("Status", "IMU Calibrating");
        telemetry.update();

        // make sure the imu gyro is calibrated before continuing.
        while (!imu.isGyroCalibrated()) {
            sleep(50);
            idle();
        }

        telemetry.addData("Status", "IMU Calibration Done");
        telemetry.update();

        resetAngle();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        moveForwardUntilColorFound( DRIVE_SPEED, COLOR_RED );

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        // S1: Forward 6 Inches with 4 Sec timeout
//        telemetry.addData("Status",  ">> S1 Started");
//        telemetry.update();
//        encoderDrive(DRIVE_SPEED_SLOW,  6,  6, 4.0);

        // S2: Turn right 90 degrees
//        telemetry.addData("Status",  ">> S2 Started");
//        telemetry.update();
//        encoderDrive(TURN_SPEED,   -12, -12, 4.0);
//        rotate(-90, TURN_SPEED);

        // S3: Reverse 6 Inches with 4 Sec timeout
//        telemetry.addData("Status",  ">> S3 Started");
//        telemetry.update();
//        encoderDrive(DRIVE_SPEED_SLOW, 6, 6, 4.0);

        // S4: Turn left 90 degrees
//        telemetry.addData("Status",  ">> S4 Started");
//        telemetry.update();
//        rotate(90, TURN_SPEED);

        // S5: Look for Skystone
/*
        telemetry.addData("Status",  ">> S5 Started");

        telemetry.update();

        boolean foundSkyStone = false;
        while (opModeIsActive()) {
            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());

                    // step through the list of recognitions and display boundary info.
                    for (Recognition recognition : updatedRecognitions) {
                        if (recognition.getLabel().equalsIgnoreCase("SkyStone")) {
                            telemetry.addData("Label: ", "SkyStone Found!");
                            foundSkyStone = true;
                        }

 */
/*
                        telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                        telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                recognition.getLeft(), recognition.getTop());
                        telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                recognition.getRight(), recognition.getBottom());
*/
/*
                    }
                    telemetry.update();
                }
            }

            if (foundSkyStone)
                break;
        }
*/

        // S6: Play sound
        // create a sound parameter that holds the desired player parameters.

        // S7: Turn left 90 degrees
//        telemetry.addData("Status",  ">> S7 Started");
//        telemetry.update();
//        rotate(90, TURN_SPEED);

        // S8: Move forward 108 inches with 10 Sec timeout
//        telemetry.addData("Status",  ">> S8 Started");
//        telemetry.update();
//        encoderDrive(DRIVE_SPEED, 72, 72, 4.5);

        // S9: Turn right 90 degrees
//        telemetry.addData("Status",  ">> S9 Started");
//        telemetry.update();
//        rotate(-90, TURN_SPEED);

        // S10: Move forward 24 inches with 6 Sec timeout
//        telemetry.addData("Status",  ">> S10 Started");
//        telemetry.update();
//        encoderDrive(DRIVE_SPEED, 16, 16, 2.0);

//        sleep(1000);     // pause for servos to move
        // Set the panel back to the default color

        relativeLayout.post(new Runnable() {
            public void run() {
                relativeLayout.setBackgroundColor(Color.WHITE);
            }
        });

//        telemetry.addData("Path", "Complete");
//        telemetry.update();

        // Vuforia and Tensorflow related clean-up
        if (tfod != null) {
            tfod.shutdown();
        }
    }

    public void moveForwardUntilColorFound( double speed, int color_to_stop ) {
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeft.setPower(speed);
        frontRight.setPower(speed);
        backLeft.setPower(speed);
        backRight.setPower(speed);

        boolean colorDetected = false;

        while (opModeIsActive() &&
                (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy()) &&
                !colorDetected ) {

            // convert the RGB values to HSV values.
            Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsvValues);

            // Use gyro to drive in a straight line.
//            correction = checkDirection();
            correction = 0;
            frontLeft.setPower(speed + correction);
            frontRight.setPower(speed - correction);
            backLeft.setPower(speed + correction);
            backRight.setPower(speed - correction);

            int red_value = colorSensor.red();
            int blue_value = colorSensor.blue();
            int green_value = colorSensor.green();

            int red_value2 = colorSensor2.red();
            int blue_value2 = colorSensor2.blue();
            int green_value2 = colorSensor2.green();

            switch( color_to_stop ) {
                case COLOR_RED:
                    if ( ( red_value > COLOR_MIN && red_value > blue_value ) ||
                           red_value2 > COLOR_MIN2 && red_value2 > blue_value2 )
                        colorDetected = true;
                    break;

                case COLOR_BLUE:
                    if ( ( blue_value > COLOR_MIN && blue_value > red_value ) ||
                            blue_value2 > COLOR_MIN2 && blue_value2 > red_value2 )
                        colorDetected = true;
                    break;

                default:
                    break;
            }

            // Display it for the driver.
//            telemetry.addData("LF", frontLeft.getCurrentPosition());
//            telemetry.addData("RF", frontRight.getCurrentPosition());
//            telemetry.addData("LB", backLeft.getCurrentPosition());
//            telemetry.addData("RB", backRight.getCurrentPosition());
            telemetry.addData("1:Red  ", red_value);
            telemetry.addData("1:Green", green_value);
            telemetry.addData("1:Blue ", blue_value);
            telemetry.addData("2:Red  ", red_value2);
            telemetry.addData("2:Green", green_value2);
            telemetry.addData("2:Blue ", blue_value2);
            telemetry.addData("Found: ", colorDetected);

            // change the background color to match the color detected by the RGB sensor.
            // pass a reference to the hue, saturation, and value array as an argument
            // to the HSVToColor method.
            relativeLayout.post(new Runnable() {
                public void run() {
                    relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, values));
                }
            });

            telemetry.update();
        }

        // Stop all motion;
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    /*
     *  Method to perfmorm a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
/*
    public void encoderDrive(double speed,

                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = leftMotor.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = rightMotor.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            leftMotor.setTargetPosition(newLeftTarget);
            rightMotor.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            leftMotor.setPower(Math.abs(speed));
            rightMotor.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (leftMotor.isBusy() && rightMotor.isBusy())) {

                // Check whether robot is almost at target
                int leftDiff = leftMotor.getTargetPosition() - leftMotor.getCurrentPosition();
                double slowDownFactor = 1.0;
                if (leftDiff < 2 * COUNTS_PER_INCH) {
                    slowDownFactor = (double) leftDiff / (2 * COUNTS_PER_INCH);
                }

                // Use gyro to drive in a straight line.
                correction = checkDirection();
                leftMotor.setPower((Math.abs(speed) - correction) * slowDownFactor);
                rightMotor.setPower((Math.abs(speed) + correction) * slowDownFactor);


                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        leftMotor.getCurrentPosition(),
                        rightMotor.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            leftMotor.setPower(0);
            rightMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }
*/

    // Gyro related routines
    /**
     * Resets the cumulative angle tracking to zero.
     */
    private void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right.
     */
    private double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    /**
     * See if we are moving in a straight line and if not return a power correction value.
     * @return Power adjustment, + is adjust left - is adjust right.
     */
    private double checkDirection()
    {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction, angle, gain = .10;

        angle = getAngle();

        if (angle == 0)
            correction = 0;             // no adjustment.
        else
            correction = -angle;        // reverse sign of angle for correction.

        correction = correction * gain;

        return correction;
    }

    /**
     * Rotate left or right the number of degrees. Does not support turning more than 180 degrees.
     * @param degrees Degrees to turn, + is left - is right
     */
    /*
    private void rotate(int degrees, double power)
    {
        double  leftPower, rightPower;

        // restart imu movement tracking.
        resetAngle();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        if (degrees < 0) {
            // turn right.
            leftPower = power;
            rightPower = -power;
        } else if (degrees > 0) {
            // turn left.
            leftPower = -power;
            rightPower = power;
        } else
            return;

        // set power to rotate.
        leftMotor.setPower(leftPower);
        rightMotor.setPower(rightPower);

        // rotate until turn is completed.
        if (degrees < 0) {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && getAngle() == 0) {}

            double diff = getAngle() - (double) degrees;
            while (opModeIsActive() && diff > 0) {
                if ( diff < 20.0 ) {
                    leftMotor.setPower(leftPower * diff / 20);
                    rightMotor.setPower(rightPower * diff / 20);
                }
                diff = getAngle() - degrees;
            }
        } else {
            // left turn.
            double diff = (double) degrees - getAngle();
            while (opModeIsActive() && diff > 0) {
                if ( diff < 20.0 ) {
                    leftMotor.setPower(leftPower * diff / 20.0);
                    rightMotor.setPower(rightPower * diff / 20.0);
                }
                diff = degrees - getAngle();
            }
        }

        // turn the motors off.
        rightMotor.setPower(0);
        leftMotor.setPower(0);

        // wait for rotation to stop.
        sleep(1000);

        // reset angle tracking on new heading.
        resetAngle();
    }
     */

    // Vuforia and Tensorflow related functions
    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
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
