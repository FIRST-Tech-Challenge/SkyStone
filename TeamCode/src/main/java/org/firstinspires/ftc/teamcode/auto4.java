package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

@Disabled
public class auto4 extends LinearOpMode {

    PIDController pidRotate, pidDrive;
    double globalAngle, power = .30, correction, rotation;
    Orientation lastAngles = new Orientation();
    BNO055IMU imu;
    DcMotor TL, TR, BL, BR;
    Servo hookLeft, hookRight;

    public double powerUp = 0.5, powerDown = -0.5;

    DistanceSensor distanceSensor;

    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";

    private static final String VUFORIA_KEY =
            "AYNlKWT/////AAABmUOJpYR5Ckn+j2H4V2NICucswBSgY0/bl0ZpMZYB7P30Me22HMLRjpIPDhq0k8mfaA/nHnYfYymiKJmb0x5mGJpFftppbCSQJhq8mQ0+MSliYMMZkC1kevVxkzHT25sDPptgAukVI2JGCz/+cEtLf8FsFUfYXlFTFCSLp4x9R0UWqa4VBBYFEhdyjRekayYy0qPiMP/RqXBgWJENVbHfqTfZRhnMcVfD3KsGpYB/bnTTE3kO37RTgVFLxuYAwRiw6eSXN/1A9v3fnpCempHT9MGF1LXnTDcQpUwkHzMhegLHUbxAg/KAeGcIV/yxBdKt7dO1/UZY89cXo1yLxGFHxeKOYp/R6S1SvGOnjjUSScYC";

    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;




    @Override
    public void runOpMode() throws InterruptedException {
        distanceSensor = hardwareMap.get(DistanceSensor.class, "dist");

        hookLeft = hardwareMap.get(Servo.class, "hook");
        hookRight = hardwareMap.get(Servo.class, "hooke");

        BL = hardwareMap.get(DcMotor.class, "BL");
        BR = hardwareMap.get(DcMotor.class, "BR");
        TR = hardwareMap.get(DcMotor.class, "TR");
        TL = hardwareMap.get(DcMotor.class, "TL");

        TL.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);

        TL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        TR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

        pidDrive = new PIDController(.05, 0, 0);
        pidRotate = new PIDController(.004, .00004, 0);

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

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();

        while (!isStopRequested() && !imu.isGyroCalibrated()) {
            sleep(50);
            idle();
        }

        waitForStart();

        pidDrive.setSetpoint(0);
        pidDrive.setOutputRange(0, power);
        pidDrive.setInputRange(-90, 90);
        pidDrive.enable();


        correction = pidDrive.performPID(getAngle());

        telemetry.addData("1 imu heading", lastAngles.firstAngle);
        telemetry.addData("2 global heading", globalAngle);
        telemetry.addData("3 correction", correction);
        telemetry.addData("4 turn rotation", rotation);
        telemetry.update();

        /*
        START MOVING UP
         */
        TL.setPower(-(power - correction));
        BL.setPower(-(power - correction));
        TR.setPower(-(power + correction));
        BR.setPower(-(power + correction));

        while(opModeIsActive()&& !tripWireActive(14)){
            telemetry.addData("STATUS: ", "going forward (PHASE 1)");
            telemetry.update();
        }

        rest();
        sleep(2000);

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());

                        // step through the list of recognitions and display boundary info.
                        int i = 0;
                        for (Recognition recognition : updatedRecognitions) {
                            telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                            telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                    recognition.getLeft(), recognition.getTop());
                            telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                    recognition.getRight(), recognition.getBottom());
                        }
                        telemetry.update();
                    }
                }
            }
        }

        

        if (tfod != null) {
            tfod.shutdown();
        }

        //STRAFE LEFT
        TL.setPower(powerUp + joltControl());
        TR.setPower(powerDown);
        BL.setPower(powerDown);
        BR.setPower(powerUp);

        /*
        GET CAMERA TO START FOR ANALYSIS
         */

    }

    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

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

    public void raiseDL() {
        hookLeft.setPosition(0.9);
        hookRight.setPosition(0.9);
    }

    public void rest() {
        TL.setPower(0);
        TR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);
    }

    private void dropDL() {
        hookLeft.setPosition(0.1);
        hookRight.setPosition(0.1);
    }

    public boolean tripWireActive(double triggerDist) {
        if (distanceSensor.getDistance(DistanceUnit.CM) < triggerDist) {
            return true;
        } else {
            return false;
        }
    }

    public void wait(double seconds) {
        ElapsedTime Intruntime = new ElapsedTime();
        Intruntime.reset();
        while (opModeIsActive() && Intruntime.seconds() < seconds) {
            telemetry.addData("Status: ", "Executing the current Phase");
            telemetry.update();
        }
    }

    public void wait(double seconds, String phase) {
        ElapsedTime Intruntime = new ElapsedTime();
        Intruntime.reset();
        while (opModeIsActive() && Intruntime.seconds() < seconds) {
            telemetry.addData("Status: ", phase);
            telemetry.update();
        }
    }

    public void strafeCorrection(){
        double pulseStrength = 0.05;

        resetAngle();

        if (getAngle()>5){
            TL.setPower(TL.getPower() + pulseStrength);
        }
        if (getAngle()<-5) {
            BL.setPower(BL.getPower() + pulseStrength);
        }
        else {
            pulseStrength = 0;
        }
    }

    private void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    public double joltControl() {
        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();

        if (runtime.seconds() < 1.2) {
            return 0.05;
        } else {
            return 0.0;
        }
    }

    /**
     * Get current cumulative angle rotation from last reset.
     *
     * @return Angle in degrees. + = left, - = right from zero point.
     */
    private double getAngle() {
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
     * Rotate left or right the number of degrees. Does not support turning more than 359 degrees.
     *
     * @param degrees Degrees to turn, + is left - is right
     */
    private void rotate(int degrees, double power) {
        // restart imu angle tracking.
        resetAngle();

        // if degrees > 359 we cap at 359 with same sign as original degrees.
        if (Math.abs(degrees) > 359) degrees = (int) Math.copySign(359, degrees);

        // start pid controller. PID controller will monitor the turn angle with respect to the
        // target angle and reduce power as we approach the target angle. This is to prevent the
        // robots momentum from overshooting the turn after we turn off the power. The PID controller
        // reports onTarget() = true when the difference between turn angle and target angle is within
        // 1% of target (tolerance) which is about 1 degree. This helps prevent overshoot. Overshoot is
        // dependant on the motor and gearing configuration, starting power, weight of the robot and the
        // on target tolerance. If the controller overshoots, it will reverse the sign of the output
        // turning the robot back toward the setpoint value.

        pidRotate.reset();
        pidRotate.setSetpoint(degrees);
        pidRotate.setInputRange(0, degrees);
        pidRotate.setOutputRange(0, power);
        pidRotate.setTolerance(1);
        pidRotate.enable();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        // rotate until turn is completed.

        if (degrees < 0) {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && getAngle() == 0) {
                TL.setPower(power);
                BL.setPower(power);
                TR.setPower(-power);
                BR.setPower(-power);
                sleep(100);
            }

            do {
                power = pidRotate.performPID(getAngle()); // power will be - on right turn.
                TL.setPower(-power);
                BL.setPower(-power);
                TR.setPower(power);
                BR.setPower(power);
            } while (opModeIsActive() && !pidRotate.onTarget());
        } else    // left turn.
            do {
                power = pidRotate.performPID(getAngle()); // power will be + on left turn.
                TL.setPower(-power);
                BL.setPower(-power);
                TR.setPower(power);
                BR.setPower(power);
            } while (opModeIsActive() && !pidRotate.onTarget());

        // turn the motors off.
        TL.setPower(0);
        TR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);

        rotation = getAngle();

        // wait for rotation to stop.
        sleep(500);

        // reset angle tracking on new heading.
        resetAngle();
    }
}
