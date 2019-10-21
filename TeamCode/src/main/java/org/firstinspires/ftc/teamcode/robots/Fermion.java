package org.firstinspires.ftc.teamcode.robots;

import android.support.annotation.IntDef;
import android.util.Log;

import com.qualcomm.hardware.adafruit.AdafruitBNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.teamcode.RC;
import org.firstinspires.ftc.teamcode.newhardware.FXTSensors.FXTAnalogUltrasonicSensor;
import org.firstinspires.ftc.teamcode.newhardware.FXTSensors.TrackBall;
import org.firstinspires.ftc.teamcode.newhardware.FXTServo;
import org.firstinspires.ftc.teamcode.newhardware.Lights;
import org.firstinspires.ftc.teamcode.newhardware.Motor;
import org.firstinspires.ftc.teamcode.roboticslibrary.FXTCamera;
import org.firstinspires.ftc.teamcode.roboticslibrary.TaskHandler;
import org.firstinspires.ftc.teamcode.util.CircleDetector;
import org.firstinspires.ftc.teamcode.util.MathUtils;
import org.firstinspires.ftc.teamcode.util.PID;
import org.firstinspires.ftc.teamcode.util.VortexUtils;

/**
 * Created by FIXIT on 16-10-07.
 */
public class Fermion {

    /*
    STRING CONSTANTS
     */
    private String TAG = "FERMION";

    private String VEER_CHECK_TASK_KEY = "Fermion.VEERCHECK";
    public String WALL_FOLLOW_TASK_KEY = "Fermion.WALLFOLLOW";
    private String SHOOTER_TASK_KEY = "Fermion.SHOOTERCONTROL";

    /*
    SENSORS
     */
    public AdafruitBNO055IMU imu;
    public TrackBall mouse;

    private FXTAnalogUltrasonicSensor ultra;
    private FXTAnalogUltrasonicSensor ultraSide;

    private OpticalDistanceSensor leftBeacon;
    private OpticalDistanceSensor rightBeacon;
    private OpticalDistanceSensor ball;

    /*
    State Definitions
     */
    @IntDef({Robot.LEFT, Robot.RIGHT})
    public @interface LightSensors{}

    @IntDef({Robot.IN, Robot.OUT, Robot.STOP})
    public @interface CollectorStates{};

    /*
    LIGHTS
     */
    public Lights lights;

    /*
    MOTORS
     */
    public Motor leftFore;
    public Motor rightFore;
    public Motor leftBack;
    public Motor rightBack;
    public Motor collector;
    public Motor lifter;
    public Motor shooter;

    /*
    SERVOS
     */
    public FXTServo door;
    public FXTServo capRelease;
    public FXTServo whisker1;
    public FXTServo whisker2;

    /*
    ROBOT DRIVING VARIABLES
     */
    private double targetAngle = 0;
    private double targetStrafeAngle = 0;
    private double commandedStrafeSpeedRightForeLeftBack = 0;
    private double commandedStrafeSpeedRightBackLeftFore = 0;
    private double commandedStrafeAngle = 0;
    private double targetSpeed = 0;

    /*
    DRIVING CONSTANTS
     */
    private final static double MINIMUM_TURNING_SPEED = 0.2;
    private final static double TURNING_ACCURACY_DEG = 2;

    private final static double MINIMUM_TRACKING_SPEED = 0.19;
    private final static double TRACKING_ACCURACY_TIKS = 280;

    /*
    LIGHT SENSOR CONSTANT
     */
    public final static double LIGHT_THRESHOLD = 0.3;

    /*
    SHOOTER CONSTANTS
     */
    public int shooterState = LOADED;
    public final static int LOADED = 0;
    public final static int LOADING = 1;
    public final static int FIRE = 2;
    public final static int FIRING = 3;

    /*
    PID ALGORITHMS
     */
    private PID veerAlgorithm = new PID(PID.Type.PID, RC.globalDouble("VeerProportional"), RC.globalDouble("VeerIntegral"), RC.globalDouble("VeerDerivative"));
    private PID wallAlgorithm = new PID(PID.Type.PD, 0.25, 0.2);

    public Fermion(boolean auto) {

        /*
        DRIVE SYSTEM
         */
        leftFore = new Motor("leftFore");
        leftFore.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFore.setMinimumSpeed(0);

        rightFore = new Motor("rightFore");
        rightFore.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFore.setMinimumSpeed(0);

        leftBack = new Motor("leftBack");
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMinimumSpeed(0);

        rightBack = new Motor("rightBack");
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMinimumSpeed(0);

        leftFore.setReverse(true);
        leftBack.setReverse(true);

        /*
        COLLECTOR
         */

        collector = new Motor("collector");
        collector.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        collector.setReverse(false);
        collector.setMotorType(Motor.MotorType.AM20);

        /*
        LIFTER
         */
        lifter = new Motor("lifter");
        lifter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lifter.setMotorType(Motor.MotorType.AM20);

        /*
        SHOOTER
         */
        shooter = new Motor("shooter");
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter.setMotorType(Motor.MotorType.AM60);
        shooter.resetEncoder();
        shooter.stop();

        /*
        LOADING SERVO
         */
        door = new FXTServo("door");
        door.addPos("open", 0.53);
        door.addPos("close", 0);
        door.goToPos("close");

        /*
        PRONG RELEASE SERVO
         */
        capRelease = new FXTServo("capRelease");
        capRelease.addPos("init", 0.65);
        capRelease.addPos("start", 0.1);
        capRelease.addPos("release", 0.9);
        capRelease.goToPos("init");

        /*
        BALL CLEARING SERVOS
         */
        whisker1 = new FXTServo("whisker1");
        whisker1.addPos("out", 0);
        whisker1.addPos("in", 1);
        whisker1.goToPos("in");

        whisker2 = new FXTServo("whisker2");
        whisker2.addPos("out", 1);
        whisker2.addPos("in", 0);
        whisker2.goToPos("in");

        Log.i(TAG, "Fermion: ");

        /*
        TRACKBALL
         */
        mouse = new TrackBall("rightFore", "rightBack");

        /*
        ADAFRUIT IMU
        Note: init process takes some time
         */
//        if (auto) {
//            BNO055IMU.Parameters params = new BNO055IMU.Parameters();
//            params.angleUnit = BNO055IMU.AngleUnit.DEGREES;
//            params.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
//
//            imu = (AdafruitBNO055IMU) RC.h.get(BNO055IMU.class, "adafruit");
//            imu.initialize(params);
//        }//if

        /*
        LIGHT SENSORS
        to line up with beacons
         */
        leftBeacon = RC.h.opticalDistanceSensor.get("leftBeacon");
        rightBeacon = RC.h.opticalDistanceSensor.get("rightBeacon");

        /*
        LIGHT SENSOR
        to detect collected balls
         */
        ball = RC.h.opticalDistanceSensor.get("ball");

        /*
        ULTRASONIC SENSORS
         */
        ultra = new FXTAnalogUltrasonicSensor("ultra");
        ultraSide = new FXTAnalogUltrasonicSensor("ultra2");

        /*
        ROBOT LIGHTS
         */
        lights = new Lights("lights");
        lights.setLightsState(0);
    }//Fermion

    /**
     * Applies intended driving speeds
     * Avoids premature application of drive speeds (e.g. between strafe() and veer())
     */
    public void usePlannedSpeeds() {
        leftFore.usePlannedSpeed();
        leftBack.usePlannedSpeed();
        rightFore.usePlannedSpeed();
        rightBack.usePlannedSpeed();
    }//usePlannedSpeeds

    /*
    MECANUM DRIVING METHODS
     */

    /**
     * Makes the robot drive forward
     * @param speed the speed at which the robot drives
     */
    public void forward(double speed) {
        strafe(0, speed, true);
    }//forward

    /**
     * Makes the robot drive backward
     * @param speed the speed at which the robot drives
     */
    public void backward(double speed) {
        strafe(180, speed, true);
    }//forward

    public void left(double speed) {
        strafe(-90, speed, true);
    }//left

    public void right(double speed) {
        strafe(90, speed, true);
    }//right

    public void turnL(double speed) {
        TaskHandler.pauseTask(VEER_CHECK_TASK_KEY);

        leftFore.setPower(-speed);
        leftBack.setPower(-speed);
        rightFore.setPower(speed);
        rightBack.setPower(speed);
    }//turnL

    public void turnR(double speed) {
        TaskHandler.pauseTask(VEER_CHECK_TASK_KEY);

        leftFore.setPower(speed);
        leftBack.setPower(speed);
        rightFore.setPower(-speed);
        rightBack.setPower(-speed);
    }//turnL

    public void turnSingleL(double speed){
        TaskHandler.pauseTask(VEER_CHECK_TASK_KEY);
        rightFore.setPower(speed);
        rightBack.setPower(speed);
    }//turnSingleL

    public void turnSingleR(double speed){
        TaskHandler.pauseTask(VEER_CHECK_TASK_KEY);
        leftFore.setPower(speed);
        leftBack.setPower(speed);
    }//turnSingleR

    /*
    We probably need to pause/stop veer checkTimer...
    Do we reset our target angle (fully satisfy veerCheck) or just pause the veer checkTimer task (hold off veerCheck)?
     */
    public void stop() {
        commandedStrafeSpeedRightBackLeftFore = 0;
        commandedStrafeSpeedRightForeLeftBack = 0;
        leftFore.stop();
        rightFore.stop();
        leftBack.stop();
        rightBack.stop();

        resetTargetAngle();
    }//stop

    /*
    Allows robot to strafe in any direction, with 0° being the front of robot
                         0°
                         |
                 -90° –    – 90°
                         |
                       ±180°
     */
    public void strafe(double degrees, double speed, boolean setImmediately) {
        TaskHandler.resumeTask(VEER_CHECK_TASK_KEY);

        degrees += 45;

        double leftForeRightBack = Math.sin(Math.toRadians(degrees));
        double rightForeLeftBack = Math.cos(Math.toRadians(degrees));

        double multi = speed / Math.max(Math.abs(leftForeRightBack), Math.abs(rightForeLeftBack));
        leftForeRightBack *= multi;
        rightForeLeftBack *= multi;

        if (!setImmediately) {
            leftFore.setPlannedSpeed(leftForeRightBack);
            leftBack.setPlannedSpeed(rightForeLeftBack);
            rightFore.setPlannedSpeed(rightForeLeftBack);
            rightBack.setPlannedSpeed(leftForeRightBack);
        } else {
            leftFore.setPower(leftForeRightBack);
            leftBack.setPower(rightForeLeftBack);
            rightFore.setPower(rightForeLeftBack);
            rightBack.setPower(leftForeRightBack);
        }//else

        commandedStrafeSpeedRightBackLeftFore = leftForeRightBack;
        commandedStrafeSpeedRightForeLeftBack = rightForeLeftBack;
        commandedStrafeAngle = degrees - 45;

        Log.i("Left Fore", leftFore.returnCurrentState());
        Log.i("Left Back", leftBack.returnCurrentState());
        Log.i("Right Fore", rightFore.returnCurrentState());
        Log.i("Right Back", rightBack.returnCurrentState());
    }//strafe

    /*
    TRACKING METHODS
     */

    public void absoluteTrack(TrackBall.Point dest, double speed, boolean turnFirst) {

        TrackBall.Point start = mouse.getAbsoluteCoord().multiply(3 * Math.PI * 25.4 / 1440);

        Log.i("AbsoluteInfo", start.toString());

        TrackBall.Point change = dest.subtract(start);

        if (turnFirst) {

            double degrees = MathUtils.cvtAngleToNewDomain(change.acot());

            Log.i("Info", degrees + ", " + change.toString() + ", " + change.hypot());
            absoluteIMUTurn(degrees, 0.6);

            start = mouse.getAbsoluteCoord().multiply(3 * Math.PI * 25.4 / 1440);

            change = dest.subtract(start);

            Log.i("Info", degrees + ", " + change.toString() + ", " + change.hypot());

            track(0, change.hypot(), speed);
        } else {
            double degrees = MathUtils.cvtAngleToNewDomain(change.acot());

            track(degrees, change.hypot(), speed);
        }//else

    }

    //RELATIVE TRACKING
    public void track(double degrees, double mm, double speed) {
        TaskHandler.pauseTask(VEER_CHECK_TASK_KEY);

        mm *= (1440 / (25.4 * 3 * Math.PI));

        TrackBall.Point dest = mouse.getEncTiks().add(new TrackBall.Point(mm * Math.sin(Math.toRadians(degrees)), mm * Math.cos(Math.toRadians(degrees))));

        Log.i("Start, Dest", mouse.getEncTiks() + ", " + dest + "");

        while (RC.l.opModeIsActive()) {
            Log.i("EncTiks", mouse.getEncTiks() + "");

            double distanceRemaining = dest.subtract(mouse.getEncTiks()).hypot();

            if (distanceRemaining <= TRACKING_ACCURACY_TIKS) {
                break;
            }//if

            double nextSpeed = ((speed - 0.1) * (distanceRemaining / mm)) + 0.1;
            double nextDegrees = dest.subtract(mouse.getEncTiks()).acot();

            strafe(nextDegrees, nextSpeed, true);
            veerCheck(false);
        }//while

        stop();
        TaskHandler.resumeTask(VEER_CHECK_TASK_KEY);
    }//track

    public void imuTurnL(double degrees, double speed) {

        TaskHandler.pauseTask(VEER_CHECK_TASK_KEY);

        if(degrees < TURNING_ACCURACY_DEG) return;
        turnL(speed);

        this.targetAngle = MathUtils.cvtAngleToNewDomain(targetAngle - degrees);

        double beginAngle = MathUtils.cvtAngleToNewDomain(getIMUAngle()[0]);
        double targetAngle = MathUtils.cvtAngleToNewDomain(beginAngle - degrees);

        Log.i("targetAngleIMUTURNL", targetAngle + "");

        while (RC.l.opModeIsActive()) {

            double currentAngle = MathUtils.cvtAngleToNewDomain(getIMUAngle()[0]);
            double angleToTurn = MathUtils.cvtAngleJumpToNewDomain(currentAngle - targetAngle);

            Log.i("CurrentAngleXAIMUTURNL", currentAngle + "");
            Log.i("AngleToTurnIMUTURNL", angleToTurn + "");

            turnL(Math.signum(angleToTurn) * (Math.abs(angleToTurn) / 180 * (speed - MINIMUM_TURNING_SPEED) + MINIMUM_TURNING_SPEED));

            if (Math.abs(angleToTurn) < TURNING_ACCURACY_DEG) {
                break;
            }//if
        }//while

        stop();
        TaskHandler.resumeTask(VEER_CHECK_TASK_KEY);
    }//imuTurnL

    public void imuTurnR(double degrees, double speed) {

        TaskHandler.pauseTask(VEER_CHECK_TASK_KEY);

        if(degrees < TURNING_ACCURACY_DEG) return;
        turnR(speed);

        this.targetAngle = MathUtils.cvtAngleToNewDomain(targetAngle + degrees);

        double beginAngle = MathUtils.cvtAngleToNewDomain(getIMUAngle()[0]);
        double targetAngle = MathUtils.cvtAngleToNewDomain(beginAngle + degrees);

        while (RC.l.opModeIsActive()) {

            double currentAngle = MathUtils.cvtAngleToNewDomain(getIMUAngle()[0]);
            double angleToTurn = MathUtils.cvtAngleJumpToNewDomain(targetAngle - currentAngle);

            turnR(angleToTurn / 180 * (speed - MINIMUM_TURNING_SPEED) + MINIMUM_TURNING_SPEED);

            if (angleToTurn < TURNING_ACCURACY_DEG) {
                break;
            }//if
        }//while


        stop();
        TaskHandler.resumeTask(VEER_CHECK_TASK_KEY);
    }//imuTurnR

    public void absoluteIMUTurn(double degrees, double speed) {
        double currentAngle = MathUtils.cvtAngleToNewDomain(getIMUAngle()[0]);

        double toTurn = MathUtils.cvtAngleToNewDomain(degrees - currentAngle);

        Log.i("ToTurnIMUTURNL", toTurn + "");

        if (toTurn < 0) {
            imuTurnL(-toTurn, speed);
        } else {
            imuTurnR(toTurn, speed);
        }//else
        setTargetAngle(degrees);
    }//absoluteIMUTurn


    public void singleIMUTurnL(double degrees, double speed) {
        TaskHandler.pauseTask(VEER_CHECK_TASK_KEY);

        if(degrees < TURNING_ACCURACY_DEG) return;
        turnSingleL(speed);

        this.targetAngle = MathUtils.cvtAngleToNewDomain(targetAngle + degrees);

        double beginAngle = MathUtils.cvtAngleToNewDomain(getIMUAngle()[0]);
        double targetAngle = MathUtils.cvtAngleToNewDomain(beginAngle + degrees);

        while (RC.l.opModeIsActive()) {

            double currentAngle = MathUtils.cvtAngleToNewDomain(getIMUAngle()[0]);
            double angleToTurn = MathUtils.cvtAngleJumpToNewDomain(targetAngle - currentAngle);

            turnSingleR(angleToTurn / 180 * (speed - MINIMUM_TURNING_SPEED) + MINIMUM_TURNING_SPEED);

            if (angleToTurn < TURNING_ACCURACY_DEG) {
                break;
            }//if
        }//while

        stop();
        TaskHandler.resumeTask(VEER_CHECK_TASK_KEY);
    }//singleIMUTurnL

    public void singleIMUTurnR(double degrees, double speed) {
        TaskHandler.pauseTask(VEER_CHECK_TASK_KEY);

        if(degrees < TURNING_ACCURACY_DEG) return;
        turnSingleR(speed);

        this.targetAngle = MathUtils.cvtAngleToNewDomain(targetAngle + degrees);

        double beginAngle = MathUtils.cvtAngleToNewDomain(getIMUAngle()[0]);
        double targetAngle = MathUtils.cvtAngleToNewDomain(beginAngle + degrees);

        while (RC.l.opModeIsActive()) {

            double currentAngle = MathUtils.cvtAngleToNewDomain(getIMUAngle()[0]);
            double angleToTurn = MathUtils.cvtAngleJumpToNewDomain(targetAngle - currentAngle);

            turnSingleR(angleToTurn / 180 * (speed - MINIMUM_TURNING_SPEED) + MINIMUM_TURNING_SPEED);

            if (angleToTurn < TURNING_ACCURACY_DEG) {
                break;
            }//if
        }//while

        stop();
        TaskHandler.resumeTask(VEER_CHECK_TASK_KEY);
    }//imuTurnR

    public void absoluteSingleIMUTurn(double degrees, double speed) {
        double currentAngle = MathUtils.cvtAngleToNewDomain(getIMUAngle()[0]);

        double toTurn = MathUtils.cvtAngleJumpToNewDomain(degrees - currentAngle);

        if (toTurn < 0) {
            singleIMUTurnL(-toTurn, speed);
        } else {
            singleIMUTurnR(toTurn, speed);
        }//else
        setTargetAngle(degrees);
    }//absoluteIMUTurn


    /*
    VEER CHECK
     */

    public void resetTargetAngle() {
        this.targetAngle = MathUtils.cvtAngleToNewDomain(getIMUAngle()[0]);
    }//resetTargetAngle

    public void setTargetAngle(double targetAngle){
        this.targetAngle = targetAngle;
    }//setTargetAngle

    public double getTargetAngle() {
        return targetAngle;
    }//getTargetAngle

    public void addVeerCheckRunnable() {
        addVeerCheckRunnable(false);
    }//addVeerCheckRunnable

    public void addVeerCheckRunnable(final boolean preservingStrafeSpeed) {
        TaskHandler.addLoopedTask(VEER_CHECK_TASK_KEY, new Runnable() {
            @Override
            public void run() {
                veerCheck(preservingStrafeSpeed);
            }
        }, 5);
    }//addVeerCheckRunnable

    //to be used via TaskHandler
    //essentially, we need to turn and strafe at the same time
    //uses PID
    public void veerCheck(boolean preservingStrafeSpeed) {
        double currentAngle = MathUtils.cvtAngleToNewDomain(getIMUAngle()[0]);

        Log.i(VEER_CHECK_TASK_KEY, "PID Angles: " + currentAngle + ", " + targetAngle);

        double angleError = MathUtils.cvtAngleJumpToNewDomain(targetAngle - currentAngle);

        double strength = Math.signum(angleError) * Math.abs(veerAlgorithm.update(angleError));

        Log.i(VEER_CHECK_TASK_KEY, "PID Val: " + strength);

        veer(strength, preservingStrafeSpeed, true);
    }//veerCheck

    //Changes the robot's angle while letting it keep strafing
    //if speed < 0, robot will veer left
    //if speed > 0, robot will veer right
    //if preservingStrafeSpeed is true, then the robot's strafing speed won't be affected
    //if preservingStrafeSpeed is false, then the robot might slow down to accomodate veering
    public void veer(double speed, boolean preservingStrafeSpeed, boolean setImmediately) {

        speed = Math.signum(speed) * Math.min(1, Math.abs(speed));

        double leftForePower = commandedStrafeSpeedRightBackLeftFore;
        double leftBackPower = commandedStrafeSpeedRightForeLeftBack;
        double rightForePower = commandedStrafeSpeedRightForeLeftBack;
        double rightBackPower = commandedStrafeSpeedRightBackLeftFore;

        if (!setImmediately) {
            leftForePower = (leftFore.getPlannedSpeed() + rightBack.getPlannedSpeed()) / 2.0;
            leftBackPower = (rightFore.getPlannedSpeed() + leftBack.getPlannedSpeed()) / 2.0;
            rightForePower = (rightFore.getPlannedSpeed() + leftBack.getPlannedSpeed()) / 2.0;
            rightBackPower = (leftFore.getPlannedSpeed() + rightBack.getPlannedSpeed()) / 2.0;
        }//if

        if (preservingStrafeSpeed) {

            double maxCutOff = Math.max(Math.max(Math.abs(leftBackPower + speed), Math.abs(leftForePower + speed)), Math.max(Math.abs(rightBackPower - speed), Math.abs(rightForePower - speed)));
            maxCutOff -= 1;

            if (maxCutOff > 0) {
                speed -= maxCutOff;
            }//if

            leftForePower += speed;
            leftBackPower += speed;
            rightForePower -= speed;
            rightBackPower -= speed;
        } else {

            double max = MathUtils.max(Math.abs(leftBackPower + speed), Math.abs(leftForePower + speed), Math.abs(rightBackPower - speed), Math.abs(rightForePower - speed));
            double maxOriginal = MathUtils.max(Math.abs(leftBackPower), Math.abs(leftForePower), Math.abs(rightBackPower), Math.abs(rightForePower));

            if (max > 1) {
                double maxAllowed = 1 - Math.abs(speed);

                leftForePower *= maxAllowed / maxOriginal;
                leftBackPower *= maxAllowed / maxOriginal;
                rightForePower *= maxAllowed / maxOriginal;
                rightBackPower *= maxAllowed / maxOriginal;
            }//if

            leftForePower += speed;
            leftBackPower += speed;
            rightForePower -= speed;
            rightBackPower -= speed;

        }//else

        Log.i(VEER_CHECK_TASK_KEY, "LF: " + leftForePower + ", LB: " + leftBackPower + ", RF: " + rightForePower + ", RB: " + rightBackPower);

        if (!setImmediately) {
            leftFore.setPlannedSpeed(leftForePower);
            leftBack.setPlannedSpeed(leftBackPower);
            rightFore.setPlannedSpeed(rightForePower);
            rightBack.setPlannedSpeed(rightBackPower);
        } else {
            leftFore.setPower(leftForePower);
            leftBack.setPower(leftBackPower);
            rightFore.setPower(rightForePower);
            rightBack.setPower(rightBackPower);
        }//else

    }//veer

    /*
    WALL FOLLOWING
     */
    public void startWallFollowing(int ultrasonicIdx, int strafeAngle, final double targetSpeed, final int targetDistance){

        targetStrafeAngle = strafeAngle;
        this.targetSpeed = targetSpeed;
        strafe(strafeAngle, targetSpeed, true);

        //TaskHandler.removeTask(WALL_FOLLOW_TASK_KEY);

        TaskHandler.pauseTask(VEER_CHECK_TASK_KEY);

        final FXTAnalogUltrasonicSensor us = getUltrasonicFromIdx(ultrasonicIdx);

        TaskHandler.addLoopedTask(WALL_FOLLOW_TASK_KEY, new Runnable() {
            @Override
            public void run() {
                wallFollow(us, targetDistance);
                veerCheck(false);
            }
        }, 10);
    }//startWallFollowing

    public void endWallFollowing(){
        TaskHandler.removeTask(WALL_FOLLOW_TASK_KEY);
        TaskHandler.resumeTask(VEER_CHECK_TASK_KEY);
    }//endWallFollowing

    public void setTargetSpeed(double speed){
        targetSpeed = speed;
    }

    private void wallFollow(FXTAnalogUltrasonicSensor us, int targetDistance) {
        double error = targetDistance - us.getDistance();
        if (Math.abs(error) > 200) {
            error = Math.signum(error) * 200;
        }//if

        if (targetStrafeAngle <= 0) {
            strafe(targetStrafeAngle - wallAlgorithm.update(error), targetSpeed, true);
        } else {
            strafe(targetStrafeAngle + wallAlgorithm.update(error), targetSpeed, true);
        }//else

        Log.i("WallFollowA", targetStrafeAngle - wallAlgorithm.update(error) + "");
        Log.i("WallFollowD", "," + us.getDistance() + "");
    }//wallFollow

    /*
    COLLECTOR
     */
    public void setCollectorState(@CollectorStates int state){
        switch (state){
            case Robot.IN: collector.setPower(1);
                break;
            case Robot.OUT: collector.setPower(-1);
                break;
            case Robot.STOP: collector.stop();
        }//switch
    }//setCollectorState

    /*
    SHOOTER
     */
    public void shoot(){
        if (shooterState == LOADED) {
            shooterState = FIRE;
        }//if
    }//shoot

    public void loadShooter() {
        if (shooterState != FIRE && shooterState != FIRING) {
            shooterState = LOADING;
        }//if
    }//loadShooter

    public int getShooterState() {
        return shooterState;
    }//getShooterState

    private void updateShooter() {
        if (shooterState == FIRE) {
            shooterState = FIRING;
            shooter.addToTarget(shooter.getNumTiksPerRev() / 2);
            shooter.setPower(1);

            while (shooter.getBaseCurrentPosition() < shooter.getTarget()) {
                try {
                    Thread.sleep(1);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }//catch
            }//while

            shooter.stop();

            shooterState = LOADING;

            door.goToPos("open");
            try {
                Thread.sleep(100);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            setCollectorState(Robot.IN);

            try {
                Thread.sleep(1000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }//catch

            door.goToPos("close");
            setCollectorState(Robot.STOP);

            shooterState = LOADED;
        }
        else if (shooterState == LOADING) {
            door.goToPos("open");
            setCollectorState(Robot.IN);

            try {
                Thread.sleep(1000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }//catch

            door.goToPos("close");
            setCollectorState(Robot.STOP);

            shooterState = LOADED;
        }//else
    }//updateShooter

    public void waitForShooterState(int state){
        while (RC.l.opModeIsActive() && shooterState != state){
            RC.l.idle();
        }//while
    }//waitForShooterState

    public void startShooterControl(){
        TaskHandler.addLoopedTask(SHOOTER_TASK_KEY, new Runnable(){
            @Override
            public void run() {
                updateShooter();
            }//run
        }, 5);
    }//startShooterControl

    /*
    CAP BALL LIFTER
     */

    public void liftCapBall () {
        lifter.setPower(-1);
    }//liftCapBall

    public void lowerCapBall () {
        lifter.setPower(1);
    }//lowerCapBall

    /*
    SENSOR METHODS
     */
    public double getLight(@LightSensors int config){
        if(config == Robot.LEFT){
            return leftBeacon.getLightDetected();
        } else {
            return rightBeacon.getLightDetected();
        }//else
    }//getLight

    public double getCollectorLightSensor() {
        return ball.getLightDetected();
    }//getCollectorLightSensor

    public boolean seesBall(){
        return ball.getLightDetected() > 0.05;
    }//seesBall

    public double getBatteryVoltage(){
        return RC.h.voltageSensor.get("Motor Controller 1").getVoltage();
    }//getBatteryVoltage

    public double[] getIMUAngle() {
        Orientation orient = imu.getAngularOrientation();

        return new double[] {-orient.firstAngle, -orient.secondAngle, -orient.thirdAngle};
    }//getIMUAngle

    public FXTAnalogUltrasonicSensor getUltrasonicFromIdx(int idx) {
        switch(idx) {
            case 0: return ultra;
            case 1: return ultraSide;
        }//switch

        return null;
    }//getUltrasonicFromIdx

    public double getUltrasonicDistance(int idx) {
        return getUltrasonicFromIdx(idx).getDistance();
    }//getUltrasonicDistance


    public double[] lookForCircleWithTimeout(FXTCamera cam, int timeout) {

        long start = System.currentTimeMillis();
        double[] circle = null;
        while (RC.l.opModeIsActive() && System.currentTimeMillis() - start < timeout) {
            circle = CircleDetector.findBestCircle(cam.photo(), true);

            if (circle[0] != -1) {
                break;
            }//if
        }//while

        return circle;
    }

    @Deprecated
    public void strafeToBeacon(VuforiaTrackableDefaultListener beacon, double bufferDistance, double speed, boolean strafe,
                               double robotAngle, VectorF coordinate) {

        if (beacon.getPose() != null) {
            VectorF trans = beacon.getPose().getTranslation();

            Log.i(TAG, "strafeToBeacon: " + trans);

            trans = VortexUtils.navOffWall(trans, robotAngle, coordinate);

            Log.i(TAG, "strafeToBeacon: " + trans);

            double angle = Math.toDegrees(Math.atan2(trans.get(0), trans.get(2)));

            Log.i(TAG, "strafeToBeacon: " + angle);


            if (strafe) {
                track(angle, Math.hypot(trans.get(0), trans.get(2)) - bufferDistance, speed);
            } else {
                if (angle < 0) {
                    imuTurnL(-angle, speed);
                } else {
                    imuTurnR(angle, speed);
                }//else

                track(0, Math.hypot(trans.get(0), trans.get(2) - bufferDistance), speed);
            }//else

        } else {
            RC.t.addData("FERMION", "Strafe To Beacon failed: Beacon not visible");
        }//else
    }//strafeToBeacon

    public void clearBall(){
        whisker1.goToPos("out");
        if(RC.o instanceof LinearOpMode){
            RC.l.sleep(300);
        }
        whisker2.goToPos("out");
    }

    public void resetBallClearing(){
        whisker1.goToPos("in");
        whisker2.goToPos("in");
    }

}//Fermion
