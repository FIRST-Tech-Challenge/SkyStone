package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.util.Range;

import java.lang.*;

import java.lang.reflect.Field;
import java.util.List;
import java.util.Locale;


import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.ClassFactory;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

public class autoCopy {
    private DcMotor topLeft;
    private DcMotor topRight;
    private DcMotor botLeft;
    private DcMotor botRight;
    private DcMotor intakeR;
    private DcMotor intakeL;

    private DcMotor winchL;
    private DcMotor winchR;
    private Servo pullL;
    private Servo pullR;

    private Servo clawF;
    private Servo clawB;

    private Servo armL;
    private Servo armR;

    private Servo intakeServoL;
    private Servo intakeServoR;

    public int stoneSpot = 5;
    public double leftCoord = -100000;

    BNO055IMU imu;
    Orientation orientation = new Orientation();
    DcMotor.RunMode newRun;
    HardwareMap map;

    Orientation lastAngles = new Orientation();
    double globalAngle, power = .30, correction, rotation;
    PIDController pidRotate, pidDrive, pidStrafe;
    //Telemetry telemetry;


    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    private static final String VUFORIA_KEY = "AUw51u3/////AAABmS2SebfPGERUmfixnpbS89g79T2cQLWzcEcMv6u+RTGzrrvHwTVug45aIF3UiYJXKVzy/zhBFDleEJD2gEjPWWDQeYDV9k3htKwbHofAiOwRfivq8h2ZJIGcmUwiNT40UAEeUvQlKZXTcIYTrxiYmN4tAKEjmH5zKoAUfLefScv9gDDMwTYCKVm1M45M2a1VdIu0pMdoaJKo2DRZ3B+D+yZurFO/ymNtyAWME+1eE9PWyulZUmuUw/sDphp13KrdNHNbDUXwbunQN7voVm2HE5fWrFNtX5euVaPy/jedXTiM5KBeosXuemMeppimcTLHFvyhSwOMZMRhPT1Gus487FRWMt479sn2EhexfDCcd0JG";


    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    private static final float mmPerInch = 25.4f;
    private static final float mmTargetHeight = (6) * mmPerInch;          // the height of the center of the target image above the floor

    private double constant = 0;
    private int tempSleep = 100;


    public autoCopy(DcMotor.RunMode runMode, HardwareMap importedMap) {

        newRun = runMode;
        map = importedMap;
        topLeft = map.dcMotor.get("topLeft");
        topRight = map.dcMotor.get("topRight");
        botLeft = map.dcMotor.get("botLeft");
        botRight = map.dcMotor.get("botRight");

        winchL = map.dcMotor.get("winchL");
        winchR = map.dcMotor.get("winchR");

        intakeL = map.dcMotor.get("intakeL");
        intakeR = map.dcMotor.get("intakeR");

        pullL = map.servo.get("pullL");
        pullR = map.servo.get("pullR");

        armL = map.servo.get("armL");
        armR = map.servo.get("armR");

        clawF = map.servo.get("clawF");
        clawB = map.servo.get("clawB");

        intakeServoL = map.servo.get("intakeServoL");
        intakeServoR = map.servo.get("intakeServoR");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        //set reading to degrees

        imu = map.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);//axes order based on revHub orientation
        orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        constant = getHeading();//constant is meant to compensate for any difference marked at the beginning. We should be starting at 0 manually

        pidRotate = new PIDController(.016, 0, 0.033);

        // Set PID proportional value to produce non-zero correction value when robot veers off
        // straight line. P value controls samhow sensitive the correction is.
        pidDrive = new PIDController(0.003, 0.0005, 0.045);

        pidStrafe = new PIDController(0.045, 0.045, 0.38);


        // make sure the imu gyro is calibrated before continuing.
        while (!imu.isGyroCalibrated()) {
            sleep(50);
            idle();
        }
        pidRotate.disable();
        topLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        topRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        botLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        botRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        topLeft.setMode(newRun);//set to imported runMode
        topRight.setMode(newRun);
        botLeft.setMode(newRun);
        botRight.setMode(newRun);

        winchL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        winchR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intakeL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        topLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        topRight.setDirection(DcMotorSimple.Direction.REVERSE);//in reverse because of motor direction
        botLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        botRight.setDirection(DcMotorSimple.Direction.REVERSE);

        topLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        topRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        botLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        botRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }
    //all variable related to the dimensions of the robot or field
    private final double FieldSide = 141; //11.75 ft or 141 inches
    public final double ticksPerRev = 560; //dependent on the motor we use
    public final double wheelDiameter = 4.65; //in inches
    public final double robotLength = 17.5;
    public final double robotWidth = 17.5;

    //these are meant to check the orientation of our robot
    public boolean forwardIncreasesX = false;
    public boolean forwardDecreasesX = false;
    public boolean forwardIncreasesY = false;
    public boolean forwardDecreasesY = false;
    //alliance color
    public boolean red = false;
    public boolean blue = false;
    //side of the field
    public boolean foundation = false;
    public boolean sample = false;
    public boolean parkWall = false;

    public boolean pullReq = false;
    public boolean sleepReq = false;


    //coordinate system related variables
    public Coordinate foundationLocation = new Coordinate(0, 0);//univeral coordinate to move and add stones to foundation
    public Coordinate buildZone = new Coordinate(50, 100);
    public Coordinate[] sampleSpots = new Coordinate[6];//locations for every stone during sampling stage
    //if we want to intake stones
//    public Coordinate skystone = new Coordinate(-1, -1);//spot that a new skystone can be found
    public Coordinate bridge = new Coordinate(0, 0);//for parking
    public Coordinate robot = new Coordinate(0, 0);

    public int inchesToTicks(double inches) {
        return (int) ((inches / (Math.PI * wheelDiameter)) * ticksPerRev);//dividing the total revolutions completed by the motor(found by diving the desired inches by the wheel circumference), by the motor's encoder count for 1 rev.

    }
    public boolean motorsAreBusy() {
        return topLeft.isBusy() && topRight.isBusy() && botLeft.isBusy() && botRight.isBusy();
    }
    public double getProxy(double angle) {
        double curr = getHeading();
        double dist1 = Math.abs(curr - angle);
        if (Math.abs(dist1) > 180) {
            return Math.abs(360 - dist1);
        } else {
            return dist1;
        }
    }
    public void initialDown() {
        armL.setPosition(.3);
        armR.setPosition(.7);
        sleep(400);
        intakeServoL.setPosition(0.19);
        intakeServoR.setPosition(1);
    }
    public void pullDown() {
        pullL.setPosition(.48);
        pullR.setPosition(.5);
    }
    public void pullUp() {
        pullL.setPosition(.9);
        pullR.setPosition(0);
    }
    public void pullMid(){
        pullL.setPosition(.25);
        pullR.setPosition(.25);
    }
    public void intake() {
        intakeL.setPower(1);
        intakeR.setPower(-1);

    }
    public void intakeOff() {
        intakeL.setPower(0);
        intakeR.setPower(0);
    }
    public void output(long time) {
        intakeL.setPower(-1);
        intakeR.setPower(1);
        sleep(time);
        intakeL.setPower(0);
        intakeR.setPower(0);
    }
    public int armRound = 0;
    public void armOut() {
        if(armRound == 0) {
            armL.setPosition(.95);
            armR.setPosition(.05);
        }
        else if(armRound == 1){
            armL.setPosition(.8);
            armR.setPosition(.2);
        }
        else {
            armL.setPosition(.7);
            armR.setPosition(.3);
        }
        armRound++;
        armIsIn = false;
    }
    boolean armIsIn = true;
    public void armIn() {
        clawB.setPosition(0.23);
        armL.setPosition(0.08);
        armR.setPosition(0.92);
        armIsIn = true;
    }
    public void armMid(){
        armL.setPosition(.5);
        armR.setPosition(.5);

    }
    public void deposit() {
        armOut();
        sleep(400);
        release();
        sleep(100);
        armIn();
        sleep(200);
    }
    public int degreesToTicks(double degrees) {
        int ticks = (int) ((3700 / 360) * degrees);
        //int tick = (int) (26.388 * degrees);
        return ticks;
    }
    private int strafeToTicks(double inches) {
        return (int) (52 * inches);//55
    }

    public double ticksToInches(int ticks) {
        return ticks * (wheelDiameter * Math.PI / ticksPerRev);
    }

    public double getNiche() {
        double y = bridge.getY() - robot.getY();
        double angle = getHeading() + 180;
        return y / Math.abs(Math.cos(Math.toRadians(angle)));
    }

    public void resetTicks() {
        topRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        topLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        botRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        botLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void setRunMode(boolean runToPosition) {
        if (runToPosition) {
            topLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            topRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            botLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            botRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        } else {
            topLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            topRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            botLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            botRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void grip() {
        clawF.setPosition(0.85);
        clawB.setPosition(0.21);
    }

    public void release(){
        if(!armIsIn) {
            clawF.setPosition(0.42);
            clawB.setPosition(0.69);
        }
        else{
            clawF.setPosition(.42);
            clawB.setPosition(.21);
        }
    }
    public void updateOrientation() {
        double angle = getHeading();
        if (angle >= -45 && angle <= 45) {//0 degrees passed

            forwardIncreasesX = true;
            forwardDecreasesX = false;
            forwardIncreasesY = false;
            forwardDecreasesY = false;
        } else if (angle > 45 && angle < 135) {//90 degrees passed

            forwardIncreasesX = false;
            forwardDecreasesX = false;
            forwardIncreasesY = true;
            forwardDecreasesY = false;

        } else if (angle >= 135 || angle <= -135) { //180 degrees passed

            forwardIncreasesX = false;
            forwardDecreasesX = true;
            forwardIncreasesY = false;
            forwardDecreasesY = false;

        } else if (angle > -135 && angle < -45) {//270 degrees passed

            forwardIncreasesX = false;
            forwardDecreasesX = false;
            forwardIncreasesY = false;
            forwardDecreasesY = true;

        }
    }

    public void locate(boolean blue, boolean sample, boolean parkWall) {
        this.parkWall = parkWall;
        if (sample) {
            initVuforia();
            if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
                initTfod();
            }
            if (tfod != null) {
                tfod.activate();
            }
        }
        if (blue) {//on blue side
            buildZone.setPoint(56, FieldSide - 20);
            this.blue = true;
            this.red = false;

            if (sample) {
                constant = getHeading();
                this.sample = true;
                this.foundation = false;
                robot.setPoint(robotLength / 2, 34.5 + robotWidth / 2);
            } else {
                constant = getHeading();
                this.sample = false;
                this.foundation = true;
                robot.setPoint(robotLength / 2, FieldSide - 24);

            }
        } else {//on red side
            buildZone.setPoint(56, 20);
            this.blue = false;
            this.red = true;
            if (sample) {
                this.sample = true;
                this.foundation = false;
                robot.setPoint(robotLength / 2, FieldSide - 23 - robotWidth / 2);
            } else {
                this.sample = false;
                this.foundation = true;
                robot.setPoint(FieldSide - robotLength / 2, FieldSide - 24);
                forwardDecreasesX = true;
            }

        }
        if (parkWall) {
            bridge.setPoint(robotWidth/2 + 5, FieldSide / 2);
        } else {
            bridge.setPoint(34, FieldSide / 2);
        }
        for (int i = 0; i < sampleSpots.length; i++) {
            if (blue) sampleSpots[i] = new Coordinate(56, 8 * i + 4);
            else sampleSpots[i] = new Coordinate(56, FieldSide - 4 - 8 * i);
        }
        if (blue) foundationLocation.setPoint(bridge.getX(), FieldSide - 18);
        else foundationLocation.setPoint(bridge.getX(), 12);
        updateOrientation();
    }

    public void orient(double power, double angle) {
        double closest = angle - getHeading();
        rotate(power, closest);
    }

    public void autoCorrect() {
        if (forwardIncreasesY) {
            orient(0.3, 90);
        } else if (forwardDecreasesY) {
            orient(0.3, -90);
        } else if (forwardIncreasesX) {
            orient(0.3, 0);
        } else {
            orient(0.3, 180);
        }
    }
    public double homingBeacon(){
        if (forwardIncreasesY) {
            return 90;
        } else if (forwardDecreasesY) {
            return -90;
        } else if (forwardIncreasesX) {
            return 0;
        } else {
            if(blue) return -180;
            else return 180;
        }
    }
    public void moveStonesBack(boolean horizontal, int index){
        int init = index;
        int moves = 2;
        if(horizontal) {
            if (index + 2 > 5) {
                moves = 5 - index;
            }
            for (int i = 0; i < moves; i++) {
                index++;
                if (sampleSpots[index] != null) {
                    sampleSpots[index].setX(sampleSpots[init].getX() + robotWidth / 2);
                }
            }
        }
        else{
            if(index + 1 <= 5 && sampleSpots[index + 1] != null)  {
                sampleSpots[index + 1].setX(sampleSpots[init].getX() + robotWidth / 2);
            }
            if(index - 1 >= 0 && sampleSpots[index - 1] != null){
                sampleSpots[index - 1].setX(sampleSpots[init].getX() + robotWidth / 2);
            }
        }
    }

    public boolean skyFound = false;

    public void sample(boolean Specialstone, boolean foundationPull, boolean dumpClose, int cycles) {
        Coordinate dumpLocation = new Coordinate(bridge.getX(), FieldSide - 18);
        double homeAngle = 90;
        if (blue) homeAngle = -90;
        Coordinate[] strafeStones = new Coordinate[sampleSpots.length];
        for (int i = 0; i < strafeStones.length; i++) {
            strafeStones[i] = new Coordinate(56, sampleSpots[i].getY());
            if (blue) strafeStones[i].addY(18 + robotLength/2);
            else strafeStones[i].addY(-16 - robotLength/2);
        }
        if (dumpClose) {
            dumpLocation.setY(FieldSide - 34);
        }
        if (Specialstone) {
            if (!skyFound) {
                if (tfod != null) {
                    tfod.activate();
                }
                if (tfod != null) {
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        for (Recognition recognition : updatedRecognitions) {
                            if ((recognition.getLabel()).equals(LABEL_SECOND_ELEMENT)) { //if we find the skystone that is within our height restraints(to avoid background noise)
                                leftCoord = recognition.getImageHeight();
                                if (blue && (recognition.getLeft() > 250) ||  red && recognition.getRight() < 250) {
                                    //if far right based on the left boundary coordinate of image
                                    stoneSpot = 3;
                                    skyFound = true;
                                } else if (blue && (recognition.getLeft() >= 100 && recognition.getLeft() <= 250) || red && recognition.getRight() >= 250 && recognition.getRight() < 450) {
                                    stoneSpot = 4;
                                    //if middle
                                    skyFound = true;
                                } else if (blue || red && recognition.getRight() >= 450) {
                                    //if left
                                    stoneSpot = 5;
                                    skyFound = true;
                                }
                            }
                        }
                        if (!skyFound) {
                            if(blue) {
                                stoneSpot = 5;
                            }
                            else{
                                stoneSpot = 3;
                            }
                            skyFound = true;
                        }
                        //tfod.deactivate();//turn off tensorflow object detection to conserve system resources
                        initialDown();
                        //sleepReq = true;
                        forward(1, 12);
                        armIn();
                        release();
                        intake();
                        goTo(sampleSpots[stoneSpot] , 1, false, false, false);
                        backward(1, bridge.getX() - robot.getX());
                        intakeOff();
                        grip();
                        orient(1, homeAngle);
                        sampleSpots[stoneSpot] = null;
                        strafeStones[stoneSpot] = null;
                    }
                }
            }
        }
        if (skyFound) {

            if (foundationPull) {
                //goTo(foundationLocation, 0.6, false, false, false);
                backward(1, foundationLocation.getY() - robot.getY());
                if (blue) rotate(1, -getProxy(-180));
                else rotate(1, getProxy(180));
                //pullFoundation(true);
                //start
                pullMid();
                armMid();
                foundReq = true;
                backward(0.7, buildZone.getX() - robot.getX() - robotLength/2);
                //pullDown();
                //sleep(200);
                armOut();
                //sleepReq = true;
                if(blue) strafeLeft(1, 12);
                else strafeRight(1, 12);
                armMid();
                forward(1, 14);
                release();
                armIn();

                pidRotate.setPID(0.003, 0.00003, 0.00045);
                if(blue)rotate(0.7, getProxy(homeAngle));
                else rotate(0.7, -getProxy(homeAngle));
                //rotate(0.7, getProxy(homeAngle));
                armIn();
                pullUp();
                //orient(0.7, homeAngle);
                //stop
                //deposit();
                dumpLocation.setPoint(bridge.getX(), robot.getY());
                //forward(0.5, bridge.getY() - robot.getY());
                //goTo(bridge, 1, true, false, false);
                if(cycles == 1){
                    backward(1, FieldSide - 18 - robot.getY());
                }
                strafe(1, bridge.getX() - robot.getX());

                //forward(1, bridge.getY() - robot.getY());
            } else {
                pullReq = true;
                goTo(dumpLocation, 1, true, false, false);
                deposit();
                //forward(0.8, bridge.getY() - robot.getY());
            }
            cycles--;
        }
        if (cycles >= 2 || Specialstone) {
            if (Specialstone) {
                if(!parkWall) {
                    goTo(strafeStones[stoneSpot - 3], 1, false, false, false);
                    moveStonesBack(true, stoneSpot - 3);
                    intake();
                    forward(1, 10);
                    strafe(1, dumpLocation.getX() - robot.getX());
                    intakeOff();
                    grip();
                    pullReq = true;
                    backward(1, dumpLocation.getY() - robot.getY());
                    deposit();
                }
                else {
                    goTo(sampleSpots[stoneSpot - 3], 1, false, false, true);
                    backward(1, getNiche());
                    intakeOff();
                    grip();
                    orient(1, homeAngle);
                    pullReq = true;
                    goTo(dumpLocation, 1, true, false, false);
                }
                sampleSpots[stoneSpot - 3] = null;
                strafeStones[stoneSpot - 3] = null;
                cycles--;
            }
            int p = 0;
            while(p < cycles){
                if (sampleSpots[p] != null) {
                    if(p > 2 || parkWall) {
                        forward(0.8, robot.getY() - bridge.getY());
                        intake();
                        goTo(sampleSpots[p], 0.8, false, false, true);
                        backward(0.8, getNiche());
                        intakeOff();
                        grip();
                        orient(0.7, homeAngle);
                    }
                    else{
                        goTo(strafeStones[p], 0.7, false, false, false);
                        moveStonesBack(true, p);
                        intake();
                        forward(0.8, 12);
                        strafe(1, dumpLocation.getX() - robot.getX());
                        intakeOff();
                        grip();
                    }
                    pullReq = true;
                    if(p == cycles - 1 && foundationPull) sleepReq = true;
                    backward(0.8, dumpLocation.getY() - robot.getY());
                    if (p == cycles - 1 && foundationPull) {
                        armOut();
                        backward(1, FieldSide - 18 - robot.getY());
                        release();
                        sleep(200);
                        armIn();
                    }
                    else {
                        deposit();
                    }
                    sampleSpots[p] = null;
                    strafeStones[p] = null;
                    p++;
                }
            }
        }
    }
    public void park(){
        goTo(bridge, 1, true, false, false);
    }

    public final void idle() {
        Thread.yield();
    }

    public void goTo(Coordinate point, double power, boolean xFirst, boolean combo, boolean asap) {
        if (asap) {
            double distance = robot.distanceTo(point);
            turnTo(0.6, point, true);
            drive(power, distance);
        } else {
            if (forwardIncreasesX) {
                if (xFirst) {
                    drive(power, point.getX() - robot.getX());

                    if (combo) combination(power, point.getY() - robot.getY());
                    else strafe(1, point.getY() - robot.getY());
                } else {
                    if (combo) combination(power, point.getY() - robot.getY());
                    else strafe(1, point.getY() - robot.getY());

                    drive(power, point.getX() - robot.getX());
                }
            } else if (forwardDecreasesX) {
                if (xFirst) {
                    drive(power, robot.getX() - point.getX());

                    if (combo) combination(power, robot.getY() - point.getY());
                    else strafe(1, robot.getY() - point.getY());

                } else {
                    if (combo) combination(power, point.getY() - robot.getY());
                    else strafe(1, point.getY() - robot.getY());

                    drive(power, point.getX() - robot.getX());
                }
            } else if (forwardIncreasesY) {
                if (xFirst) {
                    if (combo) combination(power, robot.getX() - point.getX());
                    else strafe(1, robot.getX() - point.getX());

                    drive(power, point.getY() - robot.getY());
                } else {
                    drive(power, point.getY() - robot.getY());

                    if (combo) combination(power, point.getX() - robot.getX());
                    else strafe(1, point.getX() - robot.getX());
                }
            } else {
                if (xFirst) {
                    if (combo) combination(power, point.getX() - robot.getX());
                    else strafe(1, point.getX() - robot.getX());

                    drive(power, robot.getY() - point.getY());
                } else {
                    drive(power, robot.getY() - point.getY());

                    if (combo) combination(power, point.getX() - robot.getX());
                    else strafe(1, point.getX() - robot.getX());

                }
            }
        }
    }

    public double xCovered(double angle, double distance) {
        return Math.cos(Math.toRadians(angle)) * distance;
    }

    public double yCovered(double angle, double distance) {
        return Math.sin(Math.toRadians(angle)) * distance;
    }

    public void combination(double power, double distance) {
        //this is TEMPORARY code to replace strafing
        turn(1, Range.clip(distance, -1, 1) * -90);
        drive(power, distance);
        turn(1, Range.clip(distance, -1, 1) * 90);
    }


    public void strafe(double power, double distance) {
        if (distance > 0) {
            strafeLeft(power, distance);
        } else {
            strafeRight(power, distance);
        }
    }
    public static final float HEADING_CORRECTION_COEFFICIENT = 0.20f;//.25f
    public void gyroStrafeLeft(double power, double distance){
        setRunMode(false);
        distance = Math.abs(distance);
        int target = strafeToTicks(distance);
        float desiredHeadingRadians = (float)Math.toRadians(homingBeacon() - getHeading());
        double angle = getHeading() + 90;
        if (angle > 180){
            angle -= 360;
        }
        robot.add(xCovered(angle, distance), yCovered(angle, distance));
//        pidStrafe.reset();
//        pidStrafe.setSetpoint(0);
//        pidStrafe.setOutputRange(0, power);
//
//        pidStrafe.setInputRange(-90, 90);
//        pidStrafe.enable();
        //correction = pidDrive.performPID(getAngle());
        float currentHeadingRadians = (float)Math.toRadians(getAngle());
        float headingError = currentHeadingRadians - desiredHeadingRadians;
        if (headingError > Math.PI) headingError -= (float)Math.PI;
        else if (headingError < -Math.PI) headingError += (float)Math.PI;
        correction = -HEADING_CORRECTION_COEFFICIENT * headingError;
        topLeft.setTargetPosition(target);
        topRight.setTargetPosition(-target);
        botLeft.setTargetPosition(-target);
        botRight.setTargetPosition(target);
        setRunMode(true);
        topLeft.setPower(power + correction);
        topRight.setPower(-power - correction);
        botLeft.setPower(-power - correction);
        botRight.setPower(power + correction);
        while (motorsAreBusy()) {
//            correction = pidDrive.performPID(getAngle());

            currentHeadingRadians = (float)Math.toRadians(getAngle());
            headingError = currentHeadingRadians - desiredHeadingRadians;
            if (headingError > Math.PI) headingError -= (float)Math.PI;
            else if (headingError < -Math.PI) headingError += (float)Math.PI;
            correction = -HEADING_CORRECTION_COEFFICIENT * headingError;
            topLeft.setPower(power + correction);
            topRight.setPower(-power - correction);
            botLeft.setPower(-power - correction);
            botRight.setPower(power + correction);
        }
        //don't execute further code until after the wheels are done turning

        //updateCoordinates();
        resetTicks();
        if(!sleepReq) {
            sleep(tempSleep);
        }
        sleepReq = false;
        pidStrafe.disable();
    }


    public void strafeLeft(double power, double distance) {
        setRunMode(false);
        distance = Math.abs(distance);
        int target = strafeToTicks(distance);
        double angle = homingBeacon() + 90;
        if (angle > 180){
            angle -= 360;
        }
        robot.add(xCovered(angle, distance), yCovered(angle, distance));
        pidStrafe.reset();
        double initial = homingBeacon() - getHeading();
        if(initial > 180){
            initial -= 360;
        }
        else if(initial < -180){
            initial += 360;
        }
        pidStrafe.setSetpoint(initial);
        pidStrafe.setOutputRange(0, power);

        pidStrafe.setInputRange(-90, 90);
        pidStrafe.enable();
        correction = pidDrive.performPID(getAngle());
        topLeft.setTargetPosition(target);
        topRight.setTargetPosition(-target);
        botLeft.setTargetPosition(-target);
        botRight.setTargetPosition(target);
        setRunMode(true);
        topLeft.setPower(power + correction);
        topRight.setPower(-power - correction);
        botLeft.setPower(-power - correction);
        botRight.setPower(power + correction);
        while (motorsAreBusy()) {
            correction = pidDrive.performPID(getAngle());
            topLeft.setPower(power + correction);
            topRight.setPower(-power - correction);
            botLeft.setPower(-power - correction);
            botRight.setPower(power + correction);
        }
        //don't execute further code until after the wheels are done turning

        //updateCoordinates();
        resetTicks();
        if(!sleepReq) {
            sleep(tempSleep);
        }
        sleepReq = false;
        pidStrafe.disable();
    }

    public void strafeRight(double power, double distance) {
        setRunMode(false);
        distance = Math.abs(distance);
        double angle = homingBeacon() - 90;
        if (angle < -180){
            angle += 360;
        }
        robot.add(xCovered(angle, distance), yCovered(angle, distance));
        pidStrafe.reset();
        double initial = homingBeacon() - getHeading();
        if(initial > 180){
            initial -= 360;
        }
        else if(initial < -180){
            initial += 360;
        }
        pidStrafe.setSetpoint(initial);
        pidStrafe.setOutputRange(0, power);
        pidStrafe.setInputRange(-90, 90);
        pidStrafe.enable();
        correction = pidDrive.performPID(getAngle());
        int target = strafeToTicks(distance);
        topLeft.setTargetPosition(-target);
        topRight.setTargetPosition(target);
        botLeft.setTargetPosition(target);
        botRight.setTargetPosition(-target);
        setRunMode(true);
        topLeft.setPower(-power + correction);
        topRight.setPower(power - correction);
        botLeft.setPower(power - correction);
        botRight.setPower(-power + correction);
        while (motorsAreBusy()) {
            correction = pidDrive.performPID(getAngle());
            topLeft.setPower(-power +correction);
            topRight.setPower(power - correction);
            botLeft.setPower(power - correction);
            botRight.setPower(-power + correction);
        }
        //don't execute further code until after the wheels are done turning

        //updateCoordinates();
        resetTicks();
        if(!sleepReq) {
            sleep(tempSleep);
        }
        sleepReq = false;

    }

    public void timeStrafeLeft(double power, double time) {
        setRunMode(false);
        //double inital = referenceRange();
        topLeft.setPower(-power);
        topRight.setPower(power);
        botLeft.setPower(power);
        botRight.setPower(-power);
        sleep((long) (time * 1000));
        /*while (Math.abs(referenceRange() - inital) < distance) {

        }
        //don't execute further code until after the wheels are done turning
         */
        //updateCoordinates();
        resetTicks();
    }


    public void timeStrafeRight(double power, double time) {
        //double inital = referenceRange();
        setRunMode(false);
        topLeft.setPower(power);
        topRight.setPower(-power);
        botLeft.setPower(-power);
        botRight.setPower(power);
        sleep((long) (time * 1000));//don't execute further code until after the wheels are done turning
        //updateCoordinates();
        resetTicks();
    }

    public final void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    private String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    private String formatDegrees(double degrees) {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    private void resetAngle()//for PID
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        globalAngle = 0;
    }

    public double getAngle()//for PID
    {
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

    public double getHeading() {
        orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double val = Double.parseDouble(formatAngle(orientation.angleUnit, orientation.firstAngle)) - constant;
        if (val > 180) {
            val -= 360;
        } else if (val < -180) {
            val += 360;
        }
        return val;
    }

    public void turn(double power, double degrees) {
        if (degrees < 0) {
            turnRight(power, Math.abs(degrees));
        } else {
            turnLeft(power, degrees);
        }

    }

    public void rotate(double power, double degrees) {
        // restart imu angle tracking.
        setRunMode(false);
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

        if (degrees < 0)
        {
            // On right turn we have to get off zero first.
            while (getAngle() == 0)
            {
                topLeft.setPower(-power);
                botLeft.setPower(-power);
                topRight.setPower(power);
                botRight.setPower(power);
                sleep(100);
            }

            do
            {
                power = pidRotate.performPID(getAngle()); // power will be - on right turn.
                topLeft.setPower(power);
                botLeft.setPower(power);
                topRight.setPower(-power);
                botRight.setPower(-power);
            } while (!pidRotate.onTarget());
        }
        else    // left turn.
            do
            {
                power = pidRotate.performPID(getAngle()); // power will be + on left turn.
                topLeft.setPower(power);
                botLeft.setPower(power);
                topRight.setPower(-power);
                botRight.setPower(-power);
            } while (!pidRotate.onTarget());

        // turn the motors off.
        resetTicks();

        rotation = getAngle();

        // wait for rotation to stop.
        if(!sleepReq) {
            sleep(tempSleep);
        }
        sleepReq = false;
        // reset angle tracking on new heading.
        resetAngle();
        updateOrientation();
    }

    public void turnLeft(double power, double angle) {
        double target = getHeading() + angle;
        if (target < -180) {
            target += 360;
        } else if (target > 180) {
            target -= 360;
        }
        double prev = getProxy(target);
        setRunMode(false);
        topLeft.setPower(-power);
        topRight.setPower(power);
        botLeft.setPower(-power);
        botRight.setPower(power);
        while (getProxy(target) <= prev + 0.3) {
            prev = getProxy(target);
            if (getProxy(target) < 20) {
                power = 0.13;
                topLeft.setPower(-power);
                topRight.setPower(power);
                botLeft.setPower(-power);
                botRight.setPower(power);
            }
        }//don't execute further code until after the wheels are done turning
        resetTicks();//stop and reset tick count on wheels
        //setRunMode(false);
        updateOrientation();
        sleep(300);
    }

    public void turnRight(double power, double angle) {
        double target = getHeading() - angle;
        if (target < -180) {
            target += 360;
        } else if (target > 180) {
            target -= 360;
        }
        double prev = getProxy(target);
        setRunMode(false);
        topLeft.setPower(power);
        topRight.setPower(-power);
        botLeft.setPower(power);
        botRight.setPower(-power);
        while (getProxy(target) <= prev + 0.3) {
            prev = getProxy(target);
            if (getProxy(target) <= 20) {
                power = 0.13;
                topLeft.setPower(power);
                topRight.setPower(-power);
                botLeft.setPower(power + 0.05);
                botRight.setPower(-power - 0.05);
            }
        }//don't execute further code until after the wheels are done turning
        resetTicks();//stop and reset tick count on wheels
        sleep(300);
    }

    public void forward(double power, double distance) {
        resetAngle();
        distance = Math.abs(distance);
        double angle = homingBeacon();//subject to change
        robot.add(xCovered(angle, distance), yCovered(angle, distance));
        pidDrive.reset();
        double initial = homingBeacon() - getHeading();
        if(initial > 180){
            initial -= 360;
        }
        else if(initial < -180){
            initial += 360;
        }
        pidDrive.setSetpoint(initial);
        pidDrive.setOutputRange(0, power);
        pidDrive.setInputRange(-90, 90);
        pidDrive.enable();
        correction = pidDrive.performPID(getAngle());

        setRunMode(false);
        resetTicks();//reset encoder at the very beginning in case we didn't already do so

        topLeft.setTargetPosition(inchesToTicks(distance));//inches to ticks conversion
        topRight.setTargetPosition(inchesToTicks(distance));
        botLeft.setTargetPosition(inchesToTicks(distance));
        botRight.setTargetPosition(inchesToTicks(distance));

        setRunMode(true);//set the motors to RUN_TO_POSITION mode

        topLeft.setPower(power + correction);
        botLeft.setPower(power + correction);
        topRight.setPower(power - correction);
        botRight.setPower(power - correction);
        while (motorsAreBusy()) {

            correction = pidDrive.performPID(getAngle());
            topLeft.setPower(power + correction);
            botLeft.setPower(power + correction);
            topRight.setPower(power - correction);
            botRight.setPower(power - correction);

        }//don't execute further code until after the wheels are done turning


        resetTicks();//stop and reset tick count on wheels
        if(!sleepReq) {
            sleep(tempSleep);
        }
        sleepReq = false;
        pidDrive.disable();
    }
    public boolean foundReq = false;

    public void backward(double power, double distance) {
        resetAngle();
        distance = Math.abs(distance);
        double angle = homingBeacon();//subject to change
        double initial = homingBeacon() - getHeading();
        if(initial > 180){
            initial -= 360;
        }
        else if(initial < -180){
            initial += 360;
        }
        setRunMode(false);
        pidDrive.reset();
        pidDrive.setSetpoint(initial);
        pidDrive.setOutputRange(0, power);
        pidDrive.setInputRange(-90, 90);
        pidDrive.enable();
        correction = pidDrive.performPID(getAngle());
        resetTicks();
        robot.add(xCovered(angle, -distance), yCovered(angle, -distance));
        topLeft.setTargetPosition(-inchesToTicks(distance));//inches to ticks conversion
        topRight.setTargetPosition(-inchesToTicks(distance));
        botLeft.setTargetPosition(-inchesToTicks(distance));
        botRight.setTargetPosition(-inchesToTicks(distance));

        setRunMode(true);

        topLeft.setPower(-power + correction);
        botLeft.setPower(-power + correction);
        topRight.setPower(-power - correction);
        botRight.setPower(-power - correction);
        while (motorsAreBusy()) {
            correction = pidDrive.performPID(getAngle());
            topLeft.setPower(-power + correction);
            botLeft.setPower(-power + correction);
            topRight.setPower(-power - correction);
            botRight.setPower(-power - correction);
            if(pullReq && topLeft.getCurrentPosition() < 5 * topLeft.getTargetPosition()/6){
                armMid();
                pullReq = false;
            }
            if(foundReq && topLeft.getCurrentPosition() < 5* topLeft.getTargetPosition()/6){
                pullDown();
                foundReq = false;
            }
        }//don't execute further code until after the wheels are done turning
        resetTicks();//stop and reset tick count on wheels
        if(!sleepReq) {
            sleep(tempSleep);
        }
        sleepReq = false;
        pidDrive.disable();
    }

    public void drive(double power, double distance) {
        if (distance > 0) {
            forward(power, distance);
        } else if (distance < 0) {
            backward(power, Math.abs(distance));
        }
    }

    public void turnTo(double power, Coordinate desired, boolean facing) {
        double x = desired.getX() - robot.getX();
        double y = desired.getY() - robot.getY();

        double angle = Math.toDegrees(Math.atan2(y, x));
        if (!facing) {
            if (Math.abs(getHeading() - angle) > Math.abs(getHeading() - angle + 180)) {
                angle += 180;
            }
        }
        orient(power, angle);
    }

    public void timeForward(double power, double time) {
        setRunMode(false);
        topLeft.setPower(power);
        topRight.setPower(power);
        botLeft.setPower(power);
        botRight.setPower(power);
        sleep((long) (time * 1000));
        resetTicks();
    }

    public void timeBackward(double power, double time) {
        setRunMode(false);
        topLeft.setPower(-power);
        topRight.setPower(-power);
        botLeft.setPower(-power);
        botRight.setPower(-power);
        sleep((long) (time * 1000));
        resetTicks();
    }

    public void pullFoundation(boolean horizontal) {
        if (horizontal) {
            if (blue) {
                pullUp();
                backward(0.5, buildZone.getX() - robot.getX() - robotLength / 2);
                pullDown();
                sleep(400);
                forward(0.5, 30);
                turnLeft(0.5, 90);
                timeBackward(0.4, 0.4);
                robot.setY(FieldSide - 18.5);
                pullUp();
            } else {
                pullUp();
                backward(0.5, buildZone.getX() - robot.getX() - robotLength / 2);
                pullDown();
                sleep(400);
                forward(0.5, 30);
                turnRight(0.5, 90);
                timeBackward(0.4, 0.4);
                robot.setY(FieldSide - 18.5);
                pullUp();
            }
        } else {
            if (blue) {
                timeStrafeLeft(0.5, .2);
                pullUp();
                timeBackward(0.5, 1);
                pullDown();
                sleep(400);
                timeForward(0.7, 1);
                timeStrafeRight(1, 0.7);
            } else {
                timeStrafeLeft(0.5, .5);
                pullUp();
                backward(0.5, buildZone.getX() - robot.getX() - robotLength / 2);
                pullDown();
                sleep(400);
                timeForward(0.7, 1);
                timeStrafeRight(1, 0.7);
            }
        }
    }
    private void initVuforia() {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = map.get(WebcamName.class, "Webcam 1");
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }
    private void initTfod() {
        int tfodMonitorViewId = map.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", map.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.8;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }
}