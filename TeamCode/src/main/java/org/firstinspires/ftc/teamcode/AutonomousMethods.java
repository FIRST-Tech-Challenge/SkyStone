package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
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
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.ClassFactory;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.Odometry.OdometryGlobalCoordinatePosition;

public class AutonomousMethods {
    BNO055IMU imu;
    Orientation orientation = new Orientation();
    DcMotor.RunMode newRun;
    HardwareMap map;

    Orientation lastAngles = new Orientation();
    double globalAngle, power = .30, correction, rotation;
    PIDController pidRotate, pidDrive, pidStrafe, pidCurve;
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
    //private static final float mmTargetHeight = (6) * mmPerInch;          // the height of the center of the target image above the floor

    private double constant;
    private int tempSleep = 0;
    private Robot robot;
    public AutonomousMethods(DcMotor.RunMode runMode, HardwareMap importedMap, side side) {
        newRun = runMode;
        map = importedMap;
        this.side = side;
        robot = new Robot(runMode, map,0, 0,0, 0);
    }
    enum side{
        blue,
        red
    }
    //all variable related to the dimensions of the robot or field
    private final double FieldSide = 141; //11.75 ft or 141 inches
    public final double ticksPerRev = 560;//dependent on the motor we use
    public final double wheelDiameter = 4.65; //in inches
    public final double robotLength = 17.5;
    public final double robotWidth = 17.5;

    //alliance color

    public side side;
    //side of the field
    public boolean foundation = false;
    public boolean sample = false;
    public boolean parkWall = false;

    public boolean armReq = false;
    public boolean sleepReq = false;


    //coordinate system related variables


    public int inchesToTicks(double inches) {
        return (int) ((inches / (Math.PI * wheelDiameter)) * ticksPerRev);//dividing the total revolutions completed by the motor(found by diving the desired inches by the wheel circumference), by the motor's encoder count for 1 rev.

    }

    public boolean motorsAreBusy() {
        return robot.topLeft.isBusy() && robot.topRight.isBusy() && robot.botLeft.isBusy() && robot.botRight.isBusy();
    }
    private int TLMin;
    public int TL(){
        return robot.topLeft.getCurrentPosition() - TLMin;
    }
    private int TRMin;
    public int TR(){
        return robot.topRight.getCurrentPosition() - TRMin;
    }
    private int BLMin;
    public int BL(){
        return  robot.botLeft.getCurrentPosition() - BLMin;
    }
    private int BRMin;
    public int BR(){
        return robot.botRight.getCurrentPosition() - BRMin;
    }
    public int avgTicks() {
        //return Math.abs(robot.topLeft.getCurrentPosition() + robot.topRight.getCurrentPosition() + robot.botLeft.getCurrentPosition() + robot.botRight.getCurrentPosition())/4;
        return Math.abs(TL() + TR() + BL() + BR())/4;
    }
    public int strafeTicks(){
        return Math.max(Math.max(Math.abs(TL()), Math.abs(TR())), Math.max(Math.abs(BL()), Math.abs(BR())));
    }
    public double getProxy(double angle) {
        double curr = robot.getHeading();
        double dist1 = Math.abs(curr - angle);
        if (Math.abs(dist1) > 180) {
            return Math.abs(360 - dist1);
        } else {
            return dist1;
        }
    }
    public int armRound = 0;

    public void armOut() {
        if (armRound == 0){
            armL.setPosition(.95);
            armR.setPosition(.05);
        } else if (armRound == 1) {
            armL.setPosition(.8);
            armR.setPosition(.2);
        } else {
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

    public void armMid() {
        armL.setPosition(.5);
        armR.setPosition(.5);

    }

    public void deposit() {
        armOut();
        sleep(700);
        release();
        sleep(100);
        armMid();
        sleep(200);
        armIn();
        sleep(100);
    }

    private int strafeToTicks(double inches) {
        return (int) (55 * inches);//55
    }
    private double ticksToStrafe(int ticks){
        return ticks/55;
    }

    public double ticksToInches(int ticks) {
        return ticks * (wheelDiameter * Math.PI / ticksPerRev);
    }

    public double getNiche() {
        double y = bridge.getY() - robot.pt.getY();
        double angle = robot.getHeading() + 180;
        return y / Math.abs(Math.cos(Math.toRadians(angle)));
    }

    public void resetTicks() {
//        robot.topRight.setMode(DcMotor.RunMode.RESET_ENCODERS);
//        robot.topLeft.setMode(DcMotor.RunMode.RESET_ENCODERS);
//        robot.botRight.setMode(DcMotor.RunMode.RESET_ENCODERS);
//        robot.botLeft.setMode(DcMotor.RunMode.RESET_ENCODERS);
        robot.topLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.topRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.botLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.botRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//        robot.topRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.topLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.botRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.botLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.topRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.topLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.botRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.botLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        TLMin = robot.topLeft.getCurrentPosition();
        TRMin = robot.topRight.getCurrentPosition();
        BLMin = robot.botLeft.getCurrentPosition();
        BRMin = robot.botRight.getCurrentPosition();
    }
    public void stopAndResetTicks(){
//        robot.topLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        robot.topRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        robot.botLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        robot.botRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.topRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.topLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.botRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.botLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        TLMin = robot.topLeft.getCurrentPosition();
        TRMin = robot.topRight.getCurrentPosition();
        BLMin = robot.botLeft.getCurrentPosition();
        BRMin = robot.botRight.getCurrentPosition();
    }

    public void setRunMode(boolean runToPosition) {
        if (runToPosition) {
            robot.topLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.topRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.botLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.botRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        } else {
            robot.topLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.topRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.botLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.botRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void grip() {
        clawF.setPosition(0.8);
        clawB.setPosition(0.21);
    }

    public void release() {
        if (!armIsIn) {
            clawF.setPosition(0.4);
            clawB.setPosition(0.69);
        } else {
            clawF.setPosition(.4);
            clawB.setPosition(.21);
        }
    }
    public void updateOrientation() {
        double angle = robot.getHeading();
//        if (angle >= -45 && angle <= 45) {//0 degrees passed
//            setForwardIncreasesX();
//        } else if (angle > 45 && angle < 135) {//90 degrees passed
//            setForwardIncreasesY();
//        } else if (angle >= 135 || angle <= -135) { //180 degrees passed
//            setForwardDecreasesX();
//        } else if (angle > -135 && angle < -45) {//270 degrees passed
//            setForwardDecreasesY();
//        }
    }
    public void orient(double power, double angle) {
        double closest = angle - robot.getHeading();
        rotate(power, closest);
    }


    public double homingBeacon() {
        double angle = robot.getHeading();
        if (angle > -45 && angle < 45 ) {
            return 0;
        } else if (angle >= 45 && angle <= 135) {
            return 90;
        } else if (angle > 135 || angle < -135) {
            if (blue) return -180;
            else return 180;
        } else {
            return -90;
        }
    }

    public void moveStonesBack(boolean horizontal, int index) {
        int init = index;
        int moves = 2;
        if (horizontal) {
            if (index + 2 > 5) {
                moves = 5 - index;
            }
            for (int i = 0; i < moves; i++) {
                index++;
                if (sampleSpots[index] != null) {
                    sampleSpots[index].setX(sampleSpots[init].getX() + robotWidth / 2);
                }
            }
        } else {
            if (index + 1 <= 5 && sampleSpots[index + 1] != null) {
                sampleSpots[index + 1].setX(sampleSpots[init].getX() + robotWidth / 2);
            }
            if (index - 1 >= 0 && sampleSpots[index - 1] != null) {
                sampleSpots[index - 1].setX(sampleSpots[init].getX() + robotWidth / 2);
            }
        }
    }
    public final void idle() {
        Thread.yield();
    }
    public void goTo(Coordinate point, double power, boolean driveFirst, boolean asap) {
        if (asap) {
            double distance = robot.pt.distanceTo(point);
            sleepReq = true;
            turnTo(1, point, true);
            offTrack = true;
            drive(power, distance);
        } else {
            if(driveFirst) {
                drive(power, robot.pt.getMatchX(point, robot.getHeading()));
                strafe(power, robot.pt.getMatchY(point, robot.getHeading()));
            }
            else{
                strafe(power, robot.pt.getMatchY(point, robot.getHeading()));
                drive(power, robot.pt.getMatchX(point, robot.getHeading()));
            }
        }
    }
    public double xCovered(double angle, double distance) {
        return Math.cos(Math.toRadians(angle)) * distance;
    }

    public double yCovered(double angle, double distance) {
        return Math.sin(Math.toRadians(angle)) * distance;
    }
    public void strafe(double power, double distance) {
        if (distance > 0) {
            strafeLeft(power, distance);
        } else {
            strafeRight(power, distance);
        }
    }
    public void strafeLeft(double power, double distance) {
        resetAngle();
        setRunMode(false);
        distance = Math.abs(distance);
        int target = strafeToTicks(distance);
        double angle = robot.getHeading() + 90;
        if (angle > 180) {
            angle -= 360;
        }
        int prev = strafeTicks();
        robot.pt.add(xCovered(angle, distance), yCovered(angle, distance));
        pidStrafe.reset();
        double initial = homingBeacon() - robot.getHeading();
        if (initial > 180) {
            initial -= 360;
        } else if (initial < -180) {
            initial += 360;
        }
        pidStrafe.setSetpoint(initial);
        pidStrafe.setOutputRange(0, power);

        pidStrafe.setInputRange(-90, 90);
        //pidStrafe.enable();
        correction = pidDrive.performPID(robot.getAngle());
        robot.topLeft.setTargetPosition(robot.topLeft.getCurrentPosition() + target);
        robot.topRight.setTargetPosition(robot.topRight.getCurrentPosition() -target);
        robot.botLeft.setTargetPosition(robot.botLeft.getCurrentPosition() -target);
        robot.botRight.setTargetPosition(robot.botRight.getCurrentPosition() + target);
        setRunMode(true);
        robot.topLeft.setPower(power + correction);
        robot.topRight.setPower(-power - correction);
        robot.botLeft.setPower(-power - correction);
        robot.botRight.setPower(power + correction);
        while (motorsAreBusy()) {
            correction = pidDrive.performPID(robot.getAngle());
            robot.topLeft.setPower(power + correction);
            robot.topRight.setPower(-power - correction);
            robot.botLeft.setPower(-power - correction);
            robot.botRight.setPower(power + correction);
            robot.pt.add(xCovered(angle, ticksToStrafe(strafeTicks() - prev)), yCovered(angle, ticksToStrafe(strafeTicks() - prev)));
            prev = strafeTicks();
            angle = robot.getHeading() + 90;
            if (angle > 180) {
                angle -= 360;
            }
            idle();
        }
        if (!sleepReq) {
            stopAndResetTicks();
            sleep(tempSleep);
        }
        else{
            resetTicks();
        }
        sleepReq = false;
        pidStrafe.disable();
    }

    public void strafeRight(double power, double distance) {
        resetAngle();
        setRunMode(false);
        distance = Math.abs(distance);
        int prev = strafeTicks();
        double angle = robot.getHeading() - 90;
        if (angle < -180) {
            angle += 360;
        }
        robot.pt.add(xCovered(angle, distance), yCovered(angle, distance));
        pidStrafe.reset();
        double initial = homingBeacon() - robot.getHeading();
        if (initial > 180) {
            initial -= 360;
        } else if (initial < -180) {
            initial += 360;
        }
        pidStrafe.setSetpoint(initial);
        pidStrafe.setOutputRange(0, power);
        pidStrafe.setInputRange(-90, 90);
        //pidStrafe.enable();
        correction = pidDrive.performPID(robot.getAngle());
        int target = strafeToTicks(distance);
        robot.topLeft.setTargetPosition(robot.topLeft.getCurrentPosition() -target);
        robot.topRight.setTargetPosition(robot.topRight.getCurrentPosition() + target);
        robot.botLeft.setTargetPosition(robot.botLeft.getCurrentPosition() + target);
        robot.botRight.setTargetPosition(robot.botRight.getCurrentPosition() -target);
        setRunMode(true);
        robot.topLeft.setPower(-power + correction);
        robot.topRight.setPower(power - correction);
        robot.botLeft.setPower(power - correction);
        robot.botRight.setPower(-power + correction);
        while (motorsAreBusy()) {
            correction = pidDrive.performPID(robot.getAngle());
            robot.topLeft.setPower(-power + correction);
            robot.topRight.setPower(power - correction);
            robot.botLeft.setPower(power - correction);
            robot.botRight.setPower(-power + correction);
            robot.pt.add(xCovered(angle, ticksToStrafe(strafeTicks() - prev)), yCovered(angle, ticksToStrafe(strafeTicks() - prev)));
            prev = strafeTicks();
            angle = robot.getHeading() - 90;
            if (angle < -180) {
                angle += 360;
            }
            idle();
            sleep(100);

        }
        //don't execute further code until after the wheels are done turning

        //updateCoordinates();

        if (!sleepReq) {
            stopAndResetTicks();
            sleep(tempSleep);
        }
        resetTicks();
        sleepReq = false;

    }

    public void timeStrafeLeft(double power, double time) {
        setRunMode(false);
        //double inital = referenceRange();
        robot.topLeft.setPower(-power);
        robot.topRight.setPower(power);
        robot.botLeft.setPower(power);
        robot.botRight.setPower(-power);
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
        robot.topLeft.setPower(power);
        robot.topRight.setPower(-power);
        robot.botLeft.setPower(-power);
        robot.botRight.setPower(power);
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



    public void forward(double power, double distance) {
        updateOrientation();

        resetTicks();
        distance = Math.abs(distance);
        double initial = homingBeacon() - robot.robot.getHeading();//auto correct to straight heading
        if(initial > 180){
            initial -= 360;
        }
        else if(initial < -180){
            initial += 360;
        }
        //robot.pt.add(xCovered(robot.getHeading(), distance), yCovered(robot.getHeading(), distance));
        pidDrive.reset();
        pidDrive.setSetpoint(initial);
        pidDrive.setOutputRange(0, power);
        pidDrive.setInputRange(-90, 90);
        pidDrive.enable();

        correction = pidDrive.performPID(robot.getAngle());
        //setRunMode(false);//set the motors to RUN_USING_ENCODER
        resetTicks();//reset encoder at the very beginning in case we didn't already do so
        robot.topLeft.setTargetPosition(robot.topLeft.getCurrentPosition() + inchesToTicks(distance));//inches to ticks conversion
        robot.topRight.setTargetPosition(robot.topRight.getCurrentPosition() + inchesToTicks(distance));
        robot.botLeft.setTargetPosition(robot.botLeft.getCurrentPosition() + inchesToTicks(distance));
        robot.botRight.setTargetPosition(robot.botRight.getCurrentPosition() + inchesToTicks(distance));
        setRunMode(true);//set the motors to RUN_TO_POSITION mode
        robot.topLeft.setPower(Range.clip(power + correction, -1, 1));
        robot.botLeft.setPower(Range.clip(power + correction, -1, 1));
        robot.topRight.setPower(Range.clip(power - correction, -1, 1));
        robot.botRight.setPower(Range.clip(power - correction, -1, 1));
        while (motorsAreBusy()) {
            updateOrientation();
            pidDrive.setSetpoint(initial);
            correction = pidDrive.performPID(robot.getAngle());//use PID to autocoreect to desired location
            robot.topLeft.setPower(Range.clip(power + correction, -1, 1));
            robot.botLeft.setPower(Range.clip(power + correction, -1, 1));
            robot.topRight.setPower(Range.clip(power - correction, -1, 1));
            robot.botRight.setPower(Range.clip(power - correction, -1, 1));

            idle();
            sleep(200);
        }//don't execute further code until after the wheels are done turning
       //stop and reset tick count on wheels
        intakeReq = false;
        armReq = false;
        //robot.pt.add(xCovered(robot.getHeading(), ticksToInches(avgTicks() - prev)), yCovered(robot.getHeading(), ticksToInches(avgTicks() - prev)));//adding the inches we cover based on our current heading
        if (!sleepReq) {
            stopAndResetTicks();
            sleep(tempSleep);
        }
        else{
            resetTicks();
        }
        sleepReq = false;
        pidDrive.disable();
        updateOrientation();

    }
    public boolean foundReq = false;
    public boolean intakeReq = false;


    public void reset(){
        sleepReq = false;


        foundReq = false;
        armReq = false;
    }
    public void backward(double power, double distance) {
        updateOrientation();
        robot.resetAngle();
        resetTicks();
        distance = Math.abs(distance);
        double initial = homingBeacon() - robot.getHeading();
        int prev = avgTicks();
        if(initial > 180){
            initial -= 360;
        }
        else if(initial < -180){
            initial +=360;
        }
        robot.pt.add(xCovered(homingBeacon(), -distance), yCovered(homingBeacon(), -distance));
        //setRunMode(false);
        pidDrive.reset();
        pidDrive.setSetpoint(initial);
        pidDrive.setOutputRange(0, power);
        pidDrive.setInputRange(-90, 90);
        pidDrive.enable();
        correction = pidDrive.performPID(robot.getAngle());

        robot.topLeft.setTargetPosition(robot.topLeft.getCurrentPosition() -inchesToTicks(distance));//inches to ticks conversion
        robot.topRight.setTargetPosition(robot.topRight.getCurrentPosition() -inchesToTicks(distance));
        robot.botLeft.setTargetPosition(robot.botLeft.getCurrentPosition() -inchesToTicks(distance));
        robot.botRight.setTargetPosition(robot.botRight.getCurrentPosition() -inchesToTicks(distance));

        setRunMode(true);

        robot.topLeft.setPower(Range.clip(-power + correction, -1, 1));
        robot.botLeft.setPower(Range.clip(-power + correction, -1, 1));
        robot.topRight.setPower(Range.clip(-power - correction, -1, 1));
        robot.botRight.setPower(Range.clip(-power - correction, -1, 1));
        while (motorsAreBusy()) {
            updateOrientation();
            pidDrive.setSetpoint(initial);
            correction = pidDrive.performPID(robot.getAngle());
            robot.topLeft.setPower(Range.clip(-power + correction, -1, 1));
            robot.botLeft.setPower(Range.clip(-power + correction, -1, 1));
            robot.topRight.setPower(Range.clip(-power - correction, -1, 1));
            robot.botRight.setPower(Range.clip(-power - correction, -1, 1));
            //robot.pt.add(xCovered(robot.getHeading(), -ticksToInches(avgTicks()) - prev), yCovered(robot.getHeading(), -ticksToInches(avgTicks() - prev)));
            //robot.pt.add(xCovered(robot.getHeading(), -ticksToInches(Math.abs(avgTicks()- prev))), yCovered(robot.getHeading(), -ticksToInches(Math.abs(avgTicks() - prev))));
            idle();
            sleep(100);
        }//don't execute further code until after the wheels are done turning
        //resetTicks();//stop and reset tick count on wheels
        //robot.pt.add(xCovered(robot.getHeading(), -ticksToInches(Math.abs(avgTicks()- prev))), yCovered(robot.getHeading(), -ticksToInches(Math.abs(avgTicks() - prev))));

            stopAndResetTicks();
            sleep(tempSleep);



        pidDrive.disable();
        updateOrientation();

    }

    public void drive(double power, double distance) {
        if (distance > 0) {
            forward(power, distance);
        } else if (distance < 0) {
            backward(power, Math.abs(distance));
        }
    }

    public void turnTo(double power, Coordinate desired, boolean facing) {

        orient(power, robot.pt.angleTo(desired, facing));
    }

    public void timeForward(double power, double time) {
        setRunMode(false);
        robot.topLeft.setPower(power);
        robot.topRight.setPower(power);
        robot.botLeft.setPower(power);
        robot.botRight.setPower(power);
        sleep((long) (time * 1000));
        resetTicks();
    }

    public void timeBackward(double power, double time) {
        setRunMode(false);
        robot.topLeft.setPower(-power);
        robot.topRight.setPower(-power);
        robot.botLeft.setPower(-power);
        robot.botRight.setPower(-power);
        sleep((long) (time * 1000));
        resetTicks();
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