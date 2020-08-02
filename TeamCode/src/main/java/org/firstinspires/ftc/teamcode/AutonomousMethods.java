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
    public int stoneSpot = 5;

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
    public AutonomousMethods(DcMotor.RunMode runMode, HardwareMap importedMap, boolean blue) {
        newRun = runMode;
        map = importedMap;
        this.blue = blue;
        robot = new Robot(runMode, map,0, 0,0, 0);
    }

    //all variable related to the dimensions of the robot or field
    private final double FieldSide = 141; //11.75 ft or 141 inches
    public final double ticksPerRev = 560;//dependent on the motor we use
    public final double wheelDiameter = 4.65; //in inches
    public final double robotLength = 17.5;
    public final double robotWidth = 17.5;

    //alliance color

    public boolean blue;
    //side of the field
    public boolean foundation = false;
    public boolean sample = false;
    public boolean parkWall = false;

    public boolean armReq = false;
    public boolean sleepReq = false;


    //coordinate system related variables
    public Coordinate foundationLocation = new Coordinate(0, 0);//univeral coordinate to move and add stones to foundation
    public Coordinate buildZone = new Coordinate(50, 100);
    public Coordinate[] sampleSpots = new Coordinate[6];//locations for every stone during sampling stage
    //if we want to intake stones
//    public Coordinate skystone = new Coordinate(-1, -1);//spot that a new skystone can be found
    public Coordinate bridge = new Coordinate(0, 0);//for parking
    public CurvePoint[] suckSpots = new CurvePoint[6];
    public CurvePoint initCurve = new CurvePoint();

    public int inchesToTicks(double inches) {
        return (int) ((inches / (Math.PI * wheelDiameter)) * ticksPerRev);//dividing the total revolutions completed by the motor(found by diving the desired inches by the wheel circumference), by the motor's encoder count for 1 rev.

    }

    public boolean motorsAreBusy() {
        return robot.topLeft.isBusy() && topRight.isBusy() && botLeft.isBusy() && botRight.isBusy();
    }
    private int TLMin;
    public int TL(){
        return robot.topLeft.getCurrentPosition() - TLMin;
    }
    private int TRMin;
    public int TR(){
        return topRight.getCurrentPosition() - TRMin;
    }
    private int BLMin;
    public int BL(){
        return  botLeft.getCurrentPosition() - BLMin;
    }
    private int BRMin;
    public int BR(){
        return botRight.getCurrentPosition() - BRMin;
    }
    public int avgTicks() {
        //return Math.abs(robot.topLeft.getCurrentPosition() + topRight.getCurrentPosition() + botLeft.getCurrentPosition() + botRight.getCurrentPosition())/4;
        return Math.abs(TL() + TR() + BL() + BR())/4;
    }
    public int strafeTicks(){
        return Math.max(Math.max(Math.abs(TL()), Math.abs(TR())), Math.max(Math.abs(BL()), Math.abs(BR())));
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
        double angle = getHeading() + 180;
        return y / Math.abs(Math.cos(Math.toRadians(angle)));
    }

    public void resetTicks() {
//        topRight.setMode(DcMotor.RunMode.RESET_ENCODERS);
//        robot.topLeft.setMode(DcMotor.RunMode.RESET_ENCODERS);
//        botRight.setMode(DcMotor.RunMode.RESET_ENCODERS);
//        botLeft.setMode(DcMotor.RunMode.RESET_ENCODERS);
        robot.topLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        topRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        botLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        botRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//        topRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.topLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        botRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        botLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        topRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.topLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        botRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        botLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        TLMin = robot.topLeft.getCurrentPosition();
        TRMin = topRight.getCurrentPosition();
        BLMin = botLeft.getCurrentPosition();
        BRMin = botRight.getCurrentPosition();
    }
    public void stopAndResetTicks(){
//        robot.topLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        topRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        botLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        botRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        topRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.topLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        botRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        botLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        TLMin = robot.topLeft.getCurrentPosition();
        TRMin = topRight.getCurrentPosition();
        BLMin = botLeft.getCurrentPosition();
        BRMin = botRight.getCurrentPosition();
    }

    public void setRunMode(boolean runToPosition) {
        if (runToPosition) {
            robot.topLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            topRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            botLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            botRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        } else {
            robot.topLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            topRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            botLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            botRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
        double angle = getHeading();
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
        double closest = angle - getHeading();
        rotate(power, closest);
    }


    public double homingBeacon() {
        double angle = getHeading();
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
                drive(power, robot.pt.getMatchX(point, getHeading()));
                strafe(power, robot.pt.getMatchY(point, getHeading()));
            }
            else{
                strafe(power, robot.pt.getMatchY(point, getHeading()));
                drive(power, robot.pt.getMatchX(point, getHeading()));
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
        double angle = getHeading() + 90;
        if (angle > 180) {
            angle -= 360;
        }
        int prev = strafeTicks();
        robot.pt.add(xCovered(angle, distance), yCovered(angle, distance));
        pidStrafe.reset();
        double initial = homingBeacon() - getHeading();
        if (initial > 180) {
            initial -= 360;
        } else if (initial < -180) {
            initial += 360;
        }
        pidStrafe.setSetpoint(initial);
        pidStrafe.setOutputRange(0, power);

        pidStrafe.setInputRange(-90, 90);
        //pidStrafe.enable();
        correction = pidDrive.performPID(getAngle());
        robot.topLeft.setTargetPosition(robot.topLeft.getCurrentPosition() + target);
        topRight.setTargetPosition(topRight.getCurrentPosition() -target);
        botLeft.setTargetPosition(botLeft.getCurrentPosition() -target);
        botRight.setTargetPosition(botRight.getCurrentPosition() + target);
        setRunMode(true);
        robot.topLeft.setPower(power + correction);
        topRight.setPower(-power - correction);
        botLeft.setPower(-power - correction);
        botRight.setPower(power + correction);
        while (motorsAreBusy()) {
            correction = pidDrive.performPID(getAngle());
            robot.topLeft.setPower(power + correction);
            topRight.setPower(-power - correction);
            botLeft.setPower(-power - correction);
            botRight.setPower(power + correction);
            robot.pt.add(xCovered(angle, ticksToStrafe(strafeTicks() - prev)), yCovered(angle, ticksToStrafe(strafeTicks() - prev)));
            prev = strafeTicks();
            angle = getHeading() + 90;
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
        double angle = getHeading() - 90;
        if (angle < -180) {
            angle += 360;
        }
        robot.pt.add(xCovered(angle, distance), yCovered(angle, distance));
        pidStrafe.reset();
        double initial = homingBeacon() - getHeading();
        if (initial > 180) {
            initial -= 360;
        } else if (initial < -180) {
            initial += 360;
        }
        pidStrafe.setSetpoint(initial);
        pidStrafe.setOutputRange(0, power);
        pidStrafe.setInputRange(-90, 90);
        //pidStrafe.enable();
        correction = pidDrive.performPID(getAngle());
        int target = strafeToTicks(distance);
        robot.topLeft.setTargetPosition(robot.topLeft.getCurrentPosition() -target);
        topRight.setTargetPosition(topRight.getCurrentPosition() + target);
        botLeft.setTargetPosition(botLeft.getCurrentPosition() + target);
        botRight.setTargetPosition(botRight.getCurrentPosition() -target);
        setRunMode(true);
        robot.topLeft.setPower(-power + correction);
        topRight.setPower(power - correction);
        botLeft.setPower(power - correction);
        botRight.setPower(-power + correction);
        while (motorsAreBusy()) {
            correction = pidDrive.performPID(getAngle());
            robot.topLeft.setPower(-power + correction);
            topRight.setPower(power - correction);
            botLeft.setPower(power - correction);
            botRight.setPower(-power + correction);
            robot.pt.add(xCovered(angle, ticksToStrafe(strafeTicks() - prev)), yCovered(angle, ticksToStrafe(strafeTicks() - prev)));
            prev = strafeTicks();
            angle = getHeading() - 90;
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
        robot.topLeft.setPower(power);
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

    public double getHeading() {//for overall
        orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double val = Double.parseDouble(formatAngle(orientation.angleUnit, orientation.firstAngle)) - constant;
        if (val > 180) {
            val -= 360;
        } else if (val < -180) {
            val += 360;
        }
        return val;
    }
    public void setHeading(double desired){
        constant = getHeading() - desired;
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
        pidRotate.setTolerance(5);
        pidRotate.enable();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        // rotate until turn is completed.

        if (degrees < 0) {
            // On right turn we have to get off zero first.
            while (getAngle() == 0) {
                robot.topLeft.setPower(-power);
                botLeft.setPower(-power);
                topRight.setPower(power);
                botRight.setPower(power);
                sleep(100);
            }

            do {
                power = pidRotate.performPID(getAngle()); // power will be - on right turn.
                robot.topLeft.setPower(power);
                botLeft.setPower(power);
                topRight.setPower(-power);
                botRight.setPower(-power);
            } while (!pidRotate.onTarget());
        } else    // left turn.
            do {
                power = pidRotate.performPID(getAngle()); // power will be + on left turn.
                robot.topLeft.setPower(power);
                botLeft.setPower(power);
                topRight.setPower(-power);
                botRight.setPower(-power);
            } while (!pidRotate.onTarget());

        // turn the motors off.
        stopAndResetTicks();

        rotation = getAngle();

        // wait for rotation to stop.
        if (!sleepReq) {
            sleep(tempSleep);
        }
        sleepReq = false;
        // reset angle tracking on new heading.
        resetAngle();
        updateOrientation();
    }

    public boolean offTrack = false;
    public double ultimate = 0;
    public boolean grabbed;
    public void forward(double power, double distance) {
        updateOrientation();
        resetAngle();
        resetTicks();
        distance = Math.abs(distance);
        int prev = avgTicks();
        double initial = homingBeacon() - getHeading();//auto correct to straight heading
        if (offTrack) {
            initial = 0;//don't auto correct to a straight heading and keep on inital course
            offTrack = false;
        }
        if (ultimate != 0) {
            initial = ultimate - getHeading();
        }
        if(initial > 180){
            initial -= 360;
        }
        else if(initial < -180){
            initial += 360;
        }
        //robot.pt.add(xCovered(getHeading(), distance), yCovered(getHeading(), distance));
        pidDrive.reset();
        pidDrive.setSetpoint(initial);
        pidDrive.setOutputRange(0, power);
        pidDrive.setInputRange(-90, 90);
        pidDrive.enable();
        correction = pidDrive.performPID(getAngle());
        //setRunMode(false);//set the motors to RUN_USING_ENCODER
        resetTicks();//reset encoder at the very beginning in case we didn't already do so
        robot.topLeft.setTargetPosition(robot.topLeft.getCurrentPosition() + inchesToTicks(distance));//inches to ticks conversion
        topRight.setTargetPosition(topRight.getCurrentPosition() + inchesToTicks(distance));
        botLeft.setTargetPosition(botLeft.getCurrentPosition() + inchesToTicks(distance));
        botRight.setTargetPosition(botRight.getCurrentPosition() + inchesToTicks(distance));
        setRunMode(true);//set the motors to RUN_TO_POSITION mode
        robot.topLeft.setPower(Range.clip(power + correction, -1, 1));
        botLeft.setPower(Range.clip(power + correction, -1, 1));
        topRight.setPower(Range.clip(power - correction, -1, 1));
        botRight.setPower(Range.clip(power - correction, -1, 1));
        while (motorsAreBusy()) {
            updateOrientation();
            pidDrive.setSetpoint(initial);
            correction = pidDrive.performPID(getAngle());//use PID to autocoreect to desired location
            robot.topLeft.setPower(Range.clip(power + correction, -1, 1));
            botLeft.setPower(Range.clip(power + correction, -1, 1));
            topRight.setPower(Range.clip(power - correction, -1, 1));
            botRight.setPower(Range.clip(power - correction, -1, 1));
            if(foundReq){
                robot.pullUp();
                foundReq = false;
                foundationLocation.setPoint(bridge.getX(), robot.pt.getY());
            }
            if (intakeReq) {
                intake();
                armIn();
                release();
                if (blockDistance.getDistance(DistanceUnit.CM) < 10) {
                    intakeOff();
                    grip();
                    grabbed = true;
                    break;
                }
            }
            if (armReq && avgTicks() > robot.topLeft.getTargetPosition() / 4) {
                release();
                armIn();
                armReq = false;
            }
            idle();
            sleep(200);
        }//don't execute further code until after the wheels are done turning
       //stop and reset tick count on wheels
        intakeReq = false;
        armReq = false;
        //robot.pt.add(xCovered(getHeading(), ticksToInches(avgTicks() - prev)), yCovered(getHeading(), ticksToInches(avgTicks() - prev)));//adding the inches we cover based on our current heading
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
        ultimate = 0;
    }
    public boolean foundReq = false;
    public boolean intakeReq = false;
    public void curveForward(double power, CurvePoint curve, boolean chase, boolean foundation, boolean intake, boolean off, double ulty, boolean cont){
        double distance;
        if(curve.isCurveX()){
            distance = robot.pt.getMatchX(curve, getHeading());
        }
        else{
            distance = robot.pt.getMatchY(curve, getHeading());
        }
        //distance = Math.abs(distance);
        offTrack = false;
        sleepReq = true;
        drive(power, distance);
        resetAngle();
        distance = Math.abs(robot.pt.distanceTo(curve.getTarget()));
        int prev = avgTicks();
        double initial = robot.pt.angleTo(curve.getTarget(), true) - getHeading();//auto correct to straight heading
        if(ulty != 0){
            initial = ulty - getHeading();
        }
        if(initial > 180){
            initial -= 360;
        }
        else if(initial < -180){
            initial += 360;
        }
        pidCurve.reset();
        pidCurve.setSetpoint(initial);
        pidCurve.setOutputRange(0, 4*power/2);
        pidCurve.setInputRange(-150, 150);
        //pidCurve.setInputRange(0, initial);
        pidCurve.enable();
        correction = pidCurve.performPID(getAngle());
        //setRunMode(false);//set the motors to RUN_USING_ENCODER
        resetTicks();//reset encoder at the very beginning in case we didn't already do so
        robot.topLeft.setTargetPosition(robot.topLeft.getCurrentPosition() + inchesToTicks(distance));//inches to ticks conversion
        topRight.setTargetPosition(topRight.getCurrentPosition() + inchesToTicks(distance));
        botLeft.setTargetPosition(botLeft.getCurrentPosition() + inchesToTicks(distance));
        botRight.setTargetPosition(botRight.getCurrentPosition() + inchesToTicks(distance));
        setRunMode(true);//set the motors to RUN_TO_POSITION mode
        robot.topLeft.setPower(Range.clip(power + correction, -1, 1));
        botLeft.setPower(Range.clip(power + correction, -1, 1));
        topRight.setPower(Range.clip(power - correction, -1, 1));
        botRight.setPower(Range.clip(power - correction, -1, 1));
        while (motorsAreBusy()) {
            pidCurve.setSetpoint(initial);
            correction = pidCurve.performPID(getAngle());//use PID to autocoreect to desired location
            robot.topLeft.setPower(Range.clip(power + correction, -1, 1));
            botLeft.setPower(Range.clip(power + correction, -1, 1));
            topRight.setPower(Range.clip(power - correction, -1, 1));
            botRight.setPower(Range.clip(power - correction, -1, 1));
            prev = avgTicks();//setting the last known tick count
            if(chase){
                resetAngle();
                initial = robot.pt.angleTo(curve.getTarget(), true) - getHeading();
                if(initial > 180){
                    initial -= 360;
                }
                else if(initial < -180){
                    initial += 360;
                }
            }
            /*if(Math.abs(initial - getAngle()) <= 10){
                updateOrientation();
                resetTicks();
                offTrack = off;
                foundReq = foundation;
                intakeReq = intake;
                pidCurve.disable();
                if(ulty == 0){
                    ultimate = robot.pt.angleTo(curve.getTarget(), true);
                }
                else{
                    ultimate = ulty;
                }
                sleepReq = cont;
                forward(power, robot.pt.distanceTo(curve.getTarget()));
                return;
            }*/
            idle();
            sleep(200);
        }//don't execute further code until after the wheels are done turning
        resetTicks();//stop and reset tick count on wheels
        if (!sleepReq) {
            sleep(tempSleep);
        }
        sleepReq = false;
        pidCurve.disable();
        updateOrientation();
        ultimate = 0;
    }
    public void curveBack(double power, CurvePoint curve, boolean chase, boolean foundation, boolean intake, boolean arm, boolean off, boolean cont, double ulty, double multy) {
        double distance;
        if(curve.isCurveY()){
            distance = robot.pt.getMatchY(curve, getHeading());
        }
        else{
            distance = robot.pt.getMatchX(curve, getHeading());
        }
        //distance = Math.abs(distance);
        offTrack = off;
        sleepReq = true;
        drive(power, distance);

        resetAngle();
        resetTicks();
        distance = Math.abs(robot.pt.distanceTo(curve.getTarget()));
        double initial = robot.pt.angleTo(curve.getTarget(), false) - getHeading();
        int prev = avgTicks();
        if (ulty != 0) {
            initial = ulty - getHeading();
        }
        if(initial > 180){
            initial -= 360;
        }
        else if(initial < -180){
            initial +=360;
        }
        //setRunMode(false);
        pidCurve.reset();
        pidCurve.setSetpoint(initial);
        pidCurve.setOutputRange(0, 4*power/2);
        pidCurve.setInputRange(-90, 90);
        //pidCurve.setInputRange(0, initial);
        pidCurve.enable();
        pidCurve.enable();
        correction = pidCurve.performPID(getAngle());
        resetTicks();
        robot.topLeft.setTargetPosition(robot.topLeft.getCurrentPosition()-inchesToTicks(distance));//inches to ticks conversion
        topRight.setTargetPosition(topRight.getCurrentPosition()-inchesToTicks(distance));
        botLeft.setTargetPosition(botLeft.getCurrentPosition()-inchesToTicks(distance));
        botRight.setTargetPosition(botRight.getCurrentPosition()-inchesToTicks(distance));

        setRunMode(true);

        robot.topLeft.setPower(Range.clip(-power + correction, -1, 1));
        botLeft.setPower(Range.clip(-power + correction, -1, 1));
        topRight.setPower(Range.clip(-power - correction, -1, 1));
        botRight.setPower(Range.clip(-power - correction, -1, 1));
        while (motorsAreBusy()) {
            pidCurve.setSetpoint(initial);
            correction = pidCurve.performPID(getAngle());
            robot.topLeft.setPower(Range.clip(-power + correction, -1, 1));
            botLeft.setPower(Range.clip(-power + correction, -1, 1));
            topRight.setPower(Range.clip(-power - correction, -1, 1));
            botRight.setPower(Range.clip(-power - correction, -1, 1));
            //robot.pt.add(xCovered(getHeading(), -ticksToInches(Math.abs(avgTicks()) - Math.abs(prev))), yCovered(getHeading(), -ticksToInches(Math.abs(avgTicks()) - Math.abs(prev))));
            robot.pt.add(xCovered(getHeading(), -ticksToInches(Math.abs(avgTicks()) - prev)), yCovered(getHeading(), -ticksToInches(Math.abs(avgTicks() - prev))));
            prev = avgTicks();
            if(Math.abs(initial - getAngle()) <= 10){
                foundReq = foundation;
                intakeReq = intake;
                armReq = arm;
                offTrack = off;
                updateOrientation();
                sleepReq = cont;
                if(ulty == 0) {
                    ultimate = robot.pt.angleTo(curve.getTarget(), false);
                }
                else{
                    ultimate = ulty;
                }
                pidCurve.disable();
                backward(power*multy, robot.pt.distanceTo(curve.getTarget()));
                return;
            }
            if(chase){
                resetAngle();
                initial = robot.pt.angleTo(curve.getTarget(), false) - getHeading();
                if(initial > 180){
                    initial -= 360;
                }
                else if(initial < -180){
                    initial += 360;
                }
                //chase = false;
            }
            idle();
            sleep(100);
        }//don't execute further code until after the wheels are done turning
        resetTicks();//stop and reset tick count on wheels
        if (!sleepReq) {
            sleep(tempSleep);
        }
        sleepReq = false;
        pidCurve.disable();
        updateOrientation();
    }

    public void reset(){
        sleepReq = false;
        ultimate = 0;

        foundReq = false;
        armReq = false;
    }
    public void backward(double power, double distance) {
        updateOrientation();
        resetAngle();
        resetTicks();
        distance = Math.abs(distance);
        double initial = homingBeacon() - getHeading();
        int prev = avgTicks();
        if (offTrack) {
            initial = 0;
            offTrack = false;
        }
        if (ultimate != 0) {
            initial = ultimate - getHeading();
        }
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
        correction = pidDrive.performPID(getAngle());

        robot.topLeft.setTargetPosition(robot.topLeft.getCurrentPosition() -inchesToTicks(distance));//inches to ticks conversion
        topRight.setTargetPosition(topRight.getCurrentPosition() -inchesToTicks(distance));
        botLeft.setTargetPosition(botLeft.getCurrentPosition() -inchesToTicks(distance));
        botRight.setTargetPosition(botRight.getCurrentPosition() -inchesToTicks(distance));

        setRunMode(true);

        robot.topLeft.setPower(Range.clip(-power + correction, -1, 1));
        botLeft.setPower(Range.clip(-power + correction, -1, 1));
        topRight.setPower(Range.clip(-power - correction, -1, 1));
        botRight.setPower(Range.clip(-power - correction, -1, 1));
        while (motorsAreBusy()) {
            updateOrientation();
            pidDrive.setSetpoint(initial);
            correction = pidDrive.performPID(getAngle());
            robot.topLeft.setPower(Range.clip(-power + correction, -1, 1));
            botLeft.setPower(Range.clip(-power + correction, -1, 1));
            topRight.setPower(Range.clip(-power - correction, -1, 1));
            botRight.setPower(Range.clip(-power - correction, -1, 1));
            //robot.pt.add(xCovered(getHeading(), -ticksToInches(avgTicks()) - prev), yCovered(getHeading(), -ticksToInches(avgTicks() - prev)));
            //robot.pt.add(xCovered(getHeading(), -ticksToInches(Math.abs(avgTicks()- prev))), yCovered(getHeading(), -ticksToInches(Math.abs(avgTicks() - prev))));
            prev = avgTicks();
            if(armReq && avgTicks() < robot.topLeft.getTargetPosition() + 200){
                armMid();
                armReq = false;
            }

            if(foundReq && avgTicks() < robot.topLeft.getTargetPosition() + 5){
                pullDown();
                foundReq = false;
            }
            if (intakeReq && avgTicks() < robot.topLeft.getTargetPosition() + 700) {
                intakeOff();
                grip();
                intakeReq = false;
            }
            idle();
            sleep(100);
        }//don't execute further code until after the wheels are done turning
        //resetTicks();//stop and reset tick count on wheels
        //robot.pt.add(xCovered(getHeading(), -ticksToInches(Math.abs(avgTicks()- prev))), yCovered(getHeading(), -ticksToInches(Math.abs(avgTicks() - prev))));
        if(foundReq){
            pullDown();
            foundReq = false;
        }
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
        ultimate = 0;
    }

    public void drive(double power, double distance) {
        if (distance > 0) {
            forward(power, distance);
        } else if (distance < 0) {
            backward(power, Math.abs(distance));
        }
    }

    public void turnTo(double power, Coordinate desired, boolean facing) {
        offTrack = true;
        orient(power, robot.pt.angleTo(desired, facing));
    }

    public void timeForward(double power, double time) {
        setRunMode(false);
        robot.topLeft.setPower(power);
        topRight.setPower(power);
        botLeft.setPower(power);
        botRight.setPower(power);
        sleep((long) (time * 1000));
        resetTicks();
    }

    public void timeBackward(double power, double time) {
        setRunMode(false);
        robot.topLeft.setPower(-power);
        topRight.setPower(-power);
        botLeft.setPower(-power);
        botRight.setPower(-power);
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