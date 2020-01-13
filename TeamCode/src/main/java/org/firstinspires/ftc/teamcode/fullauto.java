package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name = "FULL-Auto-BLUE", group = "FINAL")
//@Disabled
public class fullauto extends LinearOpMode {
    ColorSensor colorSensor;

    PIDController pidRotate, pidDrive;
    double globalAngle, power = .50, correction, rotation;
    Orientation lastAngles = new Orientation();
    BNO055IMU imu;
    DcMotor TL, TR, BL, BR;
    Servo hookLeft, hookRight, middleGrab;

    public double powerUp = 0.5, powerDown = -0.5;

    DistanceSensor distanceSensor;

    Boolean PHASE1, PHASE2, PHASE3, PHASE4,  PHASE5, PHASE6, PHASE7, PHASE8, PHASE9;


    @Override
    public void runOpMode() throws InterruptedException {

        ElapsedTime runtime = new ElapsedTime();
        distanceSensor = hardwareMap.get(DistanceSensor.class, "dist");

        hookLeft = hardwareMap.get(Servo.class, "hook");
        hookRight = hardwareMap.get(Servo.class, "hooke");
        middleGrab = hardwareMap.get(Servo.class, "middleGrab");

        colorSensor = hardwareMap.get(ColorSensor.class, "cs");

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
        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        middleGrab.setPosition(0.0);

        while (!isStopRequested() && !imu.isGyroCalibrated()) {
            sleep(50);
            idle();
        }

        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        telemetry.update();

        hookRight.setPosition(0.9);
        hookLeft.setPosition(0.9);

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
        --------------------------------------------------------------------------------------
        ACTION CODE (QZ0)
        -------------------------------------------------------------------------------------
         */

        /*
        PHASE 0 Description: Goes forward slightly to get the robot off the wall in preparation for the rotation
         */
        goForward(0.45);
        wait(1.6);
        rest();

        sleep(700);

        //strafe left
        runtime.reset();
        resetAngle();
        TL.setPower(powerDown);
        TR.setPower(powerUp);
        BL.setPower(powerUp);
        BR.setPower(powerDown + joltControl(runtime));
        wait(1.8);
        rest();

        goForward(0.45);
        while(opModeIsActive()&&!tripWireActive(9.8)){
            telemetry.addData("STATUS: ", "(PHASE 1)");
            telemetry.update();
        }

        rest();

        runtime.reset();
        resetAngle();
        TL.setPower(powerDown + joltControl(runtime));
        TR.setPower(powerUp);
        BL.setPower(powerUp);
        BR.setPower(powerDown);

        //sleep(2000);

        while(opModeIsActive()&&!SkyStoneSpotted()){
            telemetry.addData("Status: ", SkyStoneSpotted());
            telemetry.update();

        }

        rest();
        sleep(2000);

        TL.setPower(-0.2 + joltControl(runtime));
        TR.setPower(0.2);
        BL.setPower(0.2);
        BR.setPower(-0.2);
        while(opModeIsActive()&&SkyStoneSpotted()){
            telemetry.addData("Status: ", SkyStoneSpotted());
            telemetry.update();
        }
        sleep(300);
        rest();
        goForward();
        wait(0.2, "pushing forward");

        grabServo();

        wait(0.6, "grabbing Servo (PHASE 2)");

        goBack();
        wait(1.2, "go back (PHASE 3)");
        rest();

        dropDL();
        wait(1.0, "dropping the hookers");

        sleep(1500);
        rotate(90, power);

        goForward(1.0);
        wait(1.0);


        raiseDL();
        wait(1.7);

        rest();

        goBackwards();
        wait(0.2);
        rest();

        /*
        --------------------------------------------------------------
        --------------------------------------------------------------

         */

        rotate(-90, power);

        TL.setPower(powerUp + joltControl(runtime));
        TR.setPower(powerDown);
        BL.setPower(powerDown);
        BR.setPower(powerUp);

        wait(0.7);
        rest();

        resetAngle();
        goForwards();
        while (opModeIsActive() && (!tripWireActive(9.5))) {
            telemetry.addData("Status: ", "going towards foundation (PHASE 4)");
            telemetry.update();
        }
        rest();

        /*
        PHASE 5 Description: Drops the hookers
         */
        dropDL();
        wait(2.0, "dropping hookers (PHASE 5)");

        /*
        PHASE 6 Description: Goes back towards the depot
         */
        resetAngle();
        goBackwards();
        wait(3.0, "going backwards towards depot (PHASE 6)");
        rest();
        sleep(2000);

        /*
        PHASE 7: Raises the Hookers and prepares to move our of the way
         */
        raiseDL();
        wait(2.0);

        /*
        PHASE 8 Description: makes sure that the robot is clear of the foundation
         */
        goBackwards();
        wait(0.5, "repositioning (PHASE 8)");
        rest();

        goForwards();
        wait(0.2);

        /*
        PHASE 9 Description: strafes to the right and parks
         */
        strafeRight();
        wait(4.0, "strafing out and parking (PHASE 8)");
        rest();


        telemetry.addData("Status: ", "MISSION COMPLETE (PHASE 10)");
        telemetry.update();


    }

    public void grabServo(){
        middleGrab.setPosition(1.0);
    }

    public boolean colorCheclerGreen(ColorSensor cs, int GVal, int tolerance){

        return ((cs.green()< (GVal+tolerance)) && ((cs.green()>GVal-tolerance)));
    }

    public boolean colorCheclerBlue(ColorSensor cs, int BVal, int tolerance){

        return ((cs.green()< (BVal+tolerance)) && ((cs.green()>BVal-tolerance)));
    }

    public boolean colorCheclerRed(ColorSensor cs, int RVal, int tolerance){

        return ((cs.green()< (RVal+tolerance)) && ((cs.green()>RVal-tolerance)));
    }

    public void goForwards(){
        TL.setPower(-(power - correction));
        BL.setPower(-(power - correction));
        TR.setPower(-(power + correction));
        BR.setPower(-(power + correction));
    }

    public void goBackwards(){
        TL.setPower((power - correction));
        BL.setPower((power - correction));
        TR.setPower((power + correction));
        BR.setPower((power + correction));
    }

    public boolean SkyStoneSpotted(){
        int HUE = 16777216;
        int GVal = 14;
        int BVal = 10;
        int RVal = 10;

        int tolerance = 10;

        if (colorCheclerGreen(colorSensor, GVal, tolerance) && colorCheclerBlue(colorSensor, BVal, tolerance) && colorCheclerRed(colorSensor, RVal, tolerance)){
            return true;
        }
        else {
            return false;
        }
    }

    public void raiseDL() {
        hookLeft.setPosition(0.9);
        hookRight.setPosition(0.9);
        middleGrab.setPosition(0.0);
    }

    public void rest() {
        TL.setPower(0);
        TR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);
    }

    public void dropDL() {
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
        //sleep(500);
    }

    public void strafeCorrection(){
        double pulseStrength = 0.05;

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

    public void strafeCorrection(int i){
        double pulseStrength = 0.003;

        if (getAngle()>5){
            powerUp = powerUp + pulseStrength;
        }
        if (getAngle()<-5) {
            powerDown = powerDown + pulseStrength;
        }
        else {
            pulseStrength = 0;
            powerUp = 0.4;
            powerDown = 0.4;
        }
    }

    private void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    public double joltControl(ElapsedTime runtime) {
        if (runtime.seconds() < 1.0) {
            return 0.0050;
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
    public void rotate(int degrees, double power) {
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

    public void goForward(){
        TL.setPower(-(power - correction));
        BL.setPower(-(power - correction));
        TR.setPower(-(power + correction));
        BR.setPower(-(power + correction));
    }

    public void goForward(double power1){
        TL.setPower(-(power1 - correction));
        BL.setPower(-(power1 - correction));
        TR.setPower(-(power1 + correction));
        BR.setPower(-(power1 + correction));
    }

    public void goBack() {
        TL.setPower((power - correction));
        BL.setPower((power - correction));
        TR.setPower((power + correction));
        BR.setPower((power + correction));
    }

    public void strafeRight(){
        //TL.setPower(powerDown + joltControl());
        TR.setPower(powerUp);
        BL.setPower(powerUp);
        BR.setPower(powerDown);
    }

    public void strafeLeft(){
        //TL.setPower(powerUp + joltControl());
        TR.setPower(powerDown);
        BL.setPower(powerDown);
        BR.setPower(powerUp);
    }
}
