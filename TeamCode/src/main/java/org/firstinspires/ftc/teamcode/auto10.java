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

@Autonomous(name = "PHASE2 - CROSS OVER - RED", group = "S-Side")
public class auto10 extends LinearOpMode {

    ColorSensor colorSensor;

    Servo middleGrab;

    PIDController pidRotate, pidDrive;
    double globalAngle, power = .40, correction, rotation;
    Orientation lastAngles = new Orientation();
    BNO055IMU imu;
    DcMotor TL, TR, BL, BR;
    Servo hookLeft, hookRight;

    public double powerUp = 0.5, powerDown = -0.5;

    DistanceSensor distanceSensor;

    @Override
    public void runOpMode() throws InterruptedException {
        ElapsedTime runtime = new ElapsedTime();
        distanceSensor = hardwareMap.get(DistanceSensor.class, "dist");
        colorSensor = hardwareMap.get(ColorSensor.class, "cs");

        middleGrab = hardwareMap.get(Servo.class, "middleGrab");

        hookLeft = hardwareMap.get(Servo.class, "hooke");
        hookRight = hardwareMap.get(Servo.class, "hook");

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

        while (!isStopRequested() && !imu.isGyroCalibrated()) {
            sleep(50);
            idle();
        }

        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        telemetry.update();

        hookRight.setPosition(0.9);
        hookLeft.setPosition(0.9);

        middleGrab.setPosition(0.6);

        waitForStart();

        /*
        ---------------------------------------------------------------------------------
        ACTION CODE (QZ0)
        ---------------------------------------------------------------------------------
         */

        //sleep(15000);

        //strafeRight();
        strafeLeft();
        wait(1.2);
        rest();

        /*goForward(0.55);
        wait(1.4);
        rest();

        sleep(700);

        //strafe left
        runtime.reset();
        resetAngle();
        TL.setPower(0.3 + joltControl(runtime));
        TR.setPower(-0.3);
        BL.setPower(-0.3);
        BR.setPower(0.3 + joltControl(runtime));
        wait(1.9);
        rest();

        goForward(0.55);
        while(opModeIsActive()&&!tripWireActive(11.2)){
            telemetry.addData("STATUS: ", "(PHASE 1)");
            telemetry.update();
        }
        rest();

        runtime.reset();
        resetAngle();
        TL.setPower(powerUp + joltControl(runtime));
        TR.setPower(powerDown);
        BL.setPower(powerDown + joltControl(runtime));
        BR.setPower(powerUp);



        //sleep(2000);

        while(opModeIsActive()&&!SkyStoneSpotted()){
            telemetry.addData("Status: ", SkyStoneSpotted());
            telemetry.update();

        }

        rest();
        //sleep(2000);

        TL.setPower(-0.3 + joltControl(runtime));
        TR.setPower(0.3);
        BL.setPower(0.3);
        BR.setPower(-0.3 );
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
        wait(3.8, "go back (PHASE 3)");
        rest();

        goForward();
        wait(0.2);

        *//*rotate(-90, power);

        goForward();
        wait(1.2);

        dropDL();
        wait(1.0, "dropping the hookers");

        sleep(1500);
        rotate(-90, power);


       /* goForward(1.0);
        wait(1.0);*//*

       rotate(-90, power);

       rest();

        raiseDL();
        wait(2.0);


        goForward();
        wait(2.5);
        rest();

        goBack();
        wait(0.5);
        rest();
*/
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

