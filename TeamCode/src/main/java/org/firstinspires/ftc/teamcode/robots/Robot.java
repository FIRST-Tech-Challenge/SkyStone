package org.firstinspires.ftc.teamcode.robots;

import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.RC;
import org.firstinspires.ftc.teamcode.newhardware.Motor;
import org.firstinspires.ftc.teamcode.util.MathUtils;

/**
 * Created by FIXIT on 15-08-18.
 * Default template for all raw robots. Include basic motion methods.
 */
public class Robot {
    /**
     * Motions of any robot
     */

    public static final int UP = 0;
    public static final int DOWN = 1;
    public static final int LEFT = 2;
    public static final int RIGHT = 3;
    public static final int FORWARD = 4;
    public static final int BACKWARD = 5;
    public static final int STOP = 6;
    public static final int IN = 7;
    public static final int OUT = 8;
    public static final int OPEN = 9;
    public static final int CLOSE = 10;
    public static final int CENTRE = 11;
    /**
     * This is the left drive system motor
     */
    public Motor motorL;
    /**
     * This is the right drive system motor
     */
    public Motor motorR;

    /**
     * The diameter of the wheel in inches. Default is 4 inches
     */
    public float wheelDiameter = 4;

    /**
     * The minimum power to supply the wheels when turning
     */
    private double minTurningSpeed = 0.09;

    /**
     * The built in Adafruit IMU
     */
    BNO055IMU imu;

    /**
     * The turning tolerance in degrees
     */
    private int degreeTolerance = 3;

    /**
     * Creates a default robot with a drive system in which one motor must be reversed
     * to make driving forward simpler to understand. the XML config file must call the drive system motors
     * driveR and driveL
     */
    public Robot() {
        this("driveL", "driveR");
    }//Robot

    /**
     * Creates a default Robot with the drive system being two motors
     * that can be named whatever
     * @param driveL The left drive system motor
     * @param driveR The right drive system motor
     */
    public Robot(Motor driveL, Motor driveR) {
        this.motorL = driveL;
        this.motorR = driveR;
        this.motorR.setReverse(true);

        if(RC.h.getAll(BNO055IMU.class).size() > 0){
            imu = RC.h.get(BNO055IMU.class, "imu");

            BNO055IMU.Parameters params = new BNO055IMU.Parameters();
            params.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            params.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;

            imu.initialize(params);
        }

    }//Robot

    /**
     * Creates a robot based on he name of the two motors in XML config file
     * @param driveL The name of the left drive system motor
     * @param driveR The name of the right drive system motor
     */
    public Robot(String driveL, String driveR) {
        this(new Motor(driveL), new Motor(driveR));
    }//Robot

    /**
     * Effectively pauses the thread without risk of exception
     *
     * @param time The duration the robot waits in milliseconds
     */
    public static void wait(int time) {
        if(RC.o instanceof LinearOpMode){
            RC.l.sleep(time);
        }
    }//wait

    public boolean allReady() {
        return motorL.isTimeFin() && motorR.isTimeFin();
    }//allReady

    public void checkAllSystems() {
        motorL.updateTimer();
        motorR.updateTimer();
    }//checkAllSystems


    //MOVE METHODS

    /**
     * Powers the left side of the robot
     * @param speed The speed at which the motor is powered value between -1.0 and +1.0
     */
    public void driveL(double speed) {
        motorL.setPower(speed);
    }//driveL

    /**
     * Powers the right side of the robot
     * @param speed The speed at which the motor is powered value between -1.0 and +1.0
     */
    public void driveR(double speed) {
        motorR.setPower(speed);
    }//driveR

    /**
     * Drives the robot forward. Does not stop the robot after execution.
     * @param speed The speed at which the motors turn value between 0.0 and 1.0
     */
    public void forward(double speed) {
        motorL.setPower(speed);
        motorR.setPower(speed);

    }//forward

    /**
     * The method for driving forward. Stops the robot at the end of execution.
     * @param speed The speed at which the motors turn value between 0.0 and 1.0
     * @param time The duration the motors are on
     */
    public void forward(double speed, int time) {
        motorL.setPower(speed);
        motorR.setPower(speed);
        wait(time);
        stop();
    }//forward


    public void slantforward(double leftspeed, double rightspeed){
        motorL.setPower(leftspeed);
        motorR.setPower(rightspeed);

    }

    /**
     * Drives the robot backward. Does not stop the robot after execution.
     * @param speed The speed at which the motors turn value between 0.0 and 1.0
     */
    public void backward(double speed) {
        motorL.setPower(-speed);
        motorR.setPower(-speed);

    }//backward

    /**
     * The method for driving backward. Stops the robot at the end of execution.
     * @param speed The speed at which the motors turn value between 0.0 and 1.0
     * @param time The duration the motors are on
     */
    public void backward(double speed, int time) {
        backward(speed);
        wait(time);
        stop();
    }//backward

    /**
     * Drive the robot forward for a certain distance using encoders
     * @param mm The distance in millimetres for the robot to drive
     * @param speed The speed at which the motors turn value between 0.0 and 100
     *              If speed > 1 speed will be modified to match the parameters of the Motor
     */
    public void forwardDistance(int mm, double speed) {
        int target = (int)(mm * 1120 / (wheelDiameter * 25.4 * Math.PI)); //convert mm to tiks
        target = Math.round(target);

        RC.t.addData(target);

        motorL.resetEncoder();
        motorR.resetEncoder();

        motorL.setTargetAndPower(target, speed);
        motorR.setTargetAndPower(target, speed);

        while(RC.l.opModeIsActive() && !motorR.isFin() && !motorL.isFin()){
            motorL.setPower((motorL.getTarget()-motorL.getPosition())*0.005*speed + minTurningSpeed);
            motorR.setPower((motorR.getTarget()-motorR.getPosition())*0.005*speed + minTurningSpeed);

            if(motorL.isFin()){
                motorL.stop();
            }//if
            if(motorR.isFin()){
                motorR.stop();
            }//if
//
//            if(motorR.isFin() && motorL.isFin()){
//                motorL.stop();
//                motorR.stop();
//            }

            Log.i("Encoders", "Left: " + motorL.getPosition() + ", Right: " + motorR.getPosition());
            Log.i("EncodersT", "Left: " + motorL.getTarget() + ", Right: " + motorR.getTarget());

        }//while
        stop();

    }//forwardDistance

    /**
     * Drive the robot backward for a certain distance using encoders
     * @param mm The distance in millimetres for the robot to drive
     * @param speed The speed at which the motors turn value between 0.0 and 100
     *              If speed > 1 speed will be modified to match the parameters of the Motor
     */
    public void backwardDistance(int mm, double speed) {
        int target = (int)(mm * 1120 / (wheelDiameter * 25.4 * Math.PI)); //convert mm to tiks

        motorL.resetEncoder();
        motorR.resetEncoder();

        motorL.setTargetAndPower(-target, speed);
        motorR.setTargetAndPower(-target, speed);

        while(RC.l.opModeIsActive() && !motorR.isFin() && !motorL.isFin()){
            motorL.setPower((motorL.getTarget()-motorL.getPosition())*0.005*speed + minTurningSpeed);
            motorR.setPower((motorR.getTarget()-motorR.getPosition())*0.005*speed + minTurningSpeed);

            if(motorL.isFin()){
                motorL.stop();
            }//if
            if(motorR.isFin()){
                motorR.stop();
            }//if

            Log.i("Encoders", "Left: " + motorL.getPosition() + ", Right: " + motorR.getPosition());
            Log.i("EncodersT", "Left: " + motorL.getTarget() + ", Right: " + motorR.getTarget());

        }//while
        stop();

    }//forwardDistance

    /**
     * Begins turning the robot towards the left
     * @param speed The speed at which the motors turn value between 0.0 and 1.0
     */
     public void turnR (double speed) {
         motorL.setPower(speed);
         motorR.setPower(-speed);
    }//turnR

    /**
     * Begins turning the robot towards the right
     * @param speed The speed at which the motors turn value between 0.0 and 1.0
     */
    public void turnL (double speed) {
        motorL.setPower(-speed);
        motorR.setPower(speed);
    }//turnL

    /**
     * Turns the robot right and stops it after the given period of time
     * @param speed The speed to drive
     * @param time The time it is on for
     */
    public void turnR(double speed, int time){
        turnR(speed);
        wait(time);
        stop();
    }

    /**
     * Turns the robot left and stops it after the given period of time
     * @param speed The speed to drive
     * @param time The time it is on for
     */
    public void turnL(double speed, int time) {
        turnL(speed);
        wait(time);
        stop();
    }//turnL

    /**
     * Turn right using the IMU for a given degrees. There is a built in ramping functionality.
     * @param degrees The degrees to turn
     * @param speed The speed at which to turn
     */
    public void imuTurnR(double degrees, double speed) {

        if(degrees < degreeTolerance) return;
        //if its a really small degree, don't bother ^^^
        turnR(speed);

        double beginAngle = MathUtils.cvtAngleToNewDomain(getAngle());
        //Assigns begin angle and target angle
        double targetAngle = MathUtils.cvtAngleToNewDomain(beginAngle + degrees);

        while (RC.l.opModeIsActive()) {

            double currentAngle = MathUtils.cvtAngleToNewDomain(getAngle());
            //figure out curret angl
            double angleToTurn = MathUtils.cvtAngleJumpToNewDomain(targetAngle - currentAngle);

            Log.i("Angle", currentAngle + "");
            Log.i("AnSpeeds", motorL.getPower() + ", " + motorR.getPower());

            turnR(angleToTurn / 180 * (speed - minTurningSpeed) + minTurningSpeed);

            if (angleToTurn < degreeTolerance) {
                break;
            }//if
        }//while


        stop();
    }//imuTurnR


    //SF
    public void oneMTurnL(double speed){
        motorL.setPower(speed);
    }
    public void SfIMUTurnLNoIn(double degrees, double speed) {

        if(degrees < degreeTolerance) return;
        //if its a really small degree, don't bother ^^^
        oneMTurnL(-speed);

        double beginAngle = MathUtils.cvtAngleToNewDomain(getAngle());
        //Assigns begin angle and target angle
        double targetAngle = MathUtils.cvtAngleToNewDomain(beginAngle + degrees);

        while (RC.l.opModeIsActive()) {

            double currentAngle = MathUtils.cvtAngleToNewDomain(getAngle());
            //figure out curret angl
            double angleToTurn = MathUtils.cvtAngleJumpToNewDomain(targetAngle - currentAngle);

            Log.i("Angle", currentAngle + "");
            Log.i("AnSpeeds", motorL.getPower() + ", " + motorR.getPower());

            oneMTurnL(-speed);

            if (angleToTurn < degreeTolerance) {
                break;
            }//if
        }//while


        stop();
    }//noimuTurnL
    /**
     * Turn left using the IMU for a given degrees. There is a built in ramping functionality.
     * @param degrees The degrees to turn
     * @param speed The speed at which to turn
     */
    public void imuTurnL(double degrees, double speed) {

        if(degrees < degreeTolerance) return;
        turnL(speed);

        double beginAngle = MathUtils.cvtAngleToNewDomain(getAngle());
        double targetAngle = MathUtils.cvtAngleToNewDomain(beginAngle - degrees);

        while (RC.l.opModeIsActive()) {

            double currentAngle = MathUtils.cvtAngleToNewDomain(getAngle());
            double angleToTurn = MathUtils.cvtAngleJumpToNewDomain(currentAngle - targetAngle);

            turnL(Math.signum(angleToTurn) * (Math.abs(angleToTurn) / 180 * (speed - minTurningSpeed) + minTurningSpeed));

            if (Math.abs(angleToTurn) < degreeTolerance) {
                break;
            }//if
        }//while

        stop();
    }//imuTurnL

    /**
     * Turn to a given heading based on the robot's initial heading. Initial is 0 degrees
     * @param degrees The desired absolute heading
     * @param speed The speed at which to turn
     */
    public void absoluteIMUTurn(double degrees, double speed) {
        double currentAngle = MathUtils.cvtAngleToNewDomain(getAngle());

        double toTurn = MathUtils.cvtAngleToNewDomain(degrees - currentAngle);

        if (toTurn < 0) {
            imuTurnL(-toTurn, speed);
        } else {
            imuTurnR(toTurn, speed);
        }//else
    }//absoluteIMUTurn

    public void encTurnL(int tik, double speed) {

        motorL.setTargetAndPower(-tik, -speed);
        motorR.setTargetAndPower(tik, speed);

        while(RC.l.opModeIsActive() && !motorR.isFin() && !motorL.isFin()){
            if(motorL.isFin()){
                motorL.stop();
            }//if
            if(motorR.isFin()){
                motorR.stop();
            }//if
        }//while
        stop();
    }//encTurnL

    public void encTurnR(int tik, double speed) {

        //Ines added this code
        motorL.resetEncoder();
        motorR.resetEncoder();
        RC.t.addData("Status", "Resetting Encoders");    //

        //okay done

        motorL.setTargetAndPower(tik, speed);
        motorR.setTargetAndPower(-tik, -speed);

        while(RC.l.opModeIsActive() && !motorR.isFin() && !motorL.isFin()){
            if(motorL.isFin()){
                motorL.stop();
            }//if
            if(motorR.isFin()){
                motorR.stop();
            }//if
        }//while
        stop();
    }//encTurnR

    /**
     * Stops the robot
     */
    public void stop() {
        motorL.stop();
        motorR.stop();
    }//stop

    /**
     * Stops the robot then waits for a specified time
     * @param time The duration the robot waits
     */
    public void stop(int time) {
        motorL.stop();
        motorR.stop();
        wait(time);
    }//stop

    /**Robot helper methods**/

    /**
     * Flips the direction of both motors
     */
    public void reverseDriveSystem(){
        motorL.setReverse(!motorL.isReversed());
        motorR.setReverse(!motorR.isReversed());
    }//reverseDriveSystem

    /**
     * The minimum speed the robot while it's turning.
     * @param minSpeed The minimum speed
     */
    public void setMinTurningSpeed(double minSpeed){
        if(minSpeed <= 0){
            minSpeed = 0.05;
        }
        minTurningSpeed = minSpeed;
    }//setMinTurningSpeed

    /**
     * Sets how accurate the turn must be. Too low may never stop
     * @param degTol The new degree tolerance
     */
    public void setDegreeTolerance(int degTol){
        if (degTol < 1){
            degTol = 1;
        }
        degreeTolerance = degTol;
    }//setDegreeTolerance

    /**
     * Gets the angles from the imu
     * @return the angles in the order x, y, z from the sensor as an array
     */
    public double[] getIMUAngle() {
        Orientation orient = imu.getAngularOrientation();

        return new double[] {-orient.firstAngle, -orient.secondAngle, -orient.thirdAngle};
    }//getIMUAngle

    public double getAngle(){
        return getIMUAngle()[0];
    }


}//Robot
