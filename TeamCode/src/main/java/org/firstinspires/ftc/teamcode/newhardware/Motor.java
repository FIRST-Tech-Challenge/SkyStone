package org.firstinspires.ftc.teamcode.newhardware;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.RC;

/**
 * Created by FIXIT on 15-08-18.
 * This class take the basic functionality of DcMotor but adds some methods to it
 */
public class Motor implements FXTDevice, Timeable {

    private DcMotor m;

    private long targetTime = -1;
    private double plannedSpeed = 0;

    protected int numTiksPerRev = 1120;
    private MotorType motorType = MotorType.AM40;

    private int beginningPosition = 0;
    private double minSpeed = 0.09;
    private int positioningAccuracy = 60;
    //was 20

    public enum MotorType {
        AM20,
        AM40,
        AM60,
        Tetrix,
    }//Type

    /**
     * Initializes a new Motor based on a pre-existing DcMotor
     * @param motor a pre-existing DcMotor
     * @see DcMotor
     */

    public Motor (DcMotor motor) {
        this.m = motor;
        m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }//Motor

    public Motor(String address) {
        this(RC.h.dcMotor.get(address));
    }//Motor

    public DcMotor getM() {
        synchronized (m) {
            return m;
        }//synchronized
    }//getM

    /***************
    MOTOR PROPERTIES
     ***************/

    public int getNumTiksPerRev() {
        return numTiksPerRev;
    }//getNumTiksPerRev

    public void setMotorType(MotorType motor){
        switch (motor){
            case AM20: numTiksPerRev = 560;
                break;
            case AM40: numTiksPerRev = 1120;
                break;
            case AM60: numTiksPerRev = 1680;
                break;
            case Tetrix: numTiksPerRev = 1440;
                break;
        }//switch

        this.motorType = motor;
    }//setMotorType

    public MotorType getMotorType() {
        return motorType;
    }//getMotorType

    public void setMode(DcMotor.RunMode mode) {
        synchronized (m) {
            m.setMode(mode);
        }//synchronized
    }//setMode

    /**
     * Changes whether a motor spins forward or reverse based on a positive input value.
     * @param reverse If true the motor is reversed
     */
    public void setReverse (boolean reverse) {
        synchronized (m) {
            if (reverse) {
                m.setDirection(DcMotorSimple.Direction.REVERSE);
            } else {
                m.setDirection(DcMotorSimple.Direction.FORWARD);
            }//else
        }//synchronized
    }//setReverse

    /**
     * Check if the motor is reversed
     * @return true if the motor is reversed false otherwise
     */
    public boolean isReversed(){
        synchronized (m) {
            DcMotorSimple.Direction dir = m.getDirection();
            return dir == DcMotorSimple.Direction.REVERSE;
        }//synchronized
    }//isReversed



    /*****************************
    ENCODER TARGET & TIMER METHODS
     *****************************/

    public void setAbsoluteTarget(int target){
        setTarget(getBaseCurrentPosition() + target - getAbsolutePosition());
    }//setAbsoluteTarget

    public void setTarget(int tik) {
        synchronized (m) {
            m.setTargetPosition(tik);
        }//synchronized
    }//setTargetAndPower

    public void setRelativeTarget(int tik) {
        setTarget(tik + getBaseCurrentPosition());
    }//setTargetAndPower

    public void addToTarget(int tik) {
        setTarget(getTarget() + tik);
    }

    public int getTarget(){
        synchronized (m) {
            return m.getTargetPosition();
        }//synchronized
    }//getTargetPosition

    public void setPositioningAccuracy (int tikAcc) {
        this.positioningAccuracy = tikAcc;
    }//setPositioningAccuracy

    public int getPositioningAccuracy() {
        return positioningAccuracy;
    }//getPositioningAccuracy

    public void setTimer(int millis, double speed) {
        setPower(speed);
        targetTime = System.currentTimeMillis() + millis;
    }//setTimer

    @Deprecated
    public void toggleChecking(boolean check) {
        if (check) {
            setMode(DcMotor.RunMode.RUN_TO_POSITION);
        } else {
            setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }//else
    }//toggleChecking

    public void setTargetAndPower(int target, double speed) {
        setRelativeTarget(target);
        setPower(speed);
    }//setTargetAndPower


    /*********************
    ENCODER & TIMER STATUS
     *********************/

    public boolean isBusy() {
        synchronized (m) {
            return m.isBusy();
        }//synchronized
    }//isBusy

    public boolean isFin() {
        return Math.abs(getBaseCurrentPosition() - getM().getTargetPosition()) < positioningAccuracy;
    }//finished

    public boolean isTimeFin() {
        return targetTime == -1;
    }//isTimeFin

    public void updateTimer() {
        if (!getM().getMode().equals(DcMotor.RunMode.RUN_TO_POSITION)) {
            if (System.currentTimeMillis() > targetTime && targetTime != -1) {
                stop();
                targetTime = -1;
            }//if
        }//if
    }//run


    /**********
    MOTOR POWER
     **********/

    public void setPower(double power) {

        if (Math.abs(power) > 1) {
            power = 1 * Math.signum(power);
        } else if (Math.abs(power) < minSpeed && Math.abs(power) > 1E-10) {
            power = minSpeed * Math.signum(power);
        } else if(Math.abs(power) < 1E-10){
            power = 0; //prevent round off error
        }//else

        synchronized (m) {
            m.setPower(power);
        }//synchronized
    }//setPower

    public double getPower() {
        synchronized (m) {
            return m.getPower();
        }//synchronized
    }//getPower

    public void stop() {
        setPower(0);
    }//stop

    public void usePlannedSpeed() {
        setPower(plannedSpeed);
    }//usePlannedSpeed


    /*********************
    MOTOR SPEED PROPERTIES
     *********************/

    public void setPlannedSpeed(double plannedSpeed) {
        this.plannedSpeed = plannedSpeed;
    }//setPlannedSpeed

    public double getPlannedSpeed() {
        return this.plannedSpeed;
    }//setPlannedSpeed

    public void setMinimumSpeed(double minimumSpeed) {
        this.minSpeed = minimumSpeed;
    }//setMinimumSpeed

    public double getMinimumSpeed() {
        return minSpeed;
    }//getMinimumSpeed


    /********************************
    POSITION ACCESSORS & MANIPULATORS
     ********************************/

    /**
     * @return the position within the current revolution of this motor (relative to beginningPosition)
     */
    public int getAbsolutePosition(){
        return (getPosition() - beginningPosition) % numTiksPerRev;
    }//getAbsolutePosition

    /**
     * @return the encoder tiks that the motor has moved since its encoder was reset using default FTC libraries
     */
    public int getBaseCurrentPosition() {
        synchronized (m) {
            return m.getCurrentPosition();
        }//synchronized
    }//getBaseCurrentPosition

    /**
     * @return the encoder tiks that the motor has moved since its encoder was reset using our library
     */
    public int getPosition() {
        return getBaseCurrentPosition() - getBeginningPosition();
    }//getPosition

    /**
     * @return the encoder tiks at which the motor began
     */
    public int getBeginningPosition() {
        return beginningPosition;
    }//getBeginningPosition

    /**
     * @param beginningPosition the encoder tik position at which the motor began
     */
    public void setBeginningPosition(int beginningPosition) {
        this.beginningPosition = beginningPosition;
    }//setBeginningPosition

    public void resetEncoder(){
        setBeginningPosition(getBaseCurrentPosition());
    }//resetEncoder



    /**********************
    AUTOMATED MOTOR MOTIONS
     **********************/

    public void beginRunToPosition(int tiks, double speed){

        setMode(DcMotor.RunMode.RUN_TO_POSITION);

        setRelativeTarget(tiks);
        setPower(speed);
    }//beginRunToPosition

    public void runToPosition(int tiks, double speed) {
        beginRunToPosition(tiks, speed);

        while (isTimeFin()) {
            RC.l.idle();
        }//while

        stop();
    }//completeRunToPosition

    /******
    LOGGING
     ******/
    public String returnCurrentState() {
        return "Current Pos: " + getBaseCurrentPosition() +
                ", Power: " + getPower() +
                ", Target: " + getM().getTargetPosition();
    }//returnCurrentState

}//Motor
