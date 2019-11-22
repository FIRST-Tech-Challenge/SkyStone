package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.Comparison;

@Config
public class Elevator extends Subsystem {

    private DcMotor elevatorMotor;
    private DigitalChannel magneticLimitSwitch;

    //PID constants for the elevator
    public static double kP = 0.3;
    public static double kI = 0.05;
    public static double kD = 0;

    //Feed Forward Constant for the Elevator
    public static double kFF = 0.01;
    public static double kStatic = 0.118777073737;

    public static double kV = 0.0643202688898;
    public static double kA = 0.000127390384279;

    private static final double g = 386.22;

    public static double maxSpeed = 20;
    public static double maxAcceleration = 7;
    public static final double maxJerk = 0;

    public static double CORRECTION_CONSTANT = 1.19;

    private static final double winchDiameter = 1.55;
    private static final double gearRatioToWinch = -1.0;
    private static final double elevatorOffset = 2.5;

    private static final double assemblyMass = 10;

    private MotionProfile motionProfile;
    private PIDFController pidfController;

    private SystemState currentState = SystemState.IDLE;

    private long startTime;

    public enum SystemState {
        IDLE, MOVE_TO_TARGET, HOLD, LOWERING_TO_PLACE, DRIVER_CONTROLLED
    }

    //Units are in inches above its resting height (reference is with respect to the bottom of the elevator, not floor)
    private double currentHeight = 0;
    private double goalHeight = 0;
    private double currentEncoder = 0;

    private static Elevator elevator;

    public static Elevator getInstance(HardwareMap hardwareMap){
        if(elevator == null){
            elevator = new Elevator(hardwareMap);
        }
        return elevator;
    }

    public Elevator(HardwareMap hardwareMap){
        elevatorMotor = (DcMotor) hardwareMap.get("elevatorMotor");
        elevatorMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        elevatorMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        magneticLimitSwitch = hardwareMap.get(DigitalChannel.class, "elevatorSwitch");
        magneticLimitSwitch.setMode(DigitalChannel.Mode.INPUT);
        motionProfile = MotionProfileGenerator.generateSimpleMotionProfile(new MotionState(0,0),new MotionState(0,0),maxSpeed,maxAcceleration,maxJerk);
        startTime = System.currentTimeMillis();

        pidfController = new PIDFController(new PIDCoefficients(kP,kI,kD));
    }

    public void update(){
        switch (currentState){
            case IDLE:
                updateIDLEState();
                break;
            case HOLD:
                updateHOLDState();
                break;
            case MOVE_TO_TARGET:
                updateMOVE_TO_TARGETState();
                break;
            case LOWERING_TO_PLACE:
                updateLOWERING_TO_PLACEState();
                break;
        }

        //Zero Position if True
        if(!magneticLimitSwitch.getState()){
            zeroSensors();
        }
        updatePosition();
    }

    //General States
    private SystemState handleDefaultTransitions(SystemState wantedState){
        currentState = wantedState;
        switch(wantedState){
            case IDLE:
                return SystemState.IDLE;
            case MOVE_TO_TARGET:
                return SystemState.MOVE_TO_TARGET;
            case HOLD:
                return SystemState.HOLD;
            case LOWERING_TO_PLACE:
                return SystemState.LOWERING_TO_PLACE;
            case DRIVER_CONTROLLED:
                return SystemState.DRIVER_CONTROLLED;
        }
        return SystemState.IDLE;
    }

    //IDLE State
    private SystemState handleTransitionToIDLEState(SystemState desiredState){
        startTime = System.currentTimeMillis();
        return handleDefaultTransitions(desiredState);
    }

    private void updateIDLEState(){
        if(!Comparison.equalToEpsilon(elevatorMotor.getPower(),0)){
            elevatorMotor.setPower(0);
        }
    }

    //MOVE_TO_TARGET State
    private SystemState handleTransitionToMOVE_TO_TARGETState(SystemState futureState, double targetHeight){
        motionProfile = MotionProfileGenerator.generateSimpleMotionProfile(new MotionState(currentHeight,0,0,0),new MotionState(targetHeight,0,0,0),maxSpeed,maxAcceleration,maxJerk);
        goalHeight = targetHeight;
        startTime = System.currentTimeMillis();
        return handleDefaultTransitions(futureState);
    }

    private void updateMOVE_TO_TARGETState(){
        double t = (System.currentTimeMillis() - startTime) / 1000.0;
        if(t > motionProfile.duration()){
            handleTransitionToHOLDState(SystemState.HOLD);
            goalHeight = motionProfile.end().getX();
        }
        MotionState targetState = motionProfile.get(t);
        double error = getRelativeHeight() - targetState.getX();
        double correction = pidfController.update(error);
        double feedForward = calculateFeedForward(targetState,getCurrentMass());
        setMotorPowers(feedForward - correction);
    }

    //HOLD State
    private SystemState handleTransitionToHOLDState(SystemState futureState){
        startTime = System.currentTimeMillis();
        return handleDefaultTransitions(futureState);
    }

    private void updateHOLDState(){
        double error = getRelativeHeight() - goalHeight;
        double correction = pidfController.update(error);
        setMotorPowers(-correction);
    }

    public void setMotorPowers(double power){
        elevatorMotor.setPower(power);
    }

    //LOWERING_TO_PLACE State
    private SystemState handleTransitionToLOWERING_TO_PLACE(SystemState futureState){
        motionProfile = MotionProfileGenerator.generateSimpleMotionProfile(new MotionState(currentHeight,0,0,0),new MotionState(currentHeight-1,0,0,0),maxSpeed,maxAcceleration,maxJerk);
        goalHeight = currentHeight - 1;
        startTime = System.currentTimeMillis();
        return handleDefaultTransitions(futureState);
    }

    private void updateLOWERING_TO_PLACEState(){
        double t = (System.currentTimeMillis() - startTime) / 1000.0;
        if(t > motionProfile.duration()){
            handleTransitionToHOLDState(SystemState.HOLD);
            goalHeight = motionProfile.end().getX();
        }
        MotionState targetState = motionProfile.get(t);
        double error = getRelativeHeight() - targetState.getX();
        double correction = pidfController.update(error);
        double feedForward = calculateFeedForward(targetState,getCurrentMass());
        setMotorPowers(feedForward - correction);
    }

    //DRIVER_CONTROLLED
    public void setDriverControlled(){
        handleDefaultTransitions(SystemState.DRIVER_CONTROLLED);
    }

    @Override
    public void stop() {
        setMotorPowers(0);
        currentState = handleTransitionToIDLEState(SystemState.IDLE);
    }

    public void updatePosition(){
        currentHeight += (elevatorMotor.getCurrentPosition() - currentEncoder) / 753.2 * winchDiameter * gearRatioToWinch * Math.PI * CORRECTION_CONSTANT;
        currentEncoder = elevatorMotor.getCurrentPosition();
    }

    public double getRelativeHeight(){
        return currentHeight;
    }

    public double getAbsoluteHeight(){
        return getRelativeHeight() + elevatorOffset;
    }

    private double calculateFeedForward(MotionState targetState, double mass){
        double feedForward = kV * targetState.getV() + kA * mass * (targetState.getA() - g);
        if (Comparison.equalToEpsilon(feedForward,kStatic)){
            feedForward +=  Math.copySign(kFF, feedForward);
        }
        return feedForward;
    }

    //TODO Calculate Masses
    public double getCurrentMass(){
        return assemblyMass;
    }

    @Override
    public void zeroSensors() {
        elevatorMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
        currentHeight = 0;
    }

    public void setPosition(double goalHeight){
        handleTransitionToMOVE_TO_TARGETState(SystemState.MOVE_TO_TARGET,goalHeight);
    }

    public SystemState getCurrentState(){
        return currentState;
    }

    public void lowerToPlace(){
        handleTransitionToLOWERING_TO_PLACE(SystemState.LOWERING_TO_PLACE);
    }

    public void resetEncoder(){
        elevatorMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
        elevatorMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        currentEncoder = elevatorMotor.getCurrentPosition();
    }

    public double getEncoderPosition(){
        return currentEncoder;
    }
}
