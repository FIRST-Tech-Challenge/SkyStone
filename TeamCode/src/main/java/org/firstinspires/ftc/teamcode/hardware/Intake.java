package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Intake extends Subsystem{

    private static Intake intake;

    private Servo grabberServo;
    private Servo releaseServo;

    private State currentState;

    public static double holdPosition = 0;
    public static double releasePosition = 0.3;
    public static double grabPosition = 0;
    public static double stoneHoldPosition = 0.5;

    public enum State {
        RELEASING, GRABBING, OPEN
    }

    public static Intake getInstance(HardwareMap hardwareMap){
        if(intake == null){
            intake = new Intake(hardwareMap);
        }
        return intake;
    }

    public Intake (HardwareMap hardwareMap){
        currentState = State.OPEN;
        grabberServo = hardwareMap.servo.get("grabberServo");
        releaseServo = hardwareMap.servo.get("releaseServo");
        grabberServo.setPosition(grabPosition);
    }


    public void update(){

    }

    public void setGrabbing(){
        currentState = State.GRABBING;
        grabberServo.setPosition(stoneHoldPosition);
    }

    public void release(){
        currentState = State.RELEASING;
        releaseServo.setPosition(releasePosition);
    }

    public void open(){
        currentState = State.OPEN;
        grabberServo.setPosition(grabPosition);
    }

    public void setHold(){
        releaseServo.setPosition(holdPosition);
    }


    @Override
    public void zeroSensors() {

    }

    @Override
    public void stop() {
        grabberServo.setPosition(grabberServo.getPosition());
        releaseServo.setPosition(releaseServo.getPosition());
    }


    public boolean isBusy(){
        if(currentState.equals(State.OPEN)){
            return false;
        }
        return true;
    }

    public State getCurrentState(){
        return currentState;
    }
}
