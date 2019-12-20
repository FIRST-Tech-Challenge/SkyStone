package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class FoundationGrabber extends Subsystem {

    private static FoundationGrabber foundationGrabber;

    private interface PositionGetter{
        double getPosition();
    }

    public enum Positions implements PositionGetter{
        UP_RIGHT {
            public double getPosition() {
                return 0.50;
            }
        }, UP_LEFT{
            public double getPosition() {
                return 0.85;
            }
        }, DOWN_RIGHT {
            public double getPosition() {
                return 0.85;
            }
        }, DOWN_LEFT{
            public double getPosition() {
                return 0.50;
            }
        }
    }

    Positions currentPosition;

    //Perspective is when looking at the robot in the orientation it is in (Forward)
    //Right is being used as 0 for position getting
    //Left is being used as 1 for position getting
    private Servo rightServo; //Port 1, ExpansionHub 1
    private Servo leftServo; //Port 2, ExpansionHub 1
    private TouchSensor rightTouchSensor;
    private TouchSensor leftTouchSensor;


    public static FoundationGrabber getInstance(HardwareMap hardwareMap){
        if (foundationGrabber == null){
            foundationGrabber = new FoundationGrabber(hardwareMap);
        }
        return foundationGrabber;
    }

    public FoundationGrabber(HardwareMap hardwareMap){
        rightServo = (Servo) hardwareMap.get("rightServo");
        leftServo = (Servo) hardwareMap.get("leftServo");
        rightTouchSensor = (TouchSensor) hardwareMap.get("rightTouchSensor");
        leftTouchSensor = (TouchSensor) hardwareMap.get("leftTouchSensor");
    }

    public void update(){
        if(currentPosition.equals(Positions.UP_LEFT) || currentPosition.equals(Positions.UP_RIGHT)){
            rightServo.setPosition(Positions.UP_RIGHT.getPosition());
            leftServo.setPosition(Positions.UP_LEFT.getPosition());
        } else {
            rightServo.setPosition(Positions.DOWN_RIGHT.getPosition());
            leftServo.setPosition(Positions.DOWN_LEFT.getPosition());
        }
    }

    public void stop() {
        rightServo.setPosition(rightServo.getPosition());
        leftServo.setPosition((leftServo.getPosition()));
    }

    public void setCurrentPosition(Positions position){
        currentPosition = position;
        if(currentPosition.equals(Positions.UP_LEFT)){
            rightServo.setPosition(Positions.UP_RIGHT.getPosition());
            leftServo.setPosition(Positions.UP_LEFT.getPosition());
        } else {
            rightServo.setPosition(Positions.DOWN_RIGHT.getPosition());
            leftServo.setPosition(Positions.DOWN_LEFT.getPosition());
        }
    }


    public void zeroSensors(){

    }

    public Positions getCurrentPosition(){
        return currentPosition;
    }

    public double getRightPosition(){
        return rightServo.getPosition();
    }

    public double getLeftPosition(){
        return leftServo.getPosition();
    }

    public boolean isLinedUpWithFoundation(){
        return leftTouchSensor.isPressed() && rightTouchSensor.isPressed();
    }

    public int headingNeededToTurn(){
        if(isLinedUpWithFoundation() || (!leftTouchSensor.isPressed() && !rightTouchSensor.isPressed())){
            return 0;
        }
        if(rightTouchSensor.isPressed() && !leftTouchSensor.isPressed()){
            return 1;
        } else if(!rightTouchSensor.isPressed() && leftTouchSensor.isPressed()){
            return -1;
        }
        return 0;
    }
}
