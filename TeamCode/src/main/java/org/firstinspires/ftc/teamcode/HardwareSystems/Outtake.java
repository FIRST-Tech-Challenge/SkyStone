package org.firstinspires.ftc.teamcode.HardwareSystems;

import org.firstinspires.ftc.teamcode.Utility.RobotHardware;

public class Outtake {

    // Servo values for flipper
    private double flippedRight = 0.16, flippedLeft = 0.84; //Should sum to 1 at all times
    private double insideRight = 0.75, insideLeft = 0.25;

    // Servo values for clamp
    private double clampedFront = 0.7, clampedBack = 0.7;
    private double droppedFront = 0.35, droppedBack = 0.35;

    public void initialize(){
        setGripperState("Receive");
        setFlipperState("Receive");
    }

    public void setGripperState(String state) {

        double front, back;
        if(state.equals("Receive")) {
            front = droppedFront;
            back = clampedBack;
        }else if(state.equals("Clamped")) {
            front = clampedFront;
            back = clampedBack;
        }else if(state.equals("Deposited")) {
            front = clampedFront;
            back = droppedBack;
        }else if(state.equals("Open")) {
            front = droppedFront;
            back = droppedBack;
        }else{
            front = droppedFront;
            back = clampedBack;
        }

        RobotHardware.blockGrabberFront.setPosition(front);
        RobotHardware.blockGrabberBack.setPosition(back);

    }


    public void setFlipperState(String state){
        if(state.equals("Receive")){
            RobotHardware.flipperServoRight.setPosition(insideRight);
            RobotHardware.flipperServoLeft.setPosition(insideLeft);
        }else if(state.equals("Outside")){
            RobotHardware.flipperServoRight.setPosition(flippedRight);
            RobotHardware.flipperServoLeft.setPosition(flippedLeft);
        }
    }

    public void setFlipperPosition(double position) { //Position is of the right servo

        if(position < flippedRight) {
            position = flippedRight;
        }else if (position > insideRight){
            position = insideRight;
        }

        RobotHardware.flipperServoRight.setPosition(position);
        RobotHardware.flipperServoLeft.setPosition(1 - position);

    }

    public void setGripperPosition(double position){ //Only works if positions are the same
        if(position > clampedFront){
            position = clampedBack; //I did this just to get rid of annoying warnings since they are the same.
        }else if(position < droppedFront){
            position = droppedBack;
        }

        RobotHardware.blockGrabberFront.setPosition(position);
        RobotHardware.blockGrabberBack.setPosition(position);

    }

}
