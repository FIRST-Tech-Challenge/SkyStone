package org.firstinspires.ftc.teamcode.HardwareSystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Utility.RobotHardware;

public class Outtake {

    private LinearOpMode opMode;
    private RobotHardware hardware;

    // Servo values for flipper
    private double flippedRight = 0.16, flippedLeft = 0.84; //Should sum to 1 at all times
    private double insideRight = 0.75, insideLeft = 0.25;

    // Servo values for clamp
    private double clampedFront = 0.7, clampedBack = 0.7;
    private double droppedFront = 0.35, droppedBack = 0.35;

    public Outtake(LinearOpMode opMode, RobotHardware hardware){
        this.opMode = opMode;
        this.hardware = hardware;
    }

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

        hardware.blockGrabberFront.setPosition(front);
        hardware.blockGrabberBack.setPosition(back);

    }


    public void setFlipperState(String state){
        if(state.equals("Receive")){
            hardware.flipperServoRight.setPosition(insideRight);
            hardware.flipperServoLeft.setPosition(insideLeft);
        }else if(state.equals("Outside")){
            hardware.flipperServoRight.setPosition(flippedRight);
            hardware.flipperServoLeft.setPosition(flippedLeft);
        }
    }

    public void setFlipperPosition(double position) { //Position is of the right servo

        if(position < flippedRight) {
            position = flippedRight;
        }else if (position > insideRight){
            position = insideRight;
        }

        hardware.flipperServoRight.setPosition(position);
        hardware.flipperServoLeft.setPosition(1 - position);

    }

    public void setGripperPosition(double position){ //Only works if positions are the same
        if(position > clampedFront){
            position = clampedBack; //I did this just to get rid of annoying warnings since they are the same.
        }else if(position < droppedFront){
            position = droppedBack;
        }

        hardware.blockGrabberFront.setPosition(position);
        hardware.blockGrabberBack.setPosition(position);

    }

}
