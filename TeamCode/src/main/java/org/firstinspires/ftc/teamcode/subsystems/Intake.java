package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


public class Intake extends Subsystem {
    Servo wrist;
    Servo main;
    double wristHighPosition;
    double wristLowPosition;
    double mainHighPosition = .75;
    double mainLowPosition = .25;




    public Intake(HardwareMap hardwareMap) {
        wrist = hardwareMap.servo.get("wrist");
        main = hardwareMap.servo.get("main");
        initServos(new Servo[]{wrist, main});
        initIntake();
    }

    public void initIntake() {

    }

// all below are not finished yet, will do saturday, Ian.

    public void setwrist(double position) {
        wrist.setPosition(position);
    }

    public void setmain(double position) {
        main.setPosition(position);
    }

// values need to be tested

    public void setWristToHighPosition() {
        wrist.setPosition(wristHighPosition);
    }

// values need to be tested

    public void setWristToLowPosition() {
        wrist.setPosition(wristLowPosition);
    }

    // values need to be tested
    public void setMainToHighPosition() {
        main.setPosition(mainHighPosition);
    }

// values need to be tested

    public void setMainToLowPosition() {
        main.setPosition(mainLowPosition);
    }

    public void run(boolean setHigh) {
        if(setHigh){
            setMainToHighPosition();
        }else{
            setMainToLowPosition();
        }
    }
}
