package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


public class Intake extends Subsystem {
    Servo wrist;
    Servo grip;
    double wristHighPosition;
    double wristLowPosition;
    double gripHighPosition = .75;
    double gripLowPosition = .25;


    public Intake(HardwareMap hardwareMap) {
        wrist = hardwareMap.servo.get("wrist");
        grip = hardwareMap.servo.get("grip");
        initServos(new Servo[]{wrist, grip});
        initIntake();
    }

    public void initIntake() {

    }

// all below are not finished yet, will do saturday, Ian.

    public void setwrist(double position) {
        wrist.setPosition(position);
    }

    public void setgrip(double position) {
        grip.setPosition(position);
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
    public void setgripToHighPosition() {
        grip.setPosition(gripHighPosition);
    }

// values need to be tested

    public void setgripToLowPosition() {
        grip.setPosition(gripLowPosition);
    }

    public void run(boolean switchgrip) {
        if (switchgrip) {
            if (grip.getPosition() == gripHighPosition) {
                setgripToLowPosition();
            } else {
                setgripToHighPosition();
            }
        }
    }
}
