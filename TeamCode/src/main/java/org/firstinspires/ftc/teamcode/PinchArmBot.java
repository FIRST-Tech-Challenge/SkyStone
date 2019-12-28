package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class PinchArmBot extends FourWheelsDriveBot {
    static final double INCREMENT   = 0.01;     // amount to slew servo each CYCLE_MS cycle
    static final int    CYCLE_MS    =   500;     // period of each cycle
    static final double MAX_POS     =  1.0;     // Maximum rotational position
    static final double MIN_POS     =  0.0;     // Minimum rotational position
    static final double PINCH_ARM_FOLD = 0.3;
    static final double PINCH_ARM_VERTICAL = 0.55;
    static final double PINCH_ARM_DOWN1 = 0.85;
    static final double PINCH_ARM_DOWN2 = 0.7;
    static final double PINCH_PINCH = 0.5;
    static final double PINCH_RELEASE = 0.2;
    static final double FOUNDATION_DRAG = 0.9;



    public Servo servoArm = null;
    public Servo servoPinch = null;
    double  position = (MAX_POS - MIN_POS) / 2; // Start at halfway position
    boolean rampUp = true;

    public PinchArmBot(LinearOpMode opMode) {
        super(opMode);
    }

    @Override
    public void init(HardwareMap ahwMap) {
        super.init(ahwMap);

        // Connect to servo (Assume PushBot Left Hand)
        // Change the text in quotes to match any servo name on your robot.
        servoArm = hwMap.servo.get("servoArm");
        servoPinch = hwMap.servo.get("servoPinch");

        servoArm.setPosition(PINCH_ARM_FOLD);
        servoPinch.setPosition(PINCH_RELEASE);
        print(String.format("After Init : ARM POS : %f, PINCH POS : %f", servoArm.getPosition(), servoPinch.getPosition()));
    }


    public void pickupSkyStone(){

        print(String.format("Before Down: ARM POS : %f, PINCH POS : %f", servoArm.getPosition(), servoPinch.getPosition()));
        servoArm.setPosition(PINCH_ARM_DOWN1);
        print(String.format("After Down: ARM POS : %f, PINCH POS : %f", servoArm.getPosition(), servoPinch.getPosition()));
        opMode.sleep(1000);
        servoPinch.setPosition(PINCH_PINCH);
        print(String.format("After Pinch: ARM POS : %f, PINCH POS : %f", servoArm.getPosition(), servoPinch.getPosition()));
        opMode.sleep(500);
        servoArm.setPosition(PINCH_ARM_FOLD);
        print(String.format("After Fold: ARM POS : %f, PINCH POS : %f", servoArm.getPosition(), servoPinch.getPosition()));
//        opMode.idle();
        return;
    }

    public void dropSkyStone(){
        servoArm.setPosition(PINCH_ARM_DOWN2);
        opMode.sleep(1000);
        servoPinch.setPosition(PINCH_RELEASE);
        opMode.sleep(500);
        servoArm.setPosition(PINCH_ARM_FOLD);
        return;
    }

    public void resetArm(){
        servoPinch.setPosition(PINCH_RELEASE);
        opMode.sleep(2000);
        servoArm.setPosition(PINCH_ARM_VERTICAL);
        opMode.sleep(1000);
        return;
    }

    public void originalPosition(){
        servoArm.setPosition(PINCH_ARM_FOLD);
        servoPinch.setPosition(PINCH_RELEASE);
    }

    public void armVertical(){
        servoArm.setPosition(PINCH_ARM_VERTICAL);
    }

    public void dragFoundation() {
        servoArm.setPosition(FOUNDATION_DRAG);
        opMode.sleep(1000);
    }

}