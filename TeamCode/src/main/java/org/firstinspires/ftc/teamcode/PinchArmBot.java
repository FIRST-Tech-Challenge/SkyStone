package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class PinchArmBot extends TensorFlowBot {
    static final double INCREMENT   = 0.01;     // amount to slew servo each CYCLE_MS cycle
    static final int    CYCLE_MS    =   500;     // period of each cycle
    static final double MAX_POS     =  1.0;     // Maximum rotational position
    static final double MIN_POS     =  0.0;     // Minimum rotational position
    static final double PINCH_ARM_FOLD = 0.3;
    static final double PINCH_ARM_VERTICLE = 0.55;
    static final double PINCH_ARM_DOWN = 0.95;
    static final double PINCH_PINCH = 0.5;
    static final double PINCH_RELEASE = 0.3;




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

        print(String.format("Before Init : ARM POS : %f, PINCH POS : %f", servoArm.getPosition(), servoPinch.getPosition()));

        servoArm.setPosition(PINCH_ARM_FOLD);
        servoPinch.setPosition(PINCH_RELEASE);
        print(String.format("After Init : ARM POS : %f, PINCH POS : %f", servoArm.getPosition(), servoPinch.getPosition()));
    }

    public void pickupSkyStone(){

        print(String.format("Before Down: ARM POS : %f, PINCH POS : %f", servoArm.getPosition(), servoPinch.getPosition()));
        servoArm.setPosition(PINCH_ARM_DOWN);
        print(String.format("After Down: ARM POS : %f, PINCH POS : %f", servoArm.getPosition(), servoPinch.getPosition()));
        opMode.sleep(2000);
        servoPinch.setPosition(PINCH_PINCH);
        print(String.format("After Pinch: ARM POS : %f, PINCH POS : %f", servoArm.getPosition(), servoPinch.getPosition()));
        opMode.sleep(2000);
        servoArm.setPosition(PINCH_ARM_FOLD);
        print(String.format("After Fold: ARM POS : %f, PINCH POS : %f", servoArm.getPosition(), servoPinch.getPosition()));
//        opMode.idle();
        return;
    }

    public void dropSkyStone(){
        servoArm.setPosition(PINCH_ARM_DOWN);
        opMode.sleep(2000);
        servoPinch.setPosition(PINCH_RELEASE);
        opMode.sleep(2000);
        servoArm.setPosition(PINCH_ARM_FOLD);
        opMode.sleep(2*1000);
        return;
    }

    public void resetArm(){
        servoPinch.setPosition(PINCH_RELEASE);
        opMode.sleep(2000);
        servoArm.setPosition(PINCH_ARM_FOLD);
        opMode.sleep(2*1000);
        return;
    }

    public void dragFoundation() {
        servoArm.setPosition(PINCH_ARM_DOWN);
        opMode.sleep(2000);
    }

}
