package com.hfrobots.tnt.season1920;

import com.hfrobots.tnt.corelib.drive.ExtendedDcMotor;
import com.hfrobots.tnt.corelib.drive.NinjaMotor;
import com.hfrobots.tnt.corelib.util.SimplerHardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

public class DeliveryMechanism {

    Servo shoulderServo;
    Servo elbowServo;
    Servo wristServo;
    Servo fingerServo;

    ExtendedDcMotor liftMotor;


    ExtendedDcMotor leftIntakeMotor;
    ExtendedDcMotor rightIntakeMotor;

    // FIXME 2 Stow values needed. Wiring was in abysmal condition at the time of testing for
    //  in/out postions

    double shoulderTurnClose = 0.0;
    double shoulderTurnFar = 0.0; //see FIXME 2
    double shoulderStow = 1; //see FIXME 2
    boolean shoulderFar = false;

    double elbowTurnClose = 1.0;
    double elbowTurnFar = 0;
    double elbowStow = 0.686; //see FIXME 2
    boolean elbowFar = false;

    double wristTurnCloseNatural = 1.0;
    double wristTurnCloseRotated = 0.0;
    double wristTurnFarNatural = 0.0;
    double wristTurnFarRotated = 1.0;
    double wristStow = 0.556; //see FIXME 2
    boolean wristRotated = false ;

    double fingerGrip = 1.0;
    double fingerUngrip = 0.0;

    double liftMaxVelocity = 0.0; //FIXME wrong
    double liftMinVelocity = 0.0; // FIXME wrong
    double liftIdleVelocity = 0.0; //FIXME wrong
    double maxPos = 999999; //FIXME wrong
    double minPos = -999999; //FIXME wrong


    //FIXME constants and variables for lift needed here.

    public DeliveryMechanism(SimplerHardwareMap hardwareMap) {
        shoulderServo = hardwareMap.get(Servo.class, "shoulderServo");
        elbowServo = hardwareMap.get(Servo.class, "elbowServo");
        wristServo = hardwareMap.get(Servo.class, "wristServo");
        fingerServo = hardwareMap.get(Servo.class, "fingerServo");

        liftMotor = NinjaMotor.asNeverest20Orbital(hardwareMap.get(DcMotor.class, "liftMotor"));

        leftIntakeMotor = NinjaMotor.asNeverest20Orbital(hardwareMap.get(DcMotor.class, "leftIntakeMotor"));
        rightIntakeMotor = NinjaMotor.asNeverest20Orbital(hardwareMap.get(DcMotor.class, "rightIntakeMotor"));

        leftIntakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightIntakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        leftIntakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightIntakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        stow();
    }

    public void setIntakeVelocity(double velocity) {
        leftIntakeMotor.setPower(velocity);
        rightIntakeMotor.setPower(velocity);
    }

    public void gripBlock() {
        fingerServo.setPosition(fingerGrip);
    }

    public void ungripblock() {
        fingerServo.setPosition(fingerUngrip);
    }

    public void stow() {
        wristServo.setPosition(wristStow);
        sleepNoThrow(1000);
        elbowServo.setPosition(elbowStow);
        sleepNoThrow(1000);
        shoulderServo.setPosition(shoulderStow);
        elbowFar = false;
        shoulderFar = true;
    }

    public void  turnClose() {

        if (wristRotated = false) {
            wristServo.setPosition(wristTurnCloseNatural);
            elbowServo.setPosition(elbowTurnClose);
            shoulderServo.setPosition(shoulderTurnClose);
            elbowFar = false;
            shoulderFar = false;
        } else {
            wristServo.setPosition(wristTurnCloseRotated);
            elbowServo.setPosition(elbowTurnClose);
            shoulderServo.setPosition(shoulderTurnClose);
            elbowFar = false;
            shoulderFar = false;
        }
    }

    public void turnFar() {
        if (wristRotated = false){
            shoulderServo.setPosition(shoulderTurnFar);
            sleepNoThrow(1000);
            elbowServo.setPosition(elbowTurnFar);
            sleepNoThrow(1000);
            wristServo.setPosition(wristTurnFarNatural);

            elbowFar = true;
            shoulderFar = true;
        } else {
            wristServo.setPosition(wristTurnFarRotated);
            elbowServo.setPosition(elbowTurnFar);
            shoulderServo.setPosition(shoulderTurnFar);
            elbowFar = true;
            shoulderFar = true;
        }
    }

    public void rotateToPos() {
        if (elbowFar == false && shoulderFar == false) {
            if (wristRotated == false){
                wristServo.setPosition(wristTurnCloseRotated);
                wristRotated = true;
            } else {
                wristServo.setPosition(wristTurnCloseNatural);
                wristRotated = false;
            }
        } else if(elbowFar == true && shoulderFar == true){
            if (wristRotated == true){
                wristServo.setPosition(wristTurnFarRotated);
                wristRotated = false;
            } else {
                wristServo.setPosition(wristTurnFarNatural);
                wristRotated = true;
            }
        } else {
            // assumed it is stowed and should not turn at the moment
            wristServo.setPosition(wristStow);
            wristRotated = false;
        }
    }

    public void liftToMax() {
        if(liftMotor.getCurrentRelativePosition() <= maxPos) {
            liftMotor.setPower(liftMaxVelocity);
        } else {
            liftMotor.setPower(liftIdleVelocity);
        }
    }

    public void liftToMin() {
        if(liftMotor.getCurrentRelativePosition() >= minPos) {
            liftMotor.setPower(liftMinVelocity);
        } else {
            liftMotor.setPower(0);
        }
    }

    private void sleepNoThrow(long millis) {
        try {
            Thread.sleep(millis);
        } catch (InterruptedException intEx) {
            // do Nothing
        }
    }


}
