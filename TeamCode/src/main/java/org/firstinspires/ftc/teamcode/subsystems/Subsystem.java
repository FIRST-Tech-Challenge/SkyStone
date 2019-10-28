package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.HashMap;


public class Subsystem {
    DcMotor[] motors;
    Servo[] servos;

    public void initMotors(DcMotor[] motors) {
        this.motors = motors;
    }

    public void initServos(Servo[] servos) {
        this.servos = servos;
    }
    //Constructors

    public void setMotorPowers(double power) {
        for (DcMotor motor : motors) {
            motor.setPower(power);
        }
    }

    public HashMap<String, Integer> getMotorPositions() {
        HashMap<String, Integer> positions = new HashMap<>();
        for (DcMotor motor : motors) {
            positions.put(motor.getDeviceName(), motor.getCurrentPosition());
        }
        return positions;
    }

    public void setZeroBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        for (DcMotor motor : motors) {
            motor.setZeroPowerBehavior(zeroPowerBehavior);
        }
    }

    public void reverseMotors(DcMotor[] reverseMotors) {
        for (DcMotor reverseMotor : reverseMotors) {
            reverseMotor.setDirection(DcMotor.Direction.REVERSE);
        }
    }

    public void setServoPosition(double position) {
        for (Servo servo : servos) {
            servo.setPosition(position);
        }
    }

    public void reset() {
        for (DcMotor motor : motors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }

    public void setMode(DcMotor.RunMode runMode) {
        for (DcMotor motor : motors) {
            motor.setMode(runMode);
        }
    }

}
