package org.firstinspires.ftc.teamcode.components;

import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import java.util.EnumMap;

public class SpinnySystem {

    public enum MotorNames {
        RIGHTINTAKE, LEFTINTAKE, BOTTOMINTAKE
    }

    public EnumMap<SpinnySystem.MotorNames, DcMotor> motors;

    public SpinnySystem(EnumMap<SpinnySystem.MotorNames, DcMotor> motors) {
        this.motors = motors;
        initMotors();
    }

    public void initMotors() {
        motors.forEach((name, motor) -> {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            if (name == MotorNames.BOTTOMINTAKE) {
                motor.setDirection(DcMotorSimple.Direction.REVERSE);
            } else {
                motor.setDirection(DcMotorSimple.Direction.FORWARD);
            }

        });
        setMotorPower(0);
    }

    public void setMotorPower(double power) {
        for (DcMotor motor : motors.values()) {
            motor.setPower(power);
        }
    }

    public void spin(boolean leftBumper, boolean rightBumper) {
        if (leftBumper) {
            for (DcMotor motor : motors.values()) {
                motor.setPower(1.0);
            }
        } else if (rightBumper) {
            for (DcMotor motor : motors.values()) {
                motor.setPower(-1.0);
            }
        }
        setMotorPower(0);
    }
}
