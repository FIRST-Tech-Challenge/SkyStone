package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import java.util.EnumMap;

public class SpinnySystem {
    public enum MotorNames {
        RIGHTINTAKE, LEFTINTAKE
    }

    public EnumMap<SpinnySystem.MotorNames, DcMotor> motors;

    public SpinnySystem(EnumMap<SpinnySystem.MotorNames, DcMotor> motors) {
        this.motors = motors;
        initMotors();
    }

    public void initMotors() {
        motors.forEach((name, motor) -> {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            if (name.equals(motors.get(MotorNames.LEFTINTAKE))) {
                motor.setDirection(DcMotorSimple.Direction.FORWARD);
            } else if (name.equals(motors.get(MotorNames.RIGHTINTAKE))) {
                motor.setDirection(DcMotorSimple.Direction.REVERSE);
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
            motors.get(MotorNames.RIGHTINTAKE).setPower(-0.4);
            motors.get(MotorNames.LEFTINTAKE).setPower(0.4);
        } else if (rightBumper) {
            motors.get(MotorNames.RIGHTINTAKE).setPower(0.4);
            motors.get(MotorNames.LEFTINTAKE).setPower(-0.4);
        }
    }
}
