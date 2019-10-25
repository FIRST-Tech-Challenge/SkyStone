package org.firstinspires.ftc.teamcode.components;

import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import java.util.EnumMap;

public class SpinnySystem {
    private final String TAG = "SpinnySystem";

    public enum MotorNames {
        RIGHTINTAKE, LEFTINTAKE
    }

    public EnumMap<SpinnySystem.MotorNames, DcMotor> motors;

    public SpinnySystem(EnumMap<SpinnySystem.MotorNames, DcMotor> motors) {
        this.motors = motors;
        initMotors();
    }

    public void initMotors() {
        Log.d(TAG, "in init");
        motors.forEach((name, motor) -> {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setDirection(name == MotorNames.LEFTINTAKE ? DcMotorSimple.Direction.FORWARD : DcMotorSimple.Direction.REVERSE);
        });
        setMotorPower(0);
    }

    public void setMotorPower(double power) {
        Log.d(TAG, "in set power");
        for (DcMotor motor : motors.values()) {
            motor.setPower(power);
        }
    }

    public void spin(boolean leftBumper, boolean rightBumper) {
        Log.d(TAG, "in spin");
        if (leftBumper) {
            motors.get(MotorNames.RIGHTINTAKE).setPower(-0.4);
            motors.get(MotorNames.LEFTINTAKE).setPower(0.4);
        } else if (rightBumper) {
            motors.get(MotorNames.RIGHTINTAKE).setPower(0.4);
            motors.get(MotorNames.LEFTINTAKE).setPower(-0.4);
        }
    }
}
