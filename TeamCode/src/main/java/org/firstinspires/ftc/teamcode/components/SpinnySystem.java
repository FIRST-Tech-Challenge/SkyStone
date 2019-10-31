package org.firstinspires.ftc.teamcode.components;

import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import java.util.EnumMap;

public class SpinnySystem {
    private final String TAG = "SpinnySystem";

    public enum MotorNames {
        RIGHTINTAKE, LEFTINTAKE, BOTTOMINTAKE
    }

    public EnumMap<SpinnySystem.MotorNames, DcMotor> motors;

    public SpinnySystem(EnumMap<SpinnySystem.MotorNames, DcMotor> motors) {
        this.motors = motors;
        initMotors();
    }

    public void initMotors() {
        Log.d(TAG, "setting motor values");
        for (DcMotor motor : motors.values()) {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setDirection(DcMotorSimple.Direction.FORWARD);
        }
        setMotorPower(0);
    }

    public void setMotorPower(double power) {
        for (DcMotor motor : motors.values()) {
            motor.setPower(power);
        }
    }

    public void spin(boolean leftBumper, boolean rightBumper) {
        if (leftBumper) {
            Log.d(TAG, "left bumper --> spin in");
            for (DcMotor motor : motors.values()) {
                motor.setPower(0.4);
            }
        } else if (rightBumper) {
            Log.d(TAG, "left bumper --> spin out");
            for (DcMotor motor : motors.values()) {
                motor.setPower(-0.4);
            }
        }
    }
}
