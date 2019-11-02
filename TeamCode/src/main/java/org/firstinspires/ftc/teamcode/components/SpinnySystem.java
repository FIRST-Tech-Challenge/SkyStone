package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import java.util.EnumMap;

public class SpinnySystem {

    public enum MotorNames {
        RIGHTINTAKE, LEFTINTAKE, BOTTOMINTAKE
    }

    private EnumMap<SpinnySystem.MotorNames, DcMotor> motors;

    public SpinnySystem(EnumMap<SpinnySystem.MotorNames, DcMotor> motors) {
        this.motors = motors;
        initMotors();
    }

    public void spin(boolean leftBumper, boolean rightBumper) {
        if (leftBumper) {
            setMotorsPower(1.0);
        } else if (rightBumper) {
            setMotorsPower(-1.0);
        } else {
            setMotorsPower(0.0);
        }
    }

    private void initMotors() {
        motors.forEach((name, motor) -> {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            if (name == MotorNames.BOTTOMINTAKE) {
                motor.setDirection(DcMotorSimple.Direction.REVERSE);
            } else {
                motor.setDirection(DcMotorSimple.Direction.FORWARD);
            }

        });
        setMotorsPower(0);
    }

    private void setMotorsPower(double power) {
        for (DcMotor motor : motors.values()) {
            motor.setPower(power);
        }
    }
}
