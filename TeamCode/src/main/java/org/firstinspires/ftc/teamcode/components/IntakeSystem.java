package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import java.util.EnumMap;

public class IntakeSystem {

    public enum MotorNames {
        RIGHT_INTAKE, LEFT_INTAKE, BOTTOM_INTAKE
    }

    private EnumMap<IntakeSystem.MotorNames, DcMotor> motors;

    public IntakeSystem(EnumMap<IntakeSystem.MotorNames, DcMotor> motors) {
        this.motors = motors;
        initMotors();
    }

    private void initMotors() {
        motors.forEach((name, motor) -> {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            if (name == MotorNames.RIGHT_INTAKE) {
                motor.setDirection(DcMotorSimple.Direction.REVERSE);
            } else {
                motor.setDirection(DcMotorSimple.Direction.FORWARD);
            }
            motor.setPower(0.0);
        });
    }

    public void stop() {
        setMotorPowers(0.0);
    }

    public void suck() {
        setMotorPowers(1.0);
    }

    public void unsuck() {
        setMotorPowers(-1.0);
    }

    private void setMotorPowers(double power) {
        for (DcMotor motor : motors.values()) {
            motor.setPower(power);
        }
    }
}
