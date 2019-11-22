package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import java.util.EnumMap;

public class IntakeSystem {

    public enum MotorNames {
        RIGHT_INTAKE, LEFT_INTAKE, BOTTOM_INTAKE
    }

    private enum SuckDirection {
        SUCK, UNSUCK
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
        for (DcMotor motor : motors.values()) {
            motor.setPower(0.0);
        }
    }

    public void suck() {
        direction(SuckDirection.SUCK);
    }

    public void unsuck() {
        direction(SuckDirection.UNSUCK);
    }

    private void direction(SuckDirection direction) {
        for (DcMotor motor : motors.values()) {
            motor.setPower(direction == SuckDirection.SUCK ? 1.0 : -1.0);
        }
    }
}
