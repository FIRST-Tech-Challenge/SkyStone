package org.firstinspires.ftc.robotlib.motor;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;

public class CalibratingMotor extends ModifiedMotor
{
    // touch sensor to be referenced during calibration
    DigitalChannel touchSensor;

    public CalibratingMotor(DcMotor motor, DigitalChannel touchSensor)
    {
        super(motor);
        this.touchSensor = touchSensor;
    }

    // calibrate to 0 runs the motor to the sensor and defines that point as 0
    public void calibrateToZero(Direction direction, double velocity)
    {
        // store pre calibration parameters
        Direction storedDirection = this.motor.getDirection();
        RunMode storedMode = this.motor.getMode();

        // set motor to calibration style parameters
        this.motor.setDirection(direction);
        this.motor.setMode(RunMode.RUN_USING_ENCODER);

        // move the motor in the direction of the sensor
        while (!touchSensor.getState()) { this.motor.setPower(Math.abs(velocity)); }

        // reset the encoder to pos 0
        this.motor.setMode(RunMode.STOP_AND_RESET_ENCODER);

        // reset to pre calibration parameters
        this.motor.setMode(storedMode);
        this.motor.setDirection(storedDirection);
    }
}
