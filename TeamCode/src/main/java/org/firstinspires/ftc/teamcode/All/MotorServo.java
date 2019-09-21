package org.firstinspires.ftc.teamcode.All;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;

public class MotorServo implements Servo {
    DcMotorEx motor;
    MotorConfiguration configuration;
    double kp, ki, kd;
    double previousError, errorSum;

    public MotorServo(DcMotorEx motor, MotorConfiguration configuration) {
        this.motor = motor;
    }

    double target;

    public static class MotorConfiguration {

        public static MotorConfiguration firstJoint = new MotorConfiguration(1680, 1);
        public static MotorConfiguration secondJoint = new MotorConfiguration(1680, 1);

        public double encoderTicksPerRevolution;

        public MotorConfiguration(double baseRatio, double gearRatio) {
            encoderTicksPerRevolution = baseRatio * gearRatio;
        }
    }

    @Override
    public HardwareDevice.Manufacturer getManufacturer() {
        return null;
    }

    public void close() {

    }

    @Override
    public void resetDeviceConfigurationForOpMode() {

    }

    @Override
    public void scaleRange(double min, double max) {

    }

    @Override
    public ServoController getController() {
        return null;
    }

    @Override
    public String getDeviceName() {
        return null;
    }

    @Override
    public int getPortNumber() {
        return 0;
    }

    @Override
    public int getVersion() {
        return 0;
    }

    @Override
    public double getPosition() {
        return target / configuration.encoderTicksPerRevolution;
    }

    @Override
    public String getConnectionInfo() {
        return null;
    }

    @Override
    public Servo.Direction getDirection() {
        return null;
    }

    @Override
    public void setDirection(Servo.Direction direction) {

    }

    public void setPosition(double position) {
        target = position * configuration.encoderTicksPerRevolution;
        errorSum = 0;
        previousError = 0;
    }

    public void update() {
        double error = motor.getCurrentPosition() - target;
        double adjustment = kp * error + ki * errorSum + kd * (error - previousError);
        motor.setPower(adjustment);
        previousError = error;
        errorSum += error;

    }
}
