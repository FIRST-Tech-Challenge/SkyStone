package org.firstinspires.ftc.robotlib.motor;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

public abstract class ModifiedMotor implements DcMotor
{
    protected DcMotor motor;

    ModifiedMotor (DcMotor motor) { this.motor = motor; }

    @Override
    public MotorConfigurationType getMotorType() { return motor.getMotorType(); }

    @Override
    public void setMotorType(MotorConfigurationType motorType) { motor.setMotorType(motorType); }

    @Override
    public DcMotorController getController() { return motor.getController(); }

    @Override
    public int getPortNumber() { return motor.getPortNumber(); }

    @Override
    public void setZeroPowerBehavior(ZeroPowerBehavior zeroPowerBehavior) { motor.setZeroPowerBehavior(zeroPowerBehavior); }

    @Override
    public ZeroPowerBehavior getZeroPowerBehavior() { return motor.getZeroPowerBehavior(); }

    @Override
    public void setPowerFloat() { motor.setPowerFloat(); }

    @Override
    public boolean getPowerFloat() { return motor.getPowerFloat(); }

    @Override
    public void setTargetPosition(int position) { motor.setTargetPosition(position); }

    @Override
    public int getTargetPosition() { return motor.getTargetPosition(); }

    @Override
    public boolean isBusy() { return motor.isBusy(); }

    @Override
    public int getCurrentPosition() { return motor.getCurrentPosition(); }

    @Override
    public void setMode(RunMode mode) { motor.setMode(mode); }

    @Override
    public RunMode getMode() { return motor.getMode(); }

    @Override
    public void setDirection(Direction direction) { motor.setDirection(direction); }

    @Override
    public Direction getDirection() { return motor.getDirection(); }

    @Override
    public void setPower(double power) { motor.setPower(power); }

    @Override
    public double getPower() { return motor.getPower(); }

    @Override
    public Manufacturer getManufacturer() { return motor.getManufacturer(); }

    @Override
    public String getDeviceName() { return motor.getDeviceName(); }

    @Override
    public String getConnectionInfo() { return motor.getConnectionInfo(); }

    @Override
    public int getVersion() { return motor.getVersion(); }

    @Override
    public void resetDeviceConfigurationForOpMode() { motor.resetDeviceConfigurationForOpMode(); }

    @Override
    public void close() { motor.close(); }
}
