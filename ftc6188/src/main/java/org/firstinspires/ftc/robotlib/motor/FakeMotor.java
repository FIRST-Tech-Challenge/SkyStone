package org.firstinspires.ftc.robotlib.motor;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

// A fake motor is a motor which instead of being connected to something physical simply stores the values so you can pretend to run motors
// The fake motor can take the place of any DcMotor due to it implementing the same functions, so long as it does not matter if the motor pretends
// to "drive" and just sets power or target position
public class FakeMotor implements DcMotor
{
    // Hidden variables
    private MotorConfigurationType motorConfigurationType;
    private DcMotorController controller;
    private Manufacturer manufacturer;
    private String deviceName;
    private String connectionInfo;

    private int portNumber;
    private int version;
    private float powerFloat;

    // Useful variables
    private int targetPosition;
    private int position;
    private double power;

    private ZeroPowerBehavior zeroPowerBehavior;
    private RunMode runMode;
    private Direction direction;

    public FakeMotor (ZeroPowerBehavior zeroPowerBehavior, RunMode mode, Direction direction, String deviceName)
    {
        setZeroPowerBehavior(zeroPowerBehavior);
        setMode(mode);
        setDirection(direction);
        motorConfigurationType = new MotorConfigurationType();
        this.deviceName = deviceName;
    }

    public FakeMotor(String deviceName) { this(ZeroPowerBehavior.BRAKE, RunMode.RUN_USING_ENCODER, Direction.FORWARD, deviceName); }

    @Override
    public MotorConfigurationType getMotorType() { return motorConfigurationType; }

    @Override
    public void setMotorType(MotorConfigurationType motorType) { this.motorConfigurationType = motorType; }

    @Override
    public DcMotorController getController() { return controller; }

    @Override
    public int getPortNumber() { return portNumber; }

    @Override
    public void setZeroPowerBehavior(ZeroPowerBehavior zeroPowerBehavior) { this.zeroPowerBehavior = zeroPowerBehavior; }

    @Override
    public ZeroPowerBehavior getZeroPowerBehavior() { return zeroPowerBehavior; }

    //Ignored
    @Override
    public void setPowerFloat() { powerFloat = (float)getPower(); }

    //Ignored
    @Override
    public boolean getPowerFloat() { return false; }

    @Override
    public void setTargetPosition(int position) { this.targetPosition = position; }

    @Override
    public int getTargetPosition() { return targetPosition; }

    @Override
    public boolean isBusy() { return Math.abs(getPower()) > 0 || getTargetPosition() != position || true; }

    @Override
    public int getCurrentPosition() { return position; }

    @Override
    public void setMode(RunMode mode) { this.runMode = mode; }

    @Override
    public RunMode getMode() { return runMode; }

    @Override
    public void setDirection(Direction direction) { this.direction = direction; }

    @Override
    public Direction getDirection() { return direction; }

    @Override
    public void setPower(double power) { this.power = power; }

    @Override
    public double getPower() { return power; }

    @Override
    public Manufacturer getManufacturer() { return manufacturer; }

    @Override
    public String getDeviceName() { return deviceName; }

    @Override
    public String getConnectionInfo() { return connectionInfo; }

    @Override
    public int getVersion() { return version; }

    @Override
    public void resetDeviceConfigurationForOpMode() { }

    @Override
    public void close() { }
}
