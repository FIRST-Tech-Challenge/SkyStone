package org.firstinspires.ftc.robotlib.servo;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;

public class BoundServo implements Servo
{
    // A reference to what the servo really is
    private Servo servo;
    private int openPos;
    private int closePos;

    public BoundServo (Servo servo, int openPos, int closePos)
    {
        this.servo = servo;
        this.openPos = openPos;
        this.closePos = closePos;
    }

    public BoundServo(Servo servo)
    {
        this(servo, 1, 0);
    }

    public void openPosition()
    {
        setPosition(openPos);
    }

    public void closePosition()
    {
        setPosition(closePos);
    }

    // From Servo interface
    @Override
    public void setDirection(Direction direction)
    { servo.setDirection(direction); }

    @Override
    public Direction getDirection()
    { return servo.getDirection(); }

    @Override
    public void setPosition(double position)
    { servo.setPosition(position); }

    @Override
    public double getPosition()
    { return servo.getPosition(); }

    @Override
    public void scaleRange(double min, double max)
    { servo.scaleRange(min, max); }


    // Useless Servo interface functions
    @Override
    public ServoController getController() { return servo.getController(); }

    @Override
    public int getPortNumber() { return servo.getPortNumber(); }

    @Override
    public Manufacturer getManufacturer() { return servo.getManufacturer(); }

    @Override
    public String getDeviceName() { return servo.getDeviceName(); }

    @Override
    public String getConnectionInfo() { return servo.getConnectionInfo(); }

    @Override
    public int getVersion() { return servo.getVersion(); }

    @Override
    public void resetDeviceConfigurationForOpMode() { }

    @Override
    public void close() { }
}
