package org.firstinspires.ftc.robotlib.servo;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;

import org.firstinspires.ftc.robotlib.state.ServoState;

/**
 * A ModifiedServo is a Servo with extended functionality.
 * This class can be used to create special types of Servos.
 * This class implements Servo so it can still be treated as one.
 */
public class ModifiedServo implements Servo {
    protected Servo servo;

    public ModifiedServo (Servo servo) { this.servo = servo; }

    @Override
    public ServoController getController() {
        return servo.getController();
    }

    @Override
    public int getPortNumber() {
        return servo.getPortNumber();
    }

    @Override
    public void setDirection(Direction direction) {
        servo.setDirection(direction);
    }

    @Override
    public Direction getDirection() {
        return servo.getDirection();
    }

    @Override
    public void setPosition(double position) {
        servo.setPosition(position);
    }

    public void setPosition(ServoState state) {
        this.setPosition(state.getPosition());
    }

    @Override
    public double getPosition() {
        return servo.getPosition();
    }

    @Override
    public void scaleRange(double min, double max) {
        servo.scaleRange(min, max);
    }

    @Override
    public Manufacturer getManufacturer() {
        return servo.getManufacturer();
    }

    @Override
    public String getDeviceName() {
        return servo.getDeviceName();
    }

    @Override
    public String getConnectionInfo() {
        return servo.getConnectionInfo();
    }

    @Override
    public int getVersion() {
        return servo.getVersion();
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {
        servo.resetDeviceConfigurationForOpMode();
    }

    @Override
    public void close() {
        servo.close();
    }
}

