package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.DeviceMap;

public final class MecanumDriver implements IDriver {
    private DeviceMap map;

    public MecanumDriver() {
        this.map = map;
    }

    /**
     * Move an unspecified amount of distance
     * @param direction
     * @param power
     */
    @Override
    public void move(Direction direction, double power) {
        map.getLeftTop().setPower(direction.getLeftTop() * power);
        map.getRightBottom().setPower(direction.getRightBottom() * power);
        map.getRightTop().setPower(direction.getRightTop() * power);
        map.getLeftBottom().setPower(direction.getLeftBottom() * power);
    }

    /**
     * encoder drive
     * @param direction
     * @param power
     * @param inches
     */
    @Override
    public void move(Direction direction, double power, double inches) {
        //TODO
    }

    @Override
    public void stop() {
        for(DcMotor motor : map.getMotors())
            motor.setPower(0);
    }
}
