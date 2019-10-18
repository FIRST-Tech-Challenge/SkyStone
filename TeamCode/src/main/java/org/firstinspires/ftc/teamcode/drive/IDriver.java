package org.firstinspires.ftc.teamcode.drive;

import org.firstinspires.ftc.teamcode.drive.Direction;

import java.util.function.Consumer;

public interface IDriver {
    void move(Direction direction, double power);
    void move(Direction direction, double power, double inches);

    void turn(double power, double angle, IActive stopRequested);
    void stop();
}
