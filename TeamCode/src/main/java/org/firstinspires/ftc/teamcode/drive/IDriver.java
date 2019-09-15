package org.firstinspires.ftc.teamcode.drive;

import org.firstinspires.ftc.teamcode.drive.Direction;

public interface IDriver {
    void move(Direction direction, double power);
    void move(Direction direction, double power, double inches);

    void stop();
}
