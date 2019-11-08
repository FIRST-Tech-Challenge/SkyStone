package org.firstinspires.ftc.teamcode;

public interface RobotControl {
    void prepare();
    void execute();
    void cleanUp();

    boolean isDone();
}
