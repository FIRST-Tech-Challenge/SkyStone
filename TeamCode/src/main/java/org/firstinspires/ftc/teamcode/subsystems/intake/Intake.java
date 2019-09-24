package org.firstinspires.ftc.teamcode.subsystems.intake;

public interface Intake {

    void start();
    void stop();
    void reverse();
    void spoolOut(int degrees) throws InterruptedException;
    void spoolIn(int degrees) throws InterruptedException;
    void spoolInFully();
}
