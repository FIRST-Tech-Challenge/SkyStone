package org.firstinspires.ftc.teamcode.subsystems.dunk;

public interface Dunk
{
    void liftUp() throws InterruptedException;
    void liftDown() throws InterruptedException;

    //Dunk
    void dunk() throws InterruptedException;
    void dunkDown() throws InterruptedException;
    void dunkHold() throws InterruptedException;
}
