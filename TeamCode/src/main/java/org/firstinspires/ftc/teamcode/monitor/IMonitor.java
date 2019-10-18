package org.firstinspires.ftc.teamcode.monitor;

/**
 * This is where sensors will be managed and used
 */
public interface IMonitor extends Runnable {
    void start();
    void stop();
}
