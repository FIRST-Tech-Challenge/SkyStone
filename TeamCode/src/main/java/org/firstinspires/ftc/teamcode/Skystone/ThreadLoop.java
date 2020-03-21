package org.firstinspires.ftc.teamcode.Skystone;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Skystone.Robot;

@Deprecated
/**
 * Calls robot.update() repeatedly on a separate thread until stopped
 */
public class ThreadLoop extends Thread {

    private volatile boolean terminateRequested = false;
    private Robot robot;
    private LinearOpMode linearOpMode;

    public ThreadLoop(Robot robot, LinearOpMode linearOpMode) {
        super("threadLoop");
        this.robot = robot;
        this.linearOpMode = linearOpMode;
    }

    @Override
    public void run() {
        terminateRequested = false;
        while (!terminateRequested && !linearOpMode.isStopRequested()) {
            robot.update();
        }
    }

    public void terminate() {
        terminateRequested = true;
    }

    public boolean isTerminateRequested() {
        return terminateRequested;
    }
}