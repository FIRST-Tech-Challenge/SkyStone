package org.eastsideprep.murderbot;

/**
 * Created by gmein on 2/6/2018.
 */


public class SingleShot implements Runnable {
    MBHardware robot;
    int ms;

    SingleShot(MBHardware robot, int ms) {
        this.robot = robot;
        this.ms = ms;
    }

    @Override
    public void run() {
        synchronized (robot.state) {
            robot.state.firing = true;
            robot.state.firectrl = true;
        }

        robot.waitForTick(ms);

        synchronized (robot.state) {
            robot.state.firing = false;
            robot.state.firectrl = true;
        }
    }
}
