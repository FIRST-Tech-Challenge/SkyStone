package org.eastsideprep.murderbot;

/**
 * Created by gmein on 2/6/2018.
 */


public class BurstShot implements Runnable {
    MBHardware robot;

    BurstShot(MBHardware robot) {
        this.robot = robot;
    }

    private void fireSynchronous(int ms) {
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

    @Override
    public void run() {

        int interval = 100;

        robot.state.orientationSweepDelta = -0.5;

        for (int i = 0; i < 8; i++) {
            fireSynchronous(interval);
            if (i == 2 || i == 6) {
                robot.state.orientationSweepDelta *= -1;
            }
            if (i < 7) {
                robot.waitForTick(interval);
            }
        }
        synchronized (robot.state) {
            robot.state.firing = false;
            robot.state.firectrl = true;
        }
        robot.state.orientationSweepDelta = 0;
    }
}
