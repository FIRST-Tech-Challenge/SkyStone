package org.firstinspires.ftc.teamcode.common.math;

public class TimePose extends Pose {
    public long time;

    public TimePose(Pose p) {
        super(p.x, p.y, p.heading);
        this.time = System.currentTimeMillis();
    }

    public TimePose(Pose p, long time) {
        super(p.x, p.y, p.heading);
        this.time = time;
    }

    public TimePose(double x, double y, double heading, long time) {
        super(x, y, heading);
        this.time = time;
    }
}