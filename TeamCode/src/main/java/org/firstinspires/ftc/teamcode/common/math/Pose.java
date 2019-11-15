package org.firstinspires.ftc.teamcode.common.math;

public class Pose extends Point implements Cloneable {
    public double heading;

    public Pose(double x, double y, double heading) {
        super(x, y);
        this.heading = heading;
    }

    public Pose(Point p, double heading) {
        this(p.x, p.y, heading);
    }

    public Pose add(Pose p2) {
        return new Pose(x + p2.x, y + p2.y, heading + p2.heading);
    }
    public Pose multiply(Pose p2) {
        return new Pose(x * p2.x, y * p2.y, heading * p2.heading);
    }
    public Pose minus(Pose p2) {
        return new Pose(x - p2.x, y - p2.y, heading - p2.heading);
    }
    public Pose scale(double d) {return new Pose(x * d, y * d, heading * d);}
    public void clampAbs(Pose p2) {
        x = Math.copySign(minAbs(x, p2.x), x);
        y = Math.copySign(minAbs(y, p2.y), y);
        heading = Math.copySign(minAbs(heading, p2.heading), heading);
    }

    public void applyFriction(Pose friction) {
        x = reduceUpToZero(x, friction.x);
        y = reduceUpToZero(y, friction.y);
        heading = reduceUpToZero(heading, friction.heading);
    }

    private double reduceUpToZero(double d, double reduction) {
        return d - minAbs(d, Math.copySign(reduction, d));
    }

    private double minAbs(double a, double b) {
        return Math.abs(a) < Math.abs(b) ? a : b;
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;
        if (!super.equals(o)) return false;
        Pose pose = (Pose) o;
        return MathUtil.approxEquals(pose.heading, heading);
    }

    @Override
    public String toString() {
        return String.format("{x: %.3f, y: %.3f, θ: %.3f}", x, y, heading);
    }

    @Override
    public Pose clone() {
        return new Pose(x, y, heading);
    }
}