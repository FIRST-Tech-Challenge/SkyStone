package org.firstinspires.ftc.teamcode.Skystone.MotionProfiler;

@Deprecated
public class CatmullRomSplineGenerator {
    private double p0, p1, p2, p3;

    public CatmullRomSplineGenerator(double p0, double p1, double p2, double p3) {
        this.p0 = p0;
        this.p1 = p1;
        this.p2 = p2;
        this.p3 = p3;
    }

    public double q(double t) {
        return 0.5 * ((2 * p1) + (p2 - p0) * t + (2 * p0 - 5 * p1 + 4 * p2 - p3) * Math.pow(t, 2) + (3 * p1 - p0 - 3 * p2 + p3) * Math.pow(t, 3));
    }
}

@Deprecated
class CatmullRomSpline2D {
    private CatmullRomSplineGenerator splineXVals, splineYVals;

    public CatmullRomSpline2D(Point p0, Point p1, Point p2, Point p3) {
        assert p0 != null : "p0 cannot be null";
        assert p1 != null : "p1 cannot be null";
        assert p2 != null : "p2 cannot be null";
        assert p3 != null : "p3 cannot be null";

        splineXVals = new CatmullRomSplineGenerator(p0.x, p1.x, p2.x, p3.x);
        splineYVals = new CatmullRomSplineGenerator(p0.y, p1.y, p2.y, p3.y);
    }

    public Point q(double t) {
        return new Point(splineXVals.q(t), splineYVals.q(t));
    }
}
