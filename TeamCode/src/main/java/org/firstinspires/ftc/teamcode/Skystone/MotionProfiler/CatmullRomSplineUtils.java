package org.firstinspires.ftc.teamcode.Skystone.MotionProfiler;

import org.firstinspires.ftc.teamcode.Skystone.Robot;

public class CatmullRomSplineUtils {
    public static Point[] subdividePoints(Point[] points, int subdivisions, Robot robot) {
        assert points != null;
        assert points.length >= 3;
        //TODO add robot position as first point

        points[0] = new Point(robot.getRobotPos().x, robot.getRobotPos().y);

        Point[] subdividedPoints = new Point[((points.length - 1) * subdivisions) + 1];

        double increments = 1.0 / (double) subdivisions;

        for (int i = 0; i < points.length - 1; i++) {
            Point p0 = i == 0 ? points[i] : points[i - 1];
            Point p1 = points[i];
            Point p2 = points[i + 1];
            Point p3 = (i + 2 == points.length) ? points[i + 1] : points[i + 2];

            CatmullRomSpline2D crs = new CatmullRomSpline2D(p0, p1, p2, p3);

            for (int j = 0; j <= subdivisions; j++) {
                subdividedPoints[(i * subdivisions) + j] = crs.q(j * increments);
            }
        }

        return subdividedPoints;
    }

    public static Point[] generateSpline(Point[] points, int subDivisions, Robot robot) {
        return subdividePoints(points, subDivisions, robot);
    }
}
