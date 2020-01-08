package org.firstinspires.ftc.teamcode.Skystone.MotionProfiler;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Skystone.MathFunctions;
import org.firstinspires.ftc.teamcode.Skystone.Robot;

import java.util.Vector;

import static org.firstinspires.ftc.teamcode.Skystone.MathFunctions.angleWrap;
import static org.firstinspires.ftc.teamcode.Skystone.MathFunctions.lineCircleIntersection;

public class PurePursuit {
    Robot robot;

    public PurePursuit(Robot robot){
        this.robot = robot;
    }

    public boolean followCurve(double[][] points, double followAngle, double followDistance, double angleLockRadians, double angleLockDistance) {

        PathPoints path = new PathPoints(points, followDistance);

        Vector<CurvePoint> allPoints = path.targetPoints;

        // for angle locking
        Point secondToLastPoint = new Point(points[points.length - 2][0], points[points.length - 2][1]);
        Point lastPoint = new Point(points[points.length - 1][0], points[points.length - 1][1]);
        double lastAngle = Math.atan2(lastPoint.y - secondToLastPoint.y, lastPoint.x - secondToLastPoint.x);

        Vector<CurvePoint> pathExtended = (Vector<CurvePoint>) allPoints.clone();

        PointWithIndex distanceAlongPath = distanceAlongPath(allPoints, robot.getRobotPos());
        int currFollowIndex = distanceAlongPath.index + 1;

        CurvePoint followMe = getFollowPointPath(pathExtended, robot.getRobotPos(), allPoints.get(currFollowIndex).followDistance);

        pathExtended.set(pathExtended.size() - 1, extendLine(allPoints.get(allPoints.size() - 2), allPoints.get(allPoints.size() - 1), allPoints.get(allPoints.size() - 1).pointLength * 1.5));

        double distanceToEnd = Math.hypot(distanceAlongPath.x - allPoints.get(allPoints.size() - 1).x, distanceAlongPath.y - allPoints.get(allPoints.size() - 1).y);

        if (distanceToEnd <= followMe.followDistance + 15 || Math.hypot(robot.getRobotPos().x - allPoints.get(allPoints.size() - 1).x, robot.getRobotPos().y - allPoints.get(allPoints.size() - 1).y) < followMe.followDistance + 15) {
            followMe.setPoint(allPoints.get(allPoints.size() - 1).toPoint());
        }

        // check if finished pp
        if ((distanceToEnd < 1)) {
            return false;
        }

        // get the movements
        robot.goToPoint(followMe.x, followMe.y, followMe.moveSpeed, followMe.turnSpeed, followAngle, true);

        // angle lock
        if (distanceToEnd < angleLockDistance) {
            robot.getTelemetry().addLine("TURNMOVEMENT: " + robot.getTurnMovement());
            robot.getTelemetry().addLine("go: " + robot.getAnglePos());
            robot.getTelemetry().update();
            if (Math.abs(robot.getAnglePos() - angleLockRadians) < Math.toRadians(2)) {
                followAngle = 0;
            } else {
                robot.setTurnMovement((angleLockRadians - angleWrap(robot.getAnglePos() + 2 * Math.PI)) / Math.PI);
            }
        }

        // get the motor powers
        robot.applyMove();
        return true;
    }

    public void moveFollowCurve(double[][] points, double followAngle, double followDistance, double angleLockRadians, double angleLockDistance) {
        //pathDistance = Math.hypot(points.get(points.size() - 1).x, points.get(points.size() - 1).y);
        while (robot.getLinearOpMode().opModeIsActive()) {

            // if followCurve returns false then it is ready to stop
            // else, it moves

            if (!followCurve(points, followAngle, followDistance, angleLockRadians, angleLockDistance)) {
                robot.brakeRobot();
                return;
            }
        }
    }

    public boolean followCurve(double[][] points, double followAngle, double followDistance) {

        PathPoints path = new PathPoints(points, followDistance);

        Vector<CurvePoint> allPoints = path.targetPoints;

        Vector<CurvePoint> pathExtended = (Vector<CurvePoint>) allPoints.clone();

        PointWithIndex distanceAlongPath = distanceAlongPath(allPoints, robot.getRobotPos());
        int currFollowIndex = distanceAlongPath.index + 1;

        CurvePoint followMe = getFollowPointPath(pathExtended, robot.getRobotPos(), allPoints.get(currFollowIndex).followDistance);

        pathExtended.set(pathExtended.size() - 1, extendLine(allPoints.get(allPoints.size() - 2), allPoints.get(allPoints.size() - 1), allPoints.get(allPoints.size() - 1).pointLength * 1.5));

        double distanceToEnd = Math.hypot(distanceAlongPath.x - allPoints.get(allPoints.size() - 1).x, distanceAlongPath.y - allPoints.get(allPoints.size() - 1).y);

        if (distanceToEnd <= followMe.followDistance + 15 || Math.hypot(robot.getRobotPos().x - allPoints.get(allPoints.size() - 1).x, robot.getRobotPos().y - allPoints.get(allPoints.size() - 1).y) < followMe.followDistance + 15) {
            followMe.setPoint(allPoints.get(allPoints.size() - 1).toPoint());
        }

        double decelerationScaleFactor = Range.clip(distanceToEnd / 12, -1, 1);

        robot.goToPoint(followMe.x, followMe.y, followMe.moveSpeed * decelerationScaleFactor, followMe.turnSpeed * decelerationScaleFactor, followAngle, true);
        if ((distanceToEnd < 0.5)) {
            return false;
        }

        robot.applyMove();
        return true;
    }

    public void moveFollowCurve(double[][] points, double followAngle, double followDistance) {
        //pathDistance = Math.hypot(points.get(points.size() - 1).x, points.get(points.size() - 1).y);
        while (robot.getLinearOpMode().opModeIsActive()) {

            // if followCurve returns false then it is ready to stop
            // else, it moves

            if (!followCurve(points, followAngle, followDistance)) {
                robot.brakeRobot();
                return;
            }
        }
    }

    public PointWithIndex distanceAlongPath(Vector<CurvePoint> pathPoints, Point robot) {
        double closestDistance = Integer.MAX_VALUE;

        int closestDistanceIndex = 0;

        Point distanceAlongLine = new Point();

        for (int i = 0; i < pathPoints.size() - 1; i++) {
            CurvePoint firstPoint = pathPoints.get(i);
            CurvePoint secondPoint = pathPoints.get(i + 1);

            Point currentDistanceAlongLine = distanceAlongLine(firstPoint, secondPoint, robot);

            double distanceToClip = Math.hypot(robot.x - currentDistanceAlongLine.x, robot.y - currentDistanceAlongLine.y);

            if (distanceToClip < closestDistance) {
                closestDistance = distanceToClip;
                closestDistanceIndex = i;
                distanceAlongLine = currentDistanceAlongLine;
            }
        }
        //return the three things
        return new PointWithIndex(distanceAlongLine.x, distanceAlongLine.y, closestDistanceIndex);//now return the closestDistanceIndex
    }

    public class PointWithIndex {
        private double x;
        private double y;
        private int index;

        public PointWithIndex(double xPos, double yPos, int index) {
            this.x = xPos;
            this.y = yPos;
            this.index = index;
        }
    }

    public static Point distanceAlongLine(CurvePoint line1, CurvePoint line2, Point robot) {
        if (line1.x == line2.x) {
            line1.x = line2.x + 0.01;
        }
        if (line1.y == line2.y) {
            line1.y = line2.y + 0.01;
        }

        //calculate the slope of the line
        double m1 = (line2.y - line1.y) / (line2.x - line1.x);
        //calculate the slope perpendicular to this line
        double m2 = (line1.x - line2.x) / (line2.y - line1.y);

        //clip the robot's position to be on the line
        double xAlongLine = ((-m2 * robot.x) + robot.y + (m1 * line1.x) - line1.y) / (m1 - m2);
        double yAlongLine = (m1 * (xAlongLine - line1.x)) + line1.y;
        return new Point(xAlongLine, yAlongLine);
    }

    public CurvePoint extendLine(CurvePoint firstPoint, CurvePoint secondPoint, double distance) {
        double lineAngle = Math.atan2(secondPoint.y - firstPoint.y, secondPoint.x - firstPoint.x);
        double lineLength = Math.hypot(secondPoint.x - firstPoint.x, secondPoint.y - firstPoint.y);
        //extend the line by 1.5 pointLengths
        double extendedLineLength = lineLength + distance;

        CurvePoint extended = new CurvePoint(secondPoint);
        extended.x = Math.cos(lineAngle) * extendedLineLength + firstPoint.x;
        extended.y = Math.sin(lineAngle) * extendedLineLength + firstPoint.y;
        return extended;
    }

    private CurvePoint getFollowPointPath(Vector<CurvePoint> pathPoints, Point robotLocation, double followRadius) {
        CurvePoint followMe = new CurvePoint(pathPoints.get(0));

        for (int i = 0; i < pathPoints.size() - 1; i++) {

            CurvePoint startLine = pathPoints.get(i);
            CurvePoint endLine = pathPoints.get(i + 1);

            Vector<Point> intersections = lineCircleIntersection(robotLocation, followRadius, startLine.toPoint(), endLine.toPoint());

            double closestAngle = Double.MAX_VALUE;

            for (Point intersectionPoint : intersections) {
                double angle = Math.atan2(intersectionPoint.y - robot.getRobotPos().y, intersectionPoint.x - robot.getRobotPos().x);
                double deltaAngle = Math.abs(MathFunctions.angleWrap(angle - robot.getAnglePos()));

                if (deltaAngle < closestAngle) {
                    closestAngle = deltaAngle;
                    followMe.setPoint(intersectionPoint);
                }
            }
        }
        return followMe;
    }

}
