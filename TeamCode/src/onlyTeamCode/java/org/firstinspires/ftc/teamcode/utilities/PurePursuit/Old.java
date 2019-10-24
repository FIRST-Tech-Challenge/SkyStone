//package org.firstinspires.ftc.teamcode.utilities.PurePursuit;
//
//import android.speech.tts.TextToSpeech;
//import org.firstinspires.ftc.teamcode.AllConstants;
//import org.firstinspires.ftc.teamcode.OtherStuff.Point;
//
//import java.util.ArrayList;
////Largely taken from: file:///C:/Users/MOE%20365%20FTC/Downloads/be0e06de00e07db66f97686505c3f4dde2e332dc.pdf
////https://www.chiefdelphi.com/t/paper-implementation-of-the-adaptive-pure-pursuit-controller/166552
//
//public class MOEPurePursuit {
//    public double TRACK_WIDTH;
//    public double TURNING_CONSTANT;
//    public double PATH_SPACING;
//    public double LOOKAHEAD_DISTANCE;
//    private double
//            lastLeftVelocity;
//    private double lastRightVelocity;
//    private MOEBot robot;
//    //    private int lastKnownPointIndex;
//
//    public MOEPurePursuit(MOEBot robot) {
//        this.robot = robot;
//        lastLeftVelocity = 0;
//        lastRightVelocity = 0;
//
//        //        robot.prefs.get
//    }
//
//    public static Point getLookaheadPoint(Point startPoint, Point endPoint, Point currentPosition, double lookaheadDistance, boolean onLastSegment) {
//        double progress = getCircleLineIntersection(startPoint, endPoint, currentPosition, lookaheadDistance);
//        if (Double.isNaN(progress)) {
//            return null;
//        }
//        if (onLastSegment) {
//            //TODO: add last segment code
//        }
//        Point intersectVector = Point.sub(endPoint, startPoint);
//        Point vectorSegment = Point.multiply(intersectVector, progress);
//        return Point.add(startPoint, vectorSegment);
//    }
//
//    private static double getCircleLineIntersection(Point pointA, Point pointB, Point currentPosition, double lookaheadDistance) {
//        Point d = Point.sub(pointB, pointA);
//        Point f = Point.sub(pointA, currentPosition);
//
//        double a = d.dot(d);
//        double b = 2 * f.dot(d);
//        double c = f.dot(f) - Math.pow(lookaheadDistance, 2);
//        double discriminant = Math.pow(b, 2) - (4 * a * c);
//
//        if (discriminant < 0) {
//            System.out.println("no intersection");
//            //            return Optional.empty();
//            return Double.NaN;
//        } else {
//            discriminant = Math.sqrt(discriminant);
//            double t1 = (-b - discriminant) / (2 * a);
//            double t2 = (-b + discriminant) / (2 * a);
//
//            if (t1 >= 0 && t1 <= 1) {
//                System.out.println("intersetion: " + t1);
//
//                //                System.out.println();
//                return t1;
//                //                return Optional.of(t1);
//            }
//            if (t2 >= 0 && t2 <= 1) {
//                System.out.println("intersetion: " + t2);
//
//                return t2;
//                //                return Optional.of(t2);
//            }
//            System.out.println("nope");
//
//        }
//
//        //        return Optional.empty();
//        return Double.NaN;
//    }
//
//    public static double getSignedCurvatureFromLookaheadPoint(Point lookahead, Point currPos,
//                                                              double heading, double lookaheadDistance) {
//        if (heading == 90 || heading == 270) {
//            heading += 0.0001;
//        }
//        heading = Math.toRadians(heading);
//
//        double a = -Math.tan(heading);
//        double b = 1;
//        double c = (Math.tan(heading) * currPos.x) - currPos.y;
//        double x = Math.abs(a * lookahead.x + b * lookahead.y + c) / Math.sqrt(Math.pow(a, 2) + Math.pow(b, 2));
//        double cross = (Math.sin(heading) * (lookahead.x - currPos.x)) - (Math.cos(heading) * (lookahead.y - currPos.y));
//        double side = cross > 0 ? 1 : -1;
//        double curvature = (2 * x) / (Math.pow(lookaheadDistance, 2));
//        return curvature * side;
//    }
//
//    public static double getCurvatureOfPoints(Point leftPoint, Point centerPoint, Point rightPoint) {
//        if (centerPoint.x == 90) {
//            centerPoint.x += 0.0001;
//        } //Fixing for DivideByZero error
//
//        double k_1 = 0.5 * (centerPoint.x * centerPoint.x + centerPoint.y * centerPoint.y - leftPoint.x * leftPoint.x - leftPoint.y * leftPoint.y)
//                / (centerPoint.x - leftPoint.x);
//        double k_2 = (centerPoint.y - leftPoint.y) / (centerPoint.x - leftPoint.x);
//        double b = 0.5 * (leftPoint.x * leftPoint.x - 2 * leftPoint.x * k_1 + leftPoint.y * leftPoint.y - rightPoint.x * rightPoint.x + 2 * rightPoint.x * k_1 - rightPoint.y * rightPoint.y) /
//                (rightPoint.x * k_2 - rightPoint.y + leftPoint.y - leftPoint.x * k_2);
//        double a = k_1 - k_2 * b;
//        double r = Math.sqrt(Math.pow(centerPoint.x - a, 2) + Math.pow(centerPoint.y - b, 2));
//        Double curvature = 1 / r;
//        if (curvature.isNaN()) {
//            return 0;
//        }
//        return curvature;
//    }
//
//    public static ArrayList<Point> injectPoints(ArrayList<Point> pathing, double spacing) {
//        ArrayList<Point> newPathing = new ArrayList<>();
//
//        for (int i = 0; i < pathing.size() - 1; i++) {
//            MOEVector vector = new MOEVector(
//                    pathing.get(i + 1).x - pathing.get(i).x,
//                    pathing.get(i + 1).y - pathing.get(i).y
//            );
//
//            int numberOfPointsThatFit = (int) Math.ceil(vector.getMagnitude() / spacing);
//            vector.normalize();
//            vector.multiplyBy(spacing);
//
//            Point originalPoint = new Point(
//                    pathing.get(i).x,
//                    pathing.get(i).y
//            );
//            originalPoint.setCriticalPoint(true);
//            newPathing.add(originalPoint);
//
//            for (int a = 1; a < numberOfPointsThatFit; a++) {
//                newPathing.add(new Point(
//                        pathing.get(i).x + vector.getX() * a,
//                        pathing.get(i).y + vector.getY() * a
//                ));
//            }
//        }
//
//        Point lastPoint = pathing.get(pathing.size() - 1);
//        lastPoint.setCriticalPoint(true);
//        newPathing.add(lastPoint);
//
//        return newPathing;
//    }
//
//    public static ArrayList<Point> smoothPoints(ArrayList<Point> points, double a, double b, double tolerance) {
//        ArrayList<Point> newPath = new ArrayList<>(points.size());
//
//        for (Point foo : points) {
//            newPath.add(new Point(foo.x, foo.y));
//        }
//
//        double change = tolerance;
//        while (change >= tolerance) {
//            change = 0.0;
//            for (int i = 1; i < points.size() - 1; i++) {
//                double aux = newPath.get(i).x;
//                newPath.get(i).x += a * (points.get(i).x - newPath.get(i).x) + b *
//                        (newPath.get(i - 1).x + newPath.get(i + 1).x - (2.0 * newPath.get(i).x));
//                change += Math.abs(aux - newPath.get(i).x);
//
//                aux = newPath.get(i).y;
//                newPath.get(i).y += a * (points.get(i).y - newPath.get(i).y) + b *
//                        (newPath.get(i - 1).y + newPath.get(i + 1).y - (2.0 * newPath.get(i).y));
//                change += Math.abs(aux - newPath.get(i).y);
//            }
//        }
//        points = newPath;
//        return points;
//    }
//
//    public static ArrayList<Point> setPrefixDistanceSums(ArrayList<Point> points) {
//        double sum = 0;
//        for (int i = 0; i < points.size() - 1; i++) {
//            points.get(i).prefixDistance = sum;
//            sum += Point.distanceBetweenPoints(points.get(i), points.get(i + 1));
//        }
//        points.get(points.size() - 1).prefixDistance = sum;
//
//        return points;
//    }
//
//    public static ArrayList<Point> setMaxVelocities(ArrayList<Point> points, double turning_constant) {
//        for (int i = 1; i < points.size() - 1; i++) {
//            double curvature = getCurvatureOfPoints(points.get(i - 1), points.get(i), points.get(i + 1));
//            System.out.println("curvature; " + curvature);
//            points.get(i).velocity = Math.min(AllConstants.PurePursuit.OVERALL_MAX_VELOCITY, turning_constant / curvature);
//        }
//        points.get(0).velocity = AllConstants.PurePursuit.OVERALL_MAX_VELOCITY;
//        points.get(points.size() - 1).velocity = 0;
//        return points;
//    }
//
//    public static ArrayList<Point> smoothVelocities(ArrayList<Point> points) {
//        double velocity = 0;
//        points.get(points.size() - 1).velocity = 0;
//        for (int i = points.size() - 2; i >= 0; i--) {
//            double nextVelocity = points.get(i + 1).velocity;
//            double a = AllConstants.PurePursuit.OVERALL_MAX_VELOCITY;
//            double distance = Point.distanceBetweenPoints(points.get(i), points.get(i + 1));
//            double newVelocity = Math.sqrt(Math.pow(nextVelocity, 2) + (2 * a * distance));
//            velocity = Math.min(points.get(i).velocity, newVelocity);
//            points.get(i).velocity = velocity;
//            //            sum += Point.distanceBetweenPoints(points.get(i), points.get(i + 1));
//            //            points.get(points.size() - 1).velocity = sum;
//
//        }
//
//        return points;
//    }
//
//    public static int getClosestPointIndex(ArrayList<Point> points, int lastKnownPointIndex, Point currentPoint,
//                                           int lookBack, int lookForward) {
//        int closestPointIndex = 0;
//        double closestDistance = Double.MAX_VALUE;
//        for (int i = lastKnownPointIndex - lookBack; i < lastKnownPointIndex + lookForward + 1; i++) {
//            if (i >= 0 && i < points.size()) {
//                double dist = Point.distanceBetweenPoints(points.get(i), currentPoint);
//                if (dist < closestDistance) {
//                    closestDistance = dist;
//                    closestPointIndex = i;
//                }
//            }
//        }
//        return closestPointIndex;
//    }
//
//    public static void main(String[] args) {
//        ArrayList<Point> pathing = new ArrayList<Point>();
//        pathing.add(new Point(0, 0));
//        pathing.add(new Point(0, 100));
//        pathing.add(new Point(100, 100));
//
//        ArrayList<Point> pursuitPathing = injectPoints(pathing, AllConstants.PurePursuit.PATH_SPACING);
//        pursuitPathing = smoothPoints(pursuitPathing, AllConstants.PurePursuit.SMOOTHING_A,
//                AllConstants.PurePursuit.SMOOTHING_B, AllConstants.PurePursuit.SMOOTHING_TOLERANCE);
//        for (Point point : pursuitPathing) {
//            //            System.out.println(point.x + "\t" + point.y);
//        }
//        setMaxVelocities(pursuitPathing, 2);
//        smoothVelocities(pursuitPathing);
//
//        for (int i = 0; i < pursuitPathing.size(); i++) {
//            //            System.out.println(pursuitPathing.get(i));
//            System.out.println(i + "\t" + pursuitPathing.get(i).velocity);
//        }
//        //        ArrayList<Point> path = new ArrayList<Point>();
//        //        path.add(new Point(0, 0));
//        //        path.add(new Point(0, 50));
//        //
//        //        double curvature = getSignedCurvatureFromLookaheadPoint(new Point(0, 0), new Point(10, 0), 0, AllConstants.PurePursuit.LOOKAHEAD_DISTANCE);
//        //        System.out.println(curvature);
//        //        //path.add(new)
//        //        //        path.add(new Point(18, 0));
//        //        //        path.add(new Point(27, 33));
//        //        //        path.add(new Point(46, 68));
//        //        //            path.add(new Point(100, 100));
//        //
//        //                MOEPurePursuit purePursuit = new MOEPurePursuit(null);
//        //                path = injectPoints(path, 6);
//        //                path = smoothPoints(path, AllConstants.PurePursuit.SMOOTHING_A, AllConstants.PurePursuit.SMOOTHING_B, AllConstants.PurePursuit.SMOOTHING_TOLERANCE);
//        //                setMaxVelocities(path, AllConstants.PurePursuit.TURNING_CONSTANT);
//        //                smoothVelocities(path);
//        //
//        //
//        //                for (Point point : path) {
//        //                    System.out.println(point.velocity);
//        //                }
//        //        purePursuit.runPathing(path);
//    }
//
//    //    public static void main(String[] args) {
//    //        Point a = new Point(62.966, 178.61);
//    //        Point b = new Point(67.041699130413718, 182.69554467587588);
//    //        Point current = new Point(-58.799999237060547, 72.800003051757812);
//    //        double radius = 12;
//    //        System.out.println(getLookaheadPoint(a, b, current, radius,false));
//    //    }
//
//    //    public void runPathing(ArrayList<Point> pathing) {
//    //        pathing.add(0, robot.odometry.getOdometryPosition());
//    //
//    //        ArrayList<Point> pursuitPathing = injectPoints(pathing, AllConstants.PurePursuit.PATH_SPACING);
//    //        smoothPoints(pursuitPathing, AllConstants.PurePursuit.SMOOTHING_A,
//    //                     AllConstants.PurePursuit.SMOOTHING_B, AllConstants.PurePursuit.SMOOTHING_TOLERANCE);
//    //        //        pursuitPathing = PurePursuitMethods.setPrefixDistanceSums(pursuitPathing,AllConstants.PurePursuit.TURNING_CONSTANT);
//    //        setMaxVelocities(pursuitPathing, AllConstants.PurePursuit.TURNING_CONSTANT);
//    //        smoothVelocities(pursuitPathing);
//    //
//    //        robot.telemetry.addData("smoothed velocities!", "");
//    //            robot.telemetry.update();
//    //
//    //        //        protected void waitMilliseconds(double ms) {
//    //        robot.opMode.waitMilliseconds(2000);
//    //
//    //        int lastKnownPointIndex = 0;
//    //        Point closestPoint;
//    //        do {
//    //            lastKnownPointIndex = getClosestPointIndex(pursuitPathing, lastKnownPointIndex,
//    //                                                       robot.odometry.getOdometryPosition(), 1, 2);
//    //            closestPoint = pursuitPathing.get(lastKnownPointIndex);
//    //
//    //            Point currentPosition = robot.odometry.getOdometryPosition();
//    //            double heading = robot.gyro.getAngle();
//    //
//    //            Point lookaheadPoint = getLookaheadPointFromPathing(pursuitPathing, lastKnownPointIndex, currentPosition);
//    //            double curvature = getSignedCurvatureFromLookaheadPoint(lookaheadPoint, currentPosition, heading, AllConstants.PurePursuit.LOOKAHEAD_DISTANCE);
//    //
//    //            double leftWheelTargetVelocity = getLeftWheelTargetVelocity(closestPoint.velocity, curvature);
//    //            double rightWheelTargetVelocity = getRightWheelTargetVelocity(closestPoint.velocity, curvature);
//    //            leftWheelTargetVelocity = normalizeVelocity(leftWheelTargetVelocity);
//    //            rightWheelTargetVelocity = normalizeVelocity(rightWheelTargetVelocity);
//    //
//    //            //TODO: if (!path.isForward()) {
//    //            //            leftTargetVel = -leftTargetVel;
//    //            //            rightTargetVel = -rightTargetVel;
//    //
//    //            double leftFeedback = getWheelFeedbackVelocity(leftWheelTargetVelocity,
//    //                                                           robot.chassis.getAStarVelocity(robot.chassis.frontLeftMotor));
//    //            double rightFeedback = getWheelFeedbackVelocity(rightWheelTargetVelocity,
//    //                                                            robot.chassis.getAStarVelocity(robot.chassis.frontRightMotor));
//    //
//    //            double leftPower = leftWheelTargetVelocity + leftFeedback;
//    //            double rightPower = rightWheelTargetVelocity + rightFeedback;
//    //
//    //                robot.chassis.moveMotors(rightPower, leftPower, rightPower, leftPower, 0.6);
//    //
//    //            robot.telemetry.addData("coords: ", currentPosition.x + ", " + currentPosition.y);
//    //            robot.telemetry.addData("lp: ", lookaheadPoint.toString());
//    //
//    //            robot.telemetry.addData("left:", leftWheelTargetVelocity);
//    //            robot.telemetry.addData("right:", rightWheelTargetVelocity);
//    //            robot.telemetry.update();
//    //        } while (lastKnownPointIndex < pursuitPathing.size() - 1); //TODO: use robot x & robot y for more accurate ending
//    //    }
//
//    public static Point getLookaheadPointFromPathing(ArrayList<Point> pathing, int closestPointIndex, Point currentPosition) {
//        for (int i = closestPointIndex; i < pathing.size() - 1; i++) {
//            Point a = pathing.get(i);
//            Point b = pathing.get(i + 1);
//
//            Point lp = getLookaheadPoint(a, b, currentPosition, AllConstants.PurePursuit.LOOKAHEAD_DISTANCE, false);
//
//            if (lp != null) {
//                return lp;
//            }
//        }
//        return pathing.get(closestPointIndex);
//    }
//
//    //    @Deprecated
//    //    public void runPathingTemp(ArrayList<Point> pathing) {
//    //        pathing.add(0, robot.odometry.getOdometryPosition());
//    //
//    //        ArrayList<Point> pursuitPathing = injectPoints(pathing, AllConstants.PurePursuit.PATH_SPACING);
//    //        smoothPoints(pursuitPathing, AllConstants.PurePursuit.SMOOTHING_A,
//    //                AllConstants.PurePursuit.SMOOTHING_B, AllConstants.PurePursuit.SMOOTHING_TOLERANCE);
//    //        //        pursuitPathing = PurePursuitMethods.setPrefixDistanceSums(pursuitPathing,AllConstants.PurePursuit.TURNING_CONSTANT);
//    //        setMaxVelocities(pursuitPathing, AllConstants.PurePursuit.TURNING_CONSTANT);
//    //        smoothVelocities(pursuitPathing);
//    //        robot.gyro.setAutonOffset(-135);
//
//    //        robot.telemetry.addData("smoothed velocities!", "");
//    //
//    //        for (Point point : pursuitPathing) {
//    //            robot.telemetry.addData("X: " + point.x, " Y: " + point.y);
//    //        }
//    //
//    //        robot.telemetry.update();
//    //
//    //        int lastKnownPointIndex = 0;
//    //        Point closestPoint;
//    //        do {
//    //            lastKnownPointIndex = getClosestPointIndex(pursuitPathing, lastKnownPointIndex,
//    //                    robot.odometry.getOdometryPosition(), 1, 2);
//    //            closestPoint = pursuitPathing.get(lastKnownPointIndex);
//    //
//    //            Point currentPosition = robot.odometry.getOdometryPosition();
//    //            double heading = robot.gyro.getAngle();
//    //
//    //            Point lookaheadPoint = getLookaheadPointFromPathing(pursuitPathing, lastKnownPointIndex, currentPosition);
//    //            double curvature = getSignedCurvatureFromLookaheadPoint(lookaheadPoint, currentPosition, heading, AllConstants.PurePursuit.LOOKAHEAD_DISTANCE);
//    //            //
//    //            double leftWheelTargetVelocity = getLeftWheelTargetVelocity(closestPoint.velocity, curvature);
//    //            double rightWheelTargetVelocity = getRightWheelTargetVelocity(closestPoint.velocity, curvature);
//    //            //            leftWheelTargetVelocity = normalizeVelocity(leftWheelTargetVelocity);
//    //            //            rightWheelTargetVelocity = normalizeVelocity(rightWheelTargetVelocity);
//    //            double[] velocities = normalizeVelocities(leftWheelTargetVelocity, rightWheelTargetVelocity);
//    //            leftWheelTargetVelocity = velocities[0];
//    //            rightWheelTargetVelocity = velocities[1];
//    //            //robot.chassis.setVelocities(leftWheelTargetVelocity,rightWheelTargetVelocity);
//    //            //TODO: if (!path.isForward()) {
//    //            //            leftTargetVel = -leftTargetVel;
//    //            //            rightTargetVel = -rightTargetVel;
//    //
//    //            //            double leftFeedback = getWheelFeedbackVelocity(leftWheelTargetVelocity,
//    //            //                                                           robot.chassis.getAStarVelocity(robot.chassis.frontLeftMotor));
//    //            //            double rightFeedback = getWheelFeedbackVelocity(rightWheelTargetVelocity,
//    //            //                                                            robot.chassis.getAStarVelocity(robot.chassis.frontRightMotor));
//    //            double leftFeedback = 0;
//    //            double rightFeedback = 0;
//    //
//    //            double leftPower = leftWheelTargetVelocity + leftFeedback;
//    //            double rightPower = rightWheelTargetVelocity + rightFeedback;
//    //
//    //            robot.chassis.moveMotors(leftPower, rightPower, leftPower, rightPower, 0.8);
//    //            //robot.chassis.backLeftMotor
//    //            //            robot.telemetry.addData("coords: ", currentPosition.x + ", " + currentPosition.y);
//    //            //            robot.telemetry.addData("lp: ", lookaheadPoint.toString());
//    //            //
//    //            //            robot.telemetry.addData("left:", leftWheelTargetVelocity);
//    //            //            robot.telemetry.addData("right:", rightWheelTargetVelocity);
//    //            //            robot.telemetry.update();
//    //            double leftWheelTargetVelocity1 = getLeftWheelTargetVelocity(closestPoint.velocity, curvature);
//    //            double rightWheelTargetVelocity1 = getRightWheelTargetVelocity(closestPoint.velocity, curvature);
//    //
//    //            robot.showTelemetry(
//    //                    "location", currentPosition.simpleString(),
//    //                    "angle", "" + heading,
//    //                    "lookAhead", lookaheadPoint.simpleString(),
//    //                    "curvature", "" + curvature,
//    //                    "leftWheelTarget", "" + leftPower,
//    //                    "rightWheelTarget", "" + rightPower,
//    //                    "forward Velocity", (leftWheelTargetVelocity1 + rightWheelTargetVelocity1) / 2 + "",
//    //                    "angular Velocity", (leftWheelTargetVelocity1 - rightWheelTargetVelocity1) / AllConstants.PurePursuit.TRACK_WIDTH + ""
//    //            );
//    //        } while (lastKnownPointIndex < pursuitPathing.size() - 1 && robot.opMode.opModeIsActive()); //TODO: use robot x & robot y for more accurate ending
//    //    }
//
//    public Point getNextClosestPoint(int closestPointIndex, ArrayList<Point> pursuitPathing) {
//        if (closestPointIndex < pursuitPathing.size() - 1) {
//            for (int i = closestPointIndex + 1; i < pursuitPathing.size(); i++) {
//                if (pursuitPathing.get(i).isCriticalPoint()) {
//                    return pursuitPathing.get(i);
//                }
//            }
//        }
//
//        return pursuitPathing.get(closestPointIndex);
//    }
//
//    public void newRunPathing(ArrayList<Point> pathing) {
//        pathing.add(0, robot.odometry.getPosition());
//
//        ArrayList<Point> pursuitPathing = injectPoints(pathing, robot.prefs.PATH_SPACING);
//        pursuitPathing = smoothPoints(pursuitPathing, AllConstants.PurePursuit.SMOOTHING_A,
//                AllConstants.PurePursuit.SMOOTHING_B, AllConstants.PurePursuit.SMOOTHING_TOLERANCE);
//        setMaxVelocities(pursuitPathing, robot.prefs.getTurningConstant());
//        smoothVelocities(pursuitPathing);
//
//        robot.telemetry.addData("smoothed velocities!", "");
//
//        for (Point point : pursuitPathing) {
//            robot.telemetry.addData("X: " + point.x, " Y: " + point.y);
//        }
//
//        robot.telemetry.update();
//
//
//        int lastKnownPointIndex = 0;
//        Point closestPoint;
//        do {
//            Point currentPosition = robot.odometry.getPosition();
//
//            lastKnownPointIndex = getClosestPointIndex(pursuitPathing, lastKnownPointIndex,
//                    currentPosition, 1, 2);
//            closestPoint = pursuitPathing.get(lastKnownPointIndex);
//            //
//            if (closestPoint.isCriticalPoint() && lastKnownPointIndex != pursuitPathing.size() - 1) {
//                double angleToTurn = Point.angleBetweenPoints(currentPosition, getNextClosestPoint(lastKnownPointIndex, pursuitPathing));
//                //            robot.opMode.
//                if (robot.tts != null) {
//                    robot.tts.speak("turning", TextToSpeech.QUEUE_ADD, null, "");
//                }
//                robot.chassis.turnToDegrees(angleToTurn, true);
//                closestPoint.setCriticalPoint(false);
//                continue;
//            }
//
//            double heading = robot.gyro.getAutonAngle();
//
//            Point lookaheadPoint = getLookaheadPointFromPathing(pursuitPathing, lastKnownPointIndex, currentPosition);
//            double curvature = getSignedCurvatureFromLookaheadPoint(lookaheadPoint, currentPosition, heading, AllConstants.PurePursuit.LOOKAHEAD_DISTANCE);
//            //
//            int leftWheelTargetVelocity = (int) getLeftWheelTargetVelocity(closestPoint.velocity, curvature);
//            int rightWheelTargetVelocity = (int) getRightWheelTargetVelocity(closestPoint.velocity, curvature);
//            //            leftWheelTargetVelocity = normalizeVelocity(leftWheelTargetVelocity);
//            //            rightWheelTargetVelocity = normalizeVelocity(rightWheelTargetVelocity);
//            //            double[] velocities = normalizeVelocities(leftWheelTargetVelocity, rightWheelTargetVelocity);
//            //            leftWheelTargetVelocity = velocities[0];
//            //            rightWheelTargetVelocity = velocities[1];
//            //robot.chassis.setVelocities(leftWheelTargetVelocity,rightWheelTargetVelocity);
//            //TODO: if (!path.isForward()) {
//            //            leftTargetVel = -leftTargetVel;
//            //            rightTargetVel = -rightTargetVel;
//
//            //            double leftFeedback = getWheelFeedbackVelocity(leftWheelTargetVelocity,
//            //                                                           robot.chassis.getAStarVelocity(robot.chassis.frontLeftMotor));
//            //            double rightFeedback = getWheelFeedbackVelocity(rightWheelTargetVelocity,
//            //                                                            robot.chassis.getAStarVelocity(robot.chassis.frontRightMotor));
//            int leftFeedback = 0;
//            int rightFeedback = 0;
//
//            int leftPower = leftWheelTargetVelocity + leftFeedback;
//            int rightPower = rightWheelTargetVelocity + rightFeedback;
//            double scaleDown = 0.4;
//            int scaledLeftVelocity = (int) Math.round(leftPower * AllConstants.Tics.FORWARD_ASTAR_TICS * scaleDown);
//            int scaledRightVelocity = (int) Math.round(rightPower * AllConstants.Tics.FORWARD_ASTAR_TICS * scaleDown);
//            int scaledMaxVelocity = (int) Math.round(AllConstants.PurePursuit.OVERALL_MAX_VELOCITY * AllConstants.Tics.FORWARD_ASTAR_TICS * scaleDown);
//            //            robot.tts.speak(String.valueOf(scaledMaxVelocity), TextToSpeech.QUEUE_ADD, null, "");
//            //            scaledLeftVelocity = Math.min(scaledLeftVelocity,scaledMaxVelocity);
//            //            scaledRightVelocity = Math.min(scaledRightVelocity,scaledMaxVelocity);
//
//            robot.chassis.setVelocity(scaledLeftVelocity, scaledRightVelocity);
//
//            //robot.chassis.backLeftMotor
//            //            robot.telemetry.addData("coords: ", currentPosition.x + ", " + currentPosition.y);
//            //            robot.telemetry.addData("lp: ", lookaheadPoint.toString());
//            //
//            //            robot.telemetry.addData("left:", leftWheelTargetVelocity);
//            //            robot.telemetry.addData("right:", rightWheelTargetVelocity);
//            //            robot.telemetry.update();
//            double leftWheelTargetVelocity1 = getLeftWheelTargetVelocity(closestPoint.velocity, curvature);
//            double rightWheelTargetVelocity1 = getRightWheelTargetVelocity(closestPoint.velocity, curvature);
//
//            robot.showTelemetry(
//                    "location", currentPosition.simpleString(),
//                    "angle", "" + heading,
//                    "lookAhead", lookaheadPoint.simpleString(),
//                    "curvature", "" + curvature,
//                    "leftWheelTarget", "" + scaledLeftVelocity,
//                    "rightWheelTarget", "" + scaledRightVelocity,
//                    "forward Velocity", (leftWheelTargetVelocity1 + rightWheelTargetVelocity1) / 2 + "",
//                    "angular Velocity", (leftWheelTargetVelocity1 - rightWheelTargetVelocity1) / AllConstants.PurePursuit.TRACK_WIDTH + ""
//            );
//            while (robot.opMode.opModeIsActive() && robot.opMode.gamepad1.a) {
//                robot.chassis.setVelocity(0);
//                robot.showTelemetry(
//                        "location", currentPosition.simpleString(),
//                        "angle", "" + heading,
//                        "lookAhead", lookaheadPoint.simpleString(),
//                        "curvature", "" + curvature,
//                        "leftWheelTarget", "" + scaledLeftVelocity,
//                        "rightWheelTarget", "" + scaledRightVelocity,
//                        "forward Velocity", (leftWheelTargetVelocity1 + rightWheelTargetVelocity1) / 2 + "",
//                        "angular Velocity", (leftWheelTargetVelocity1 - rightWheelTargetVelocity1) / AllConstants.PurePursuit.TRACK_WIDTH + ""
//                );
//            }
//        } while (lastKnownPointIndex < pursuitPathing.size() - 1 && robot.opMode.opModeIsActive()); //TODO: use robot x & robot y for more accurate ending
//    }
//
//    public double normalizeVelocity(double targetVelocity) {
//        return targetVelocity / AllConstants.PurePursuit.OVERALL_MAX_VELOCITY;
//    }
//
//    public double[] normalizeVelocities(double leftVelocity, double rightVelocity) {
//        if (leftVelocity > AllConstants.PurePursuit.OVERALL_MAX_VELOCITY ||
//                rightVelocity > AllConstants.PurePursuit.OVERALL_MAX_VELOCITY) {
//            if (leftVelocity > rightVelocity) {
//                return new double[]{1.0, rightVelocity / leftVelocity};
//            } else {
//                return new double[]{leftVelocity / rightVelocity, 1.0};
//            }
//        } else {
//            return new double[]{normalizeVelocity(leftVelocity), normalizeVelocity(rightVelocity)};
//        }
//    }
//
//    public double getLeftWheelTargetVelocity(double targetVelocity, double curvature) {
//        return targetVelocity * (2 + curvature * robot.prefs.TRACK_WIDTH) / 2;
//    }
//
//    public double getRightWheelTargetVelocity(double targetVelocity, double curvature) {
//        return targetVelocity * (2 - curvature * robot.prefs.TRACK_WIDTH) / 2;
//    }
//
//    public double getWheelFeedbackVelocity(double targetVelocity, double leftWheelVelocity) {
//        return AllConstants.PurePursuit.PROPORTIONAL_FEEDBACK_CONSTANT * (targetVelocity - leftWheelVelocity);
//    }
//
//    //TODO: CHECK TO TRY THE FEEDFORWARD & FEEDBACK LATER!!
//    public double[] getLeftAndRightWheelVelocities(double targetVelocity, double curvature, double targetAcceleration,
//                                                   double leftVelocity, double rightVelocity, double track_width) {
//        double[] arr = {0, 0, 0, 0};
//        arr[0] = targetVelocity * (2 + curvature * track_width) / 2; //left
//        arr[1] = targetVelocity * (2 - curvature * track_width) / 2; //right;
//        arr[0] = AllConstants.PurePursuit.FEEDFORWARD_VELOCITY_CONSTANT * arr[0] + AllConstants.PurePursuit.FEEDFORWARD_ACCELERATION_CONSTANT * targetAcceleration +
//                AllConstants.PurePursuit.PROPORTIONAL_FEEDBACK_CONSTANT * (targetVelocity - leftVelocity);
//        arr[1] = AllConstants.PurePursuit.FEEDFORWARD_VELOCITY_CONSTANT * arr[1] + AllConstants.PurePursuit.FEEDFORWARD_ACCELERATION_CONSTANT * targetAcceleration +
//                AllConstants.PurePursuit.PROPORTIONAL_FEEDBACK_CONSTANT * (targetVelocity - rightVelocity);
//        return arr;
//    }
//}
