package org.firstinspires.ftc.teamcode.Skystone.Modules;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Skystone.HardwareCollection;
import org.firstinspires.ftc.teamcode.Skystone.MathFunctions;
import org.firstinspires.ftc.teamcode.Skystone.MotionProfiler.CatmullRomSplineUtils;
import org.firstinspires.ftc.teamcode.Skystone.MotionProfiler.Point;
import org.firstinspires.ftc.teamcode.Skystone.Robot;

import java.util.Arrays;

public class PathModule {

    // make sure no null object reference from these variables
    private boolean isFollowingPath;
    public double[][] data;
    public double moveSpeed;
    public double turnSpeed;
    public double optimalAngle;

    private double[][] oldData;
    private Point[] pointData;
    private Point[] pathPoints;
    private double robotX;
    private double robotY;
    private double robotAngle;
    private double followPointX;
    private double followPointY;

    public PathModule(){
        isFollowingPath = false;
        data = new double[][]{{0,0},{0,0}};
    }

    public void runPath(double[][] data, double moveSpeed, double turnSpeed, double optimalAngle){
        this.data = data;
        this.moveSpeed = moveSpeed;
        this.turnSpeed = turnSpeed;
        this.optimalAngle = optimalAngle;
        isFollowingPath = true;
    }

    public synchronized void update(Robot robot){
        if (isFollowingPath){

            // if data has changed then generate new pathpoints
            if (!Arrays.deepEquals(data, oldData)){
                pointData = new Point[data.length];

                for (int i = 0; i < data.length; i++) {
                    pointData[i] = new Point(data[i][0], data[i][1]);
                }

                pathPoints = CatmullRomSplineUtils.generateSpline(pointData, 100, robot);
            }

            robotX = robot.odometryModule.worldX;
            robotY = robot.odometryModule.worldY;
            robotAngle = robot.odometryModule.worldAngle;

            // splineMove logic here

            double distanceToEnd = Math.hypot(robotX - pointData[pointData.length - 1].x, robotY - pointData[pointData.length - 1].y);

            updateMovementsToPoint(robot, robotX, robotY, moveSpeed, turnSpeed, optimalAngle, false);

            if (distanceToEnd < 1){
                robot.driveModule.brakeRobot();
                isFollowingPath = false;
            }

            oldData = data;
        }
    }

    public void updateMovementsToPoint(Robot robot, double x, double y, double moveSpeed, double turnSpeed, double optimalAngle, boolean willMecanum) {
        double distanceToTarget = Math.hypot(x - robotX, y - robotY);
        double absoluteAngleToTarget = Math.atan2(y - robotY, x - robotX);

        double relativeAngleToPoint = MathFunctions.angleWrap(absoluteAngleToTarget - robotAngle);
        double relativeXToPoint = Math.cos(relativeAngleToPoint) * distanceToTarget;
        double relativeYToPoint = Math.sin(relativeAngleToPoint) * distanceToTarget;
        double relativeTurnAngle = relativeAngleToPoint + optimalAngle;

        if (relativeTurnAngle > Math.PI) {
            relativeTurnAngle -= 2 * Math.PI;
        }

        double xPower = relativeXToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));

        double yPower = 0.4 * relativeYToPoint / (Math.abs(relativeYToPoint) + Math.abs(relativeXToPoint));

        if (willMecanum) {
            yPower = relativeYToPoint / (Math.abs(relativeYToPoint) + Math.abs(relativeXToPoint));
        }

        robot.driveModule.xMovement = xPower * moveSpeed;
        robot.driveModule.yMovement = yPower * moveSpeed;
        robot.driveModule.turnMovement = 5 * Range.clip(relativeTurnAngle / Math.toRadians(360), -1, 1) * turnSpeed;

        if (willMecanum) {
            robot.driveModule.turnMovement = Range.clip(relativeTurnAngle / Math.toRadians(360), -1, 1) * turnSpeed;
        }
    }

    /**
     * public void splineMove(double[][] data, double moveSpeed, double turnSpeed, double slowDownSpeed, double slowDownDistance, double optimalAngle, double angleLockRadians, double angleLockInches, ArrayList<Action> actions) {
     *         splineMove(data, moveSpeed, turnSpeed, slowDownSpeed, slowDownDistance, optimalAngle, angleLockRadians, angleLockInches, actions, false, 0);
     *     }
     *
     *     public void splineMove(double[][] data, double moveSpeed, double turnSpeed, double slowDownSpeed, double slowDownDistance, double optimalAngle, double angleLockRadians, double angleLockInches, ArrayList<Action> actions, boolean isTimeKill, long endTime) {
     *         splineMove(data, moveSpeed, turnSpeed, slowDownSpeed, slowDownDistance, optimalAngle, angleLockRadians, angleLockInches, actions, isTimeKill, endTime, false, new Point(0, 0));
     *     }
     *
     *     public void splineMove(double[][] data, double moveSpeed, double turnSpeed, double slowDownSpeed, double slowDownDistance, double optimalAngle, double angleLockRadians, double angleLockInches, ArrayList<Action> actions, boolean isTimeKill, long endTime, boolean isMecanumPoint, Point mecanumPoint) {
     *         double posAngle;
     *
     *         Point[] data2 = new Point[data.length];
     *
     *         for (int i = 0; i < data.length && linearOpMode.opModeIsActive(); i++) {
     *             data2[i] = new Point(data[i][0], data[i][1]);
     *         }
     *
     *         Point[] pathPoints = CatmullRomSplineUtils.generateSpline(data2, 100, this);
     *
     *         addSplinePoints(pathPoints);
     *         addWaypoints(data);
     *
     *         boolean isMoving = true;
     *         boolean isStuck = false;
     *
     *         long lastPosTime = SystemClock.elapsedRealtime();
     *         Point lastPos = new Point(robotPos.x,robotPos.y);
     *
     *         int followIndex = 1;
     *         double angleLockScale;
     *         double distanceToEnd;
     *         double distanceToNext = Double.MAX_VALUE;
     *         double desiredHeading;
     *
     *         long currentTime;
     *         long startTime = SystemClock.elapsedRealtime();
     *         while (linearOpMode.opModeIsActive()) {
     *             currentTime = SystemClock.elapsedRealtime();
     *
     *             posAngle = MathFunctions.angleWrap(anglePos + 2 * Math.PI);
     *
     *             for (int p = pathPoints.length - 1; p >= 0 && linearOpMode.opModeIsActive(); p--) {
     *                 if (Math.hypot(robotPos.x - pathPoints[p].x, robotPos.y - pathPoints[p].y) < 10) {
     *                     followIndex = p;
     *                     break;
     *                 }
     *             }
     *
     *             distanceToEnd = Math.hypot(robotPos.x - data2[data2.length - 1].x, robotPos.y - data2[data2.length - 1].y);
     *             distanceToNext = Math.hypot(robotPos.x - pathPoints[followIndex].x, robotPos.y - pathPoints[followIndex].y);
     *             if (followIndex > 0) {
     *                 desiredHeading = angleWrap(Math.atan2(pathPoints[followIndex].y - pathPoints[followIndex - 1].y, pathPoints[followIndex].x - pathPoints[followIndex - 1].x) + 2 * Math.PI);
     *             } else {
     *                 desiredHeading = angleWrap(Math.atan2(pathPoints[followIndex + 1].y - pathPoints[followIndex].y, pathPoints[followIndex + 1].x - pathPoints[followIndex].x) + 2 * Math.PI);
     *             }
     *
     *
     *             if (desiredHeading == 0) {
     *                 desiredHeading = Math.toRadians(360);
     *             }
     *             if (angleLockRadians == 0) {
     *                 angleLockRadians = Math.toRadians(360);
     *             }
     *
     *             angleLockScale = Math.abs(angleLockRadians - posAngle) * Math.abs(desiredHeading - angleLockRadians) * 1.8;
     *
     *
     *             if (distanceToEnd < angleLockInches) {
     *                 updateMovementsToPoint(pathPoints[followIndex].x, pathPoints[followIndex].y, moveSpeed, turnSpeed, optimalAngle, true);
     *
     *                 if (angleLockRadians - posAngle > Math.toRadians(0) && angleLockRadians - posAngle < Math.toRadians(180)) {
     *                     turnMovement = 1 * angleLockScale;
     *                 } else if (angleLockRadians - posAngle < Math.toRadians(0) || angleLockRadians - posAngle > Math.toRadians(180)) {
     *                     turnMovement = -1 * angleLockScale;
     *                 } else {
     *                     turnMovement = 0;
     *                 }
     *             } else if (isMecanumPoint && Math.hypot(robotPos.x - mecanumPoint.x, robotPos.y - mecanumPoint.y) < 15) {
     *                 updateMovementsToPoint(pathPoints[followIndex].x, pathPoints[followIndex].y, moveSpeed, turnSpeed, optimalAngle, true);
     *             } else {
     *                 updateMovementsToPoint(pathPoints[followIndex].x, pathPoints[followIndex].y, moveSpeed, turnSpeed, optimalAngle, false);
     *             }
     *
     *             if (distanceToEnd < 1) {
     *                 isMoving = false;
     *             }
     *
     *             if (distanceToEnd < slowDownDistance) {
     *                 if (slowDownSpeed > moveSpeed) {
     *                     xMovement *= slowDownSpeed * (slowDownSpeed / moveSpeed);
     *                     yMovement *= slowDownSpeed * (slowDownSpeed / moveSpeed);
     *                     turnMovement *= slowDownSpeed * (slowDownSpeed / moveSpeed);
     *                 } else {
     *                     xMovement *= slowDownSpeed;
     *                     yMovement *= slowDownSpeed;
     *                     turnMovement *= slowDownSpeed;
     *                 }
     *             }
     *
     *             // go through all actionpoints and see if the robot is near one
     *             boolean isFinishedAllActions = true;
     *             if (actions.size() != 0) {
     *                 currentTime = SystemClock.elapsedRealtime();
     *                 for (int i = 0; i < actions.size() && linearOpMode.opModeIsActive(); i++) {
     *                     Action action = actions.get(i);
     *
     *                     Point actionPoint = action.getActionPoint();
     *
     *                     if (action.getActionState() == ActionState.PROCESSING || action.getActionState() == ActionState.PENDING) {
     *                         isFinishedAllActions = false;
     *                     }
     *
     *                     if (action.isExecuteOnEndOfPath()) {
     *                         if (!isMoving) {
     *                             action.executeAction(currentTime);
     *                         }
     *                     } else if ((Math.hypot(actionPoint.x - robotPos.x, actionPoint.y - robotPos.y) < 10) || (action.getActionState() == ActionState.PROCESSING)) {
     *                         action.executeAction(currentTime);
     *                     }
     *                 }
     *             }
     *
     *             // Test to see if the robot is stuck, every second
     *             if((currentTime - lastPosTime) >= 1000){
     *                 if(followIndex != pathPoints.length-1 && (Math.hypot(lastPos.x-robotPos.x, lastPos.y - robotPos.y)) < 1.5){
     *                     isStuck = true;
     *                 }else{
     *                     isStuck = false;
     *                 }
     *
     *                 if (isStuck) {
     *                     brakeRobot();
     *                     linearOpMode.stop();
     *                 }
     *
     *                 lastPos.x = robotPos.x;
     *                 lastPos.y = robotPos.y;
     *                 lastPosTime = currentTime;
     *             }
     *
     *             if (distanceToEnd < 1 && Math.abs(Math.toDegrees(posAngle) - Math.toDegrees(angleLockRadians)) < 5 && isFinishedAllActions) {
     *                 brakeRobot();
     *                 break;
     *             }
     *
     *             if (isTimeKill && currentTime - startTime >= endTime) {
     *                 break;
     *             }
     *
     *
     *             applyMove();
     *         }
     *     }
     *
     *     public void moveToPoint(double x, double y, double moveSpeed, double turnSpeed, double optimalAngle) {
     *
     *         // find the total distance from the start point to the end point
     *         double totalDistanceToTarget = Math.hypot(x - robotPos.x, y - robotPos.y);
     *
     *         double totalTimeSeconds = totalDistanceToTarget / 20;
     *
     *         // so deceleration works
     *         this.setDrivetrainMotorModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
     *
     *         // start a timer
     *         long startTime = SystemClock.elapsedRealtime();
     *
     *         // keep on running this
     *         while (linearOpMode.opModeIsActive() && SystemClock.elapsedRealtime() - startTime < totalTimeSeconds * 1000
     *                 && linearOpMode.gamepad1.left_stick_y == 0 && linearOpMode.gamepad1.left_trigger == 0 && linearOpMode.gamepad1.right_trigger == 0 && linearOpMode.gamepad1.right_stick_x == 0) {
     *             // store your current position in variables
     *             double xPos = robotPos.x;
     *             double yPos = robotPos.y;
     *             double anglePos = this.anglePos;
     *
     *             // find your current distance to target
     *             double distanceToTarget = Math.hypot(x - xPos, y - yPos);
     *
     *             // only way to break the loop, if the distance to target is less than 1
     *             if (distanceToTarget < 1) {
     *                 break;
     *             }
     *
     *             // find the absolute angle to target
     *             double absoluteAngleToTarget = Math.atan2(y - yPos, x - xPos);
     *             // find the relative angle of the target to the robot
     *             double relativeAngleToPoint = MathFunctions.angleWrap(absoluteAngleToTarget - anglePos);
     *             // x distance for the robot to its target
     *             double relativeXToPoint = Math.cos(relativeAngleToPoint) * distanceToTarget;
     *             // y distance for the robot to its target
     *             double relativeYToPoint = Math.sin(relativeAngleToPoint) * distanceToTarget;
     *             // adds optimal angle
     *             double relativeTurnAngle = relativeAngleToPoint + optimalAngle;
     *
     *             // converting the relativeX and relativeY to xPower and yPower
     *             double xPower = relativeXToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));
     *             double yPower = relativeYToPoint / (Math.abs(relativeYToPoint) + Math.abs(relativeXToPoint));
     *
     *             // find the deceleration
     *             double decelerationScaleFactor = Range.clip(2 * Math.sqrt(Math.pow(totalDistanceToTarget, 2) - Math.pow(totalDistanceToTarget - distanceToTarget, 2)) / totalDistanceToTarget, -1, 1);
     *
     *             // get everything into x, y, and turn movements for applyMove
     *             // the robot can be viewed as something that moves on a coordinate plane
     *             // that moves in a x and y direction but also has a heading, where it is pointing
     *             xMovement = xPower * moveSpeed * decelerationScaleFactor;
     *             yMovement = yPower * moveSpeed * decelerationScaleFactor;
     *             turnMovement = Range.clip(relativeTurnAngle / Math.toRadians(360),
     *                     -1, 1) * turnSpeed * decelerationScaleFactor;
     *
     *             applyMove();
     *         }
     *         brakeRobot();
     *     }
     */
}
