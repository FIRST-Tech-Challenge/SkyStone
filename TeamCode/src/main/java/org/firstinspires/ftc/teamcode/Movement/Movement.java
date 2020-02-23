package org.firstinspires.ftc.teamcode.Movement;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Controllers.GatedConstant;
import org.firstinspires.ftc.teamcode.Controllers.GatedPid;
import org.firstinspires.ftc.teamcode.Controllers.Proportional;
import org.firstinspires.ftc.teamcode.HardwareSystems.ActionHandler;
import org.firstinspires.ftc.teamcode.HardwareSystems.AutoClaws;
import org.firstinspires.ftc.teamcode.HardwareSystems.Intake;
import org.firstinspires.ftc.teamcode.HardwareSystems.Outtake;
import org.firstinspires.ftc.teamcode.Movement.Localization.Odometer;
import org.firstinspires.ftc.teamcode.Movement.MotionPlanning.PathingAgent;
import org.firstinspires.ftc.teamcode.Movement.MotionPlanning.RobotPoint;
import org.firstinspires.ftc.teamcode.Utility.MathFunctions;

import java.util.ArrayList;

public class Movement {

    private Drivebase drivebase;
    private Odometer odometer;
    private LinearOpMode opMode;

    // External Action Handlers
    private ActionHandler handler;

    public boolean useActionHandler = false;

    public Movement(LinearOpMode opMode, Drivebase drivebase, Odometer odometer){

        this.drivebase = drivebase;
        this.odometer = odometer;
        this.opMode = opMode;

    }

    public void setActionHandler(ActionHandler handler) {
        this.handler = handler;
    }

    public void followPath(ArrayList<RobotPoint> path, double radius){

        Proportional orient = new Proportional(0.035, 0.26);
        RobotPoint lastPoint = path.get(path.size()-1); //Last point in the ArrayList

        while(opMode.opModeIsActive()){
            RobotPoint targetPoint = PathingAgent.getTargetPoint(odometer.x, odometer.y, radius, path);
            // PathingAgent uses line-circle intersect to get 0,1, or 2 points, then picks whichever point is closest to the current "goal" RobotPoint

            if(targetPoint.x == 404 || targetPoint.y == 404){ // If there are no intersections
                // Trigger fail-safe to regain the path
                targetPoint = PathingAgent.getFailsafePoint(odometer.x, odometer.y, path);
            }

            // Checking if the robot is within a certain distance of the "last" point
            double distanceX = lastPoint.x - odometer.x;
            double distanceY = lastPoint.y - odometer.y;
            double totalDistance = Math.hypot(distanceX, distanceY);

            if(totalDistance < 5){ // End loop if you are within 5 cm of the last point
                break;
            }

            //Now that the robot knows where to go (targetPoint, returned by PathingAgent), the following code handles motor powers.
            double xDist, yDist, distance, heading;
            double targSpeed, scale, targVX, targVY;

            xDist = targetPoint.x - odometer.x;
            yDist = targetPoint.y - odometer.y;
            distance = Math.hypot(xDist, yDist);
            heading = odometer.heading;

            targSpeed = Math.abs(targetPoint.speed);
            scale = targSpeed / distance;

            targVX = xDist * scale;
            targVY = yDist * scale;

            orient.update(targetPoint.heading, heading); //targetPoint.heading is goal heading

            setGlobalVelocity(targVX, targVY, orient.correction);

            doActions(targetPoint);

            odometer.update();
            drivebase.update();

        }
        drivebase.freeze();

    }

    public void setGlobalVelocity(double xVel, double yVel, double hVel) { // Verified
        if(opMode.opModeIsActive()){
            double h = odometer.heading;

            double xRelVel = MathFunctions.cosine(-h) * xVel - MathFunctions.sine(-h) * yVel;
            double yRelVel = MathFunctions.sine(-h) * xVel + MathFunctions.cosine(-h) * yVel;

            drivebase.setRelativeVelocity(xRelVel, yRelVel, hVel);
            drivebase.update();
        }else{
            drivebase.freeze();
        }
    }

    // Stand-alone Movement Functions
    public void moveToPointConstants(RobotPoint targetPoint, double speedFar, double speedNear, double switchThresh, double arrivedThresh) {

        GatedPid orient = new GatedPid(10, 0.3,0.009,0,0.023,0, 0.4, 0.05);

        double xDist, yDist, distance, heading;
        double targSpeed, scale;
        double targVX, targVY, hCorrect;
        boolean endCondition;

        GatedConstant speedFinder = new GatedConstant(speedFar, speedNear, switchThresh);

        do {

            xDist = targetPoint.x - odometer.x;
            yDist = targetPoint.y - odometer.y;
            distance = Math.hypot(xDist, yDist);
            heading = odometer.heading;

            targSpeed = Math.abs(speedFinder.correction);
            scale = targSpeed / distance;

            targVX = xDist * scale;
            targVY = yDist * scale;
            // Verified ^

            speedFinder.update(0, distance);
            orient.update(targetPoint.heading, heading);
            hCorrect = orient.correction;

            setGlobalVelocity(targVX, targVY, hCorrect);

            endCondition = (distance < arrivedThresh) && orient.error < 2;

            doActions(targetPoint);

            odometer.update();

        }while(!endCondition && opMode.opModeIsActive());

        drivebase.freeze();

    }

    public void moveToPointPD(RobotPoint targetPoint, double switchThresh, double arrivedThresh) {

        GatedPid orient = new GatedPid(10, 0.3,0.009,0,0.023,0, 0.4, 0.05);

        double xDist, yDist, distance, heading;
        double targSpeed, scale;
        double targVX, targVY, hCorrect;
        boolean endCondition;

        GatedPid speedFinder = new GatedPid(switchThresh, 0.8, 0.018, 0, 0.05, 0, 0.8,0.2);

        do {

            xDist = targetPoint.x - odometer.x;
            yDist = targetPoint.y - odometer.y;
            distance = Math.hypot(xDist, yDist);
            heading = odometer.heading;

            targSpeed = Math.abs(speedFinder.correction);
            scale = targSpeed / distance;

            targVX = xDist * scale;
            targVY = yDist * scale;
            // Verified ^

            speedFinder.update(0, distance);
            orient.update(targetPoint.heading, heading);
            hCorrect = orient.correction;

            setGlobalVelocity(targVX, targVY, hCorrect);

            endCondition = (distance < arrivedThresh) && orient.error < 2;

            doActions(targetPoint);

            odometer.update();

        }while(!endCondition && opMode.opModeIsActive());

        drivebase.freeze();

    }

    public void moveToPointPD2(RobotPoint targetPoint,double switchThresh, double arrivedThresh) {

        GatedPid orient = new GatedPid(10, 0.3,0.009,0,0.023,0, 0.4, 0.05);

        double xDist, yDist, distance, heading;
        double targSpeed, scale;
        double targVX, targVY, hCorrect;
        boolean endCondition;

        GatedPid speedFinder = new GatedPid(switchThresh, 1.25, 0.026, 0, 0.06, 0, 1.25,0.1);

        do {

            xDist = targetPoint.x - odometer.x;
            yDist = targetPoint.y - odometer.y;
            distance = Math.hypot(xDist, yDist);
            heading = odometer.heading;

            targSpeed = Math.abs(speedFinder.correction);
            scale = targSpeed / distance;

            targVX = xDist * scale;
            targVY = yDist * scale;
            // Verified ^

            speedFinder.update(0, distance);
            orient.update(targetPoint.heading, heading);
            hCorrect = orient.correction;

            setGlobalVelocity(targVX, targVY, hCorrect);

            endCondition = (distance < arrivedThresh) && orient.error < 2;

            doActions(targetPoint);

            odometer.update();

        }while(!endCondition && opMode.opModeIsActive());

        drivebase.freeze();

    }

    private void doActions(RobotPoint point){
        if(useActionHandler){
            handler.doActions(point);
        }
    }



}
