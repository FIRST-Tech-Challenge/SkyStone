package org.firstinspires.ftc.teamcode.Movement;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Controllers.GatedConstant;
import org.firstinspires.ftc.teamcode.Controllers.Proportional;
import org.firstinspires.ftc.teamcode.Movement.Localization.Odometer;
import org.firstinspires.ftc.teamcode.Movement.MotionPlanning.PathingAgent;
import org.firstinspires.ftc.teamcode.Movement.MotionPlanning.RobotPoint;
import org.firstinspires.ftc.teamcode.Utility.MathFunctions;

import java.util.ArrayList;

public class Movement {

    private Drivebase drivebase;
    private Odometer odometer;
    private LinearOpMode opMode;

    public Movement(LinearOpMode opMode, Drivebase drivebase, Odometer odometer){
        this.drivebase = drivebase;
        this.odometer = odometer;
        this.opMode = opMode;

    }

    public void followPath(ArrayList<RobotPoint> path){

        Proportional orient = new Proportional(0.06, 0.6);
        RobotPoint lastPoint = path.get(path.size()-2); //Actually is the second to last point

        while(opMode.opModeIsActive()){
            RobotPoint targetPoint = PathingAgent.getTargetPoint(odometer.x, odometer.y, path);

            if(targetPoint.x == 404 || targetPoint.y == 404){ //If it's the end of the path, or there is no intersection
                break;
            }

            // Checking if the robot is within a certain distance of the "last" point
            double distanceX = lastPoint.x - odometer.x;
            double distanceY = lastPoint.y - odometer.y;
            double totalDistance = Math.hypot(distanceX, distanceY);

            if(totalDistance < lastPoint.radius+5){
                break;
            }

            double xDist, yDist, distance, heading;
            double targSpeed, scale, targVX, targVY;

            xDist = targetPoint.x - odometer.x;
            yDist = targetPoint.y - odometer.y;
            distance = Math.hypot(xDist, yDist);
            heading = odometer.heading;

            targSpeed = targetPoint.speed;
            scale = targSpeed/distance;

            targVX = xDist * scale;
            targVY = yDist * scale;

            orient.update(targetPoint.heading, heading);

            setGlobalVelocity(targVX, targVY, orient.correction);

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
    public void movetoPointConstants(RobotPoint targetPoint, double speedFar, double speedNear, double arrivedThresh) {

        Proportional orient = new Proportional(0.02, 0.3);

        double xDist, yDist, distance, heading;
        double targSpeed, scale;
        double targVX, targVY, hCorrect;
        boolean endCondition;

        GatedConstant speedFinder = new GatedConstant(speedFar, speedNear, arrivedThresh);

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

            endCondition = (distance < arrivedThresh);

            odometer.update();

        }while(!endCondition && opMode.opModeIsActive());

    }

}
