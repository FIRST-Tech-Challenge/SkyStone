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

    public void followPath(double roundRadius, ArrayList<RobotPoint> path){
        while(opMode.opModeIsActive()){
            RobotPoint targetPoint = PathingAgent.getTargetPoint(roundRadius, odometer.x, odometer.y, path);

            if(targetPoint.x == 404 && targetPoint.y == 404){ //If it's the end of the path, or there is no intersection
                break;
            }

            double xDist, yDist, distance, heading;
            double targSpeed, scale, targVX, targVY;

            xDist = targetPoint.x - odometer.x;
            yDist = targetPoint.y - odometer.y;
            distance = Math.hypot(xDist, yDist);
            heading = odometer.x;

            targSpeed = 0.7;
            scale = targSpeed/distance;

            targVX = xDist * scale;
            targVY = yDist * scale;

            setGlobalVelocity(targVX, targVY, 0);

            odometer.update();
            drivebase.update();

        }
        drivebase.freeze();

    }

    public void setGlobalVelocity(double xVel, double yVel, double hVel) { // Verified
        double h = odometer.heading;

        double xRelVel = MathFunctions.cosine(-h) * xVel - MathFunctions.sine(-h) * yVel;
        double yRelVel = MathFunctions.sine(-h) * xVel + MathFunctions.cosine(-h) * yVel;

        drivebase.setRelativeVelocity(xRelVel, yRelVel, hVel);
        drivebase.update();

    }

    // Stand-alone Movement Functions
    public void movetoPointConstants(RobotPoint targetPoint, double speedFar, double speedNear, double arrivedThresh) {

        Proportional orient = new Proportional(0.02, 0.2);

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

            orient.update(targetPoint.heading, heading);
            hCorrect = orient.correction;

            setGlobalVelocity(targVX, targVY, hCorrect);

            endCondition = (distance < arrivedThresh);

        }while(!endCondition && opMode.opModeIsActive());

    }

}
