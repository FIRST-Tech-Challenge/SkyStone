package org.firstinspires.ftc.teamcode.Movement.MotionPlanning;

import org.firstinspires.ftc.teamcode.Utility.MathFunctions;

import java.util.ArrayList;

public class PathingAgent {

    public static RobotPoint getTargetPoint(double robotX, double robotY, ArrayList<RobotPoint> path){

        RobotPoint targetPoint;
        RobotPoint endPoint = new RobotPoint(404, 404, 0, 0, 0); // Default case

        for(int i=0; i<path.size()-1; i++){
            RobotPoint pathPoint1 = path.get(i);
            RobotPoint pathPoint2 = path.get(i+1);

            ArrayList<RobotPoint> candidates = MathFunctions.lineCircleIntersection(robotX, robotY,
                                                pathPoint1.radius, pathPoint1.x, pathPoint1.y, pathPoint2.x, pathPoint2.y);

            if(!candidates.isEmpty()){
                if (candidates.size() == 1) {
                    targetPoint = candidates.get(0);

                } else if (candidates.size() == 2) {
                    RobotPoint point1 = candidates.get(0);
                    RobotPoint point2 = candidates.get(1);

                    // Figure out which point is closer to the next point (further along the path)
                    double distance1 = Math.hypot((pathPoint2.x - point1.x), (pathPoint2.y - point1.y));
                    double distance2 = Math.hypot((pathPoint2.x - point2.x), (pathPoint2.y - point2.y));

                    if (distance1 > distance2) { //Choose point 2
                        targetPoint = point2;
                    } else { //Choose point 1
                        targetPoint = point1;
                    }
                }

                if(pathPoint2.isLastPoint){
                    targetPoint = endPoint;
                }

            }else{
                targetPoint = endPoint;
            }


            double segmentDistance = Math.hypot((pathPoint1.x - pathPoint2.x), (pathPoint1.y - pathPoint2.y));

            targetPoint.heading = pathPoint2.heading;
            targetPoint.speed = pathPoint1.speed;

        }

        return targetPoint;
    }


}
