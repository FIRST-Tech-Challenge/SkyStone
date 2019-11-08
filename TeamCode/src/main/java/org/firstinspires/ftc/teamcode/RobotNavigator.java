package org.firstinspires.ftc.teamcode;

import android.util.Log;

public class RobotNavigator {
    private  int ENCODER_COUNTS_PER_ROTATION = 1440;

    private  double heading;
    private  double worldX, worldY, worldAngle;
    private  int previousLE;
    private  int previousRE;
    private  int previousHE;

    private  double deltaLE;
    private  double deltaRE;
    private  double deltaHE;
    RobotProfile profile;

    public RobotNavigator(RobotProfile profile) {
        this.profile = profile;
    }

    //10/19/19 Danielle - create init method
    public  void setInitPosition(double iY, double iX, double iAngle){
        worldX = iY;
        worldY = iY;
        heading= iAngle;
    }

    public  double getWorldY(){ return worldY; }
    public  double getWorldX(){ return worldX; }
    public  double getHeading(){return heading;}
    public  double getWorldAngle(){ return  worldAngle; }

    double center_radius;
    //10/5 & 10/11 Athena, Alejandra, Claire, Marianna implemented and calculated displacement for robot
    //William Gutrich 10/13/19, refactor code to a method to set all positions
    public  void updateAllPositions(int leftEncoder, int rightEncoder, int horEncoder) {
        double x=0,y = 0;
        double angle = 0;
        double tempY=0, tempX=0;

        ////10/5 Gavin and Hayden, calculate angle in each cycle
        deltaHE = convertEncoderCountsToCentimeters(horEncoder - previousHE);
        deltaLE = convertEncoderCountsToCentimeters(leftEncoder - previousLE);
        deltaRE = convertEncoderCountsToCentimeters(rightEncoder - previousRE);
//        Logger.logFile("deltaHE = " + deltaHE);
//        Logger.logFile("deltaLE = "+ deltaLE);
//        Logger.logFile("deltaRE = " + deltaRE);
        double abDiffEncoder = Math.abs(deltaLE) - Math.abs(deltaRE);

        //x, y are the displacement in each drive cycle
        if (deltaLE != deltaRE) {
            double radius = profile.hardwareSpec.leftRightWheelDist * deltaLE / (deltaLE - deltaRE);
            if(Math.abs(radius)> profile.hardwareSpec.leftRightWheelDist / 2) {
                center_radius = radius - profile.hardwareSpec.leftRightWheelDist / 2;
            }else{
                center_radius = radius;
            }
            if (deltaLE == 0) {
                angle = -deltaRE / profile.hardwareSpec.leftRightWheelDist;
//                Logger.logFile("deltaLE==0, angle = " + angle);
            }else if(center_radius == 0){
                angle =  Math.asin((deltaLE-deltaRE)/radius);
//                Logger.logFile("center_radius == 0, angle = " + angle);
            }else {
                angle =  deltaLE /  center_radius;
//                Logger.logFile("angle = " + angle);
            }

            double y1 = center_radius * Math.sin(angle);
            double x1 = center_radius * (1-Math.cos(angle));
            double x2 = deltaHE * Math.cos(angle);
            double y2 = deltaHE * Math.sin(angle);
            x = x1 + x2 ;
            y = y1 - y2;

            //apply the local/relative changes to the coordination system - adjust to start from 0 degree
            tempX = y * Math.sin(heading) + x * Math.cos(heading);
            tempY = y * Math.cos(heading) - x * Math.sin(heading);
        } else {
            tempX = deltaLE * Math.sin(heading) + deltaHE * Math.cos(heading) ;
            tempY = deltaLE * Math.cos(heading) + deltaHE * Math.sin(heading);
            //angle = 0;
        }

        //this is the heading angle relative to starting position
//        Logger.logFile("old heading = " + heading);
        heading = angleWrap( heading + angle);
        worldY  += tempY;
        worldX  += tempX;
//        Logger.logFile("heading = " + heading);
        //worldAngle = -Math.toDegrees(Math.atan2(worldY,worldX) - Math.PI/2);
        setLastEncoderPosition(leftEncoder,rightEncoder,horEncoder);
    }

    public  double toDegree(double radian){
        double degree = Math.toDegrees(radian);
        if(degree > 360){
            return angleWrap(degree % 360);
        }
        return degree;
    }

    // October 6th: Lucas and Marianna
    public  double convertEncoderCountsToCentimeters(double encoderCounts) {
        return encoderCounts*((profile.hardwareSpec.trackWheelDiameter*Math.PI)/ENCODER_COUNTS_PER_ROTATION);
    }

    public  double convertCMToEncoderCounts(double cm){
        return cm * ENCODER_COUNTS_PER_ROTATION / (Math.PI * profile.hardwareSpec.trackWheelDiameter);
    }

    //10/14 Gabriel, limit angle range -180 to 180
    public  double angleWrap(double angle) {
        if (angle > Math.PI)
            angle = angle - 2 * Math.PI;
        else if (angle < -Math.PI)
            angle = angle + 2 * Math.PI;
        return angle;
    }

    public  void reset(){
        worldX = 0;
        worldY = 0;
        worldAngle = 0;
    }

    public  void setInitEncoderCount(int LE, int RE, int HE){
        previousLE = LE;
        previousRE = RE;
        previousHE = HE;
    }

    public  void setLastEncoderPosition (int leftEncoder, int rightEncoder, int middleEncoder){
        previousLE = leftEncoder;
        previousRE = rightEncoder;
        previousHE = middleEncoder;
    }

    public  void setWorldPosition(double x, double y) {
        worldX = x;
        worldY = y;
    }

    public  void setEncoderCountPerRotation (int cpr){
        ENCODER_COUNTS_PER_ROTATION = cpr;
    }

}