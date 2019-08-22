package org.firstinspires.ftc.teamcode.helperclasses;

/**
 * Class for pure pursuit points
 * Contains info of physical location, turn bias, and max velocities
 */
public class WayPoint {

    public static int iRel; // variable between methods to tell which segment the point is on
    public double x; // x-point (cm)
    public double y; // y-point (cm)
    public double maxSpeed; // max speed the robot can go
    public double goalTurnPercentageBias; // how much of the turn is based upon the next segment
    public double turnAccelerationCap; // maximum acceleration the robot can move

    public WayPoint(double x, double y, double mS, double gTPB, double tAC){
        this.x = x;
        this.y = y;
        maxSpeed = mS;
        goalTurnPercentageBias = gTPB;
        turnAccelerationCap = tAC;
    }
}
