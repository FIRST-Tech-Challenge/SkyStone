package org.firstinspires.ftc.teamcode.auto.structurebuilder;

import com.acmerobotics.roadrunner.geometry.Vector2d;

public class Stone {

    /* Axes of the stone will be defined with respect to the foundation.
     * positive x-axis points towards the long side of the foundation on the right defined by looking at the foundation when approaching from the loading zone
     * positive y-axis points towards the short side of the foundation straight ahead defined by looking at the foundation when approaching from the loading zone
     * positive z-axis is up with respect to the ground
     * 0 in any axis is as the bottom left of the foundation
     * 1 unit in the z-axis is one stone high
     */

    private double xyOrientation;
    private double zOrientation;

    private int xCoord;
    private int yCoord;
    private int zCoord;

    public Stone(double xyOrientation, double zOrientation, int xCoord, int yCoord, int zCoord){
        this.xyOrientation = xyOrientation;
        this.zOrientation = zOrientation;
        this.xCoord = xCoord;
        this.yCoord = yCoord;
        this.zCoord = zCoord;
    }

    public double getXYOrientation(){
        return xyOrientation;
    }

    public double getZOrientation(){
        return zOrientation;
    }

    public int getXCoord(){
        return xCoord;
    }

    public int getYCoord(){
        return yCoord;
    }

    public int getZCoord(){
        return zCoord;
    }

    public void setXYOrientation(double xyOrientation){
        this.xyOrientation = xyOrientation;
    }

    public void setZOrientation(double zOrientation){
        this.zOrientation = zOrientation;
    }

    public void setXCoord(int xCoord){
        this.xCoord = xCoord;
    }

    public void setYOrientation(int yCoord){
        this.yCoord = yCoord;
    }

    public void setZOrientation(int zCoord){
        this.zOrientation = zCoord;
    }

    public Vector2d[] getOccupiedSpace(){
        Vector2d[] spaces = new Vector2d[2];
        spaces[0] = new Vector2d(xCoord,yCoord);
        spaces[1] = new Vector2d(xCoord + Math.cos(xyOrientation),yCoord + Math.sin(xyOrientation));
        return spaces;
    }

}
