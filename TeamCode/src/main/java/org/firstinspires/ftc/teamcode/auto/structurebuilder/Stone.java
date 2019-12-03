package org.firstinspires.ftc.teamcode.auto.structurebuilder;

import com.acmerobotics.roadrunner.geometry.Vector2d;

public class Stone {

//  Axes of the Foundation will be defined with respect to the Foundation
//  The nub closest to the origin will be counted as the origin of the stone

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
