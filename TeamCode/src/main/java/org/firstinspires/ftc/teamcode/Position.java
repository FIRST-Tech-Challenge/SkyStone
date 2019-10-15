package org.firstinspires.ftc.teamcode;

/**
 * 2019.10.06
 * Position class holds all of the variables containing robot position parts
 * Created by Athena Z.
 */

public class Position {
    //coordinates of the robot in the field
    private double worldX;
    private double worldY;
    private double worldAngle;
    private double worldPreviousX;
    private double worldPreviousY;
    private double worldPreviousAngle;

    //coordinates of the robot relative of its previous position (when the encoders last stopped)
    private double relativeX;
    private double relativeY;
    private double relativeAngle;
    private double relativePreviousX;
    private double relativePreviousY;
    private double relativePreviousAngle;

    private double initX;
    private double initY;

    public double getRelativeX() {
        return relativeX;
    }
    
    public void setRelativeX(double x){
        relativeX = x;
    }
    
    public double getRelativeY(){
        return relativeY;
    }

    public void setRelativeY(double y){
        relativeY = y;
    }

    public double getRelativeAngle() {
        return relativeAngle;
    }

    public void setRelativeAngle(double x){
        relativeAngle = x;
    }

    public double getWorldY(){
        return worldY;
    }

    public void setWorldY(double y){
        worldY = y;
    }

    public double getWorldAngle() {
        return worldAngle;
    }

    public void setWorldAngle(double x){
        worldAngle = x;
    }

    public double getWorldX() {
        return worldX;
    }

    public void setWorldX(double x){
        worldX = x;
    }

    public double getRelativePreviousAngle(){return this.relativePreviousAngle;}

    public void setRelativePreviousAngle(double angle){relativePreviousAngle = angle;}

    public double getRelativePreviousX() { return this.relativePreviousX; }

    public void setRelativePreviousX(double x) { relativePreviousX = x; }

    public double getRelativePreviousY() { return this.relativePreviousY; }

    public void setRelativePreviousY(double y) { relativePreviousY = y; }

    public double getWorldPreviousX() { return this.worldPreviousX; }

    public void setWorldPreviousX(double x) { worldPreviousX = x; }

    public double getWorldPreviousY() { return this.worldPreviousY; }

    public void setWorldPreviousY(double y) { worldPreviousY = y; }

    public double getWorldPreviousAngle() { return this.worldPreviousAngle; }

    public void setWorldPreviousAngle(double angle) { worldPreviousAngle = angle; }

    public double getInitX(){return this.initX; }

    public void setInitX ( double x) {initX = x;}

    public double getInitY() { return this.initY ;}

    public void setInitY(double y) {initY = y; }
}
