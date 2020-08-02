package org.firstinspires.ftc.teamcode;

public class CurvePoint extends Coordinate {
    private boolean curveY;
    private Coordinate target;
    private boolean active = false;
    private boolean facing;
    public CurvePoint(double x, double y, boolean curveY, Coordinate target){
        super(x, y);
        this.curveY = curveY;
        this.target = target;
        active = true;
    }
    public CurvePoint(double x, double y){
        super(x, y);
    }
    public CurvePoint(){
    }
    public boolean isCurveY(){
        return curveY;
    }
    public boolean isCurveX(){
        return !curveY;
    }
    public void setCurveY(){
        curveY = true;
        active = true;
    }
    public void setCurveX() {
        curveY = false;
        active = true;
    }
    public void setTarget(Coordinate desired){
        target = desired;
        active = true;
    }
    public boolean getFacing(){
      return facing;
    }
    public void setFacing(boolean face){
      facing = face;
    }
    public Coordinate getTarget(){
        return target;
    }
    public void setActive(boolean set){
        active = set;
    }
    public boolean isActive(){
        return active;
    }
    @Override
    public void setPoint(Coordinate point){
        super.setPoint(point);
        active = true;
    }
}
