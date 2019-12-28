package org.firstinspires.ftc.teamcode.Experimental.Units;

public class Vector {
    public enum MoveBehavior{
        LineTo, RotateTo, QuadraticCurveTo, CubicCurveTo, StrafeTo
    }
    private double posX;
    private double posY;
    private double heading;
    private Action action;
    private int paramType;
    private MoveBehavior moveBehavior;
    private double power;

    public Vector(double power, double positionX, double positionY, double heading, MoveBehavior moveBehavior){
        this.posX = positionX;
        this.posY = positionY;
        this.heading = heading;
        this.moveBehavior = moveBehavior;
        this.power = power;
        paramType = 0;
    }

    public Vector(Action action){
        this.action = action;
        paramType = 1;
    }

    public Vector(double power, double positionX, double positionY, MoveBehavior moveBehavior){
        this.posX = positionX;
        this.posY = positionY;
        this.moveBehavior = moveBehavior;
        this.power = power;
        paramType = 2;
    }

    public Vector(double power, double heading, MoveBehavior moveBehavior){
        this.heading = heading;
        this.moveBehavior = moveBehavior;
        this.power = power;
        paramType = 3;
    }

    public Vector(double positionX, double positionY, double heading){
        this.posX = positionX;
        this.posY = positionY;
        this.heading = heading;
        paramType = 4;
    }

    public double getX() { return posX; }

    public double getY() { return posY; }

    public double getHeading() { return heading; }

    public Action getAction() { return action; }

    public int getParamType() { return paramType; }

    public MoveBehavior getMoveBehavior() { return moveBehavior; }

    public double getPower() { return power; }

    public void setX(double posX) { this.posX = posX; }

    public void setY(double posY) { this.posY = posY; }

    public void setHeading(double heading) { this.heading = heading; }

    public void setAction(Action action) { this.action = action; }

    public void setParamType(int paramType) { this.paramType = paramType; }

    public void setMoveBehavior(MoveBehavior moveBehavior) { this.moveBehavior = moveBehavior; }

    public void setPower(double power) { this.power = power; }
}
