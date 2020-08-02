package org.firstinspires.ftc.teamcode;

public class Coordinate {
    private double x;
    private double y;

    public Coordinate(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public Coordinate(Coordinate point) {
        this.x = point.getX();
        this.y = point.getY();
    }
    public Coordinate(){}
    public void setPoint(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public void setPoint(Coordinate point) {
        this.x = point.getX();
        this.y = point.getY();
    }

    public void setX(double x) {
        this.x = x;
    }

    public void setY(double y) {
        this.y = y;
    }

    public double[] getPoint() {
        double[] arr = {x, y};
        return arr;
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    @Override
    public String toString() {
        return "[" + x + ", " + y + "]";
    }

    public void addX(double x) {
        this.x += x;
    }

    public void addY(double y) {
        this.y += y;
    }

    public void add(double x, double y) {
        addX(x);
        addY(y);
    }

    public double distanceTo(Coordinate B) {
        return Math.sqrt(Math.pow(B.getX() - getX(), 2) + Math.pow(B.getY() - getY(), 2));
    }
    public void polarAdd(double angle, double distance){
        add(xCovered(angle, distance), yCovered(angle, distance));
    }
    public double angleTo(Coordinate desired, boolean facing) {
        double x = desired.getX() - getX();
        double y = desired.getY() - getY();
        double angle = Math.toDegrees(Math.atan2(y, x));
        if (!facing) {
            angle -= 180;
        }
        if(angle > 180){
            angle -= 360;
        }
        else if(angle < -180){
            angle += 360;
        }
        return angle;
    }
    public double curveDistanceTo(Coordinate C, Coordinate B){
        double a = C.distanceTo(B);
        double c = distanceTo(B);
        return a+c;
        //return Math.abs(C.getX() - getX()) + Math.abs(C.getY() - getY());
    }
    public double getMatchX(Coordinate C, double heading){
        if(heading > 180){
            heading -= 360;
        }
        else if(heading < -180){
            heading += 360;
        }
        double x = C.getX() - getX();
        return x/Math.cos(Math.toRadians(heading));
    }
    public double getMatchY(Coordinate C, double heading){//verify
        if(heading > 180){
            heading -= 360;
        }
        else if(heading < -180){
            heading += 360;
        }
        double y = C.getY() - getY();
        return y/Math.sin(Math.toRadians(heading));
    }
    public double xCovered(double angle, double distance) {
        return Math.cos(Math.toRadians(angle)) * distance;
    }

    public double yCovered(double angle, double distance) {
        return Math.sin(Math.toRadians(angle)) * distance;
    }
}