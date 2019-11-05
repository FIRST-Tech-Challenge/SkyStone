package org.firstinspires.ftc.teamcode.teamcode.Judging;


public class Point {
    double x = 0;
    double y = 0;
    double t = 0;
    double derivative = 0;
    double secondDerivative = 0;

    public Point(double t, double x, double y) {
        this.x = x;
        this.y = y;
        this.t = t;
    }
    public Point(double t, double x, double y, double derivative, double secondDerivative) {
        this.x = x;
        this.y = y;
        this.t = t;
        this.derivative = derivative;
        this.secondDerivative = secondDerivative;
    }

    public double getT() {
        return t;
    }

    public void setT(double t) {
        this.t = t;
    }

    public double getX() {
        return x;
    }

    public void setX(double x) {
        this.x = x;
    }

    public double getY() {
        return y;
    }

    public void setY(double y) {
        this.y = y;
    }

    @Override
    public String toString() {
        return "(" + x + ", "  + y +")";
    }
}
