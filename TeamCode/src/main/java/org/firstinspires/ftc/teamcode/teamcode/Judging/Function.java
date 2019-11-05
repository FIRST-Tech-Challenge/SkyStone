package org.firstinspires.ftc.teamcode.teamcode.Judging;


public class Function {

    double arclength = 0;

    double aX = 0;
    double bX = 0;
    double cX = 0;
    double dX = 0;

    double aY = 0;
    double bY = 0;
    double cY = 0;
    double dY = 0;

    double startT;
    double endT;

    public Function(double aX, double bX, double cX, double dX, double aY, double bY, double cY, double dY, double startT, double endT) {


        this.startT = startT;
        this.endT = endT;

        this.aX = aX;
        this.bX = bX;
        this.cX = cX;
        this.dX = dX;

        this.aY = aY;
        this.bY = bY;
        this.cY = cY;
        this.dY = dY;
    }

    public double getFuncX(double t)
    {
        t = t - startT;
        return aX + bX * t + cX * Math.pow(t, 2) + dX * Math.pow(t, 3);
    }

    public double getDerX(double t)
    {
        t = t - startT;
        return bX + 2 * cX * t + 3 * dX * Math.pow(t, 2);
    }

    public double getSecondDerX(double t)
    {
        t = t - startT;
        return 2 * cX + 6 * dX * t;
    }

    public double getSecondDerY(double t)
    {
        t = t - startT;
        return 2 * cY + 6 * dY * t;
    }
    public double getFuncY(double t)
    {
        t = t - startT;
        return aY + bY*t + cY*Math.pow(t, 2) + dY*Math.pow(t, 3);
    }

    public double getDerY(double t)
    {
        t = t - startT;
        return bY + 2 * cY * t + 3 * dY * Math.pow(t, 2);
    }

    @Override
    public String toString() {



        return "( " + aX + " + " + bX + "(t -" + startT + ") + " + cX + "(t -" + startT + ")^2 + " +
                dX + "(t -" + startT + ")^3 , " + aY + " + " + bY + "(t -" + startT + ") + " + cY + "(t -" + startT + ")^2 + " +
                dY + "(t -" + startT + ")^3 ) ";
    }
}
