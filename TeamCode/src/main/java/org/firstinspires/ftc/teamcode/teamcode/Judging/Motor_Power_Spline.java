package org.firstinspires.ftc.teamcode.teamcode.Judging;

public class Motor_Power_Spline {
    double leftPower;
    double rightPower;
    public static double noLoadSpeed = 31.4 ; // Max Angular Velocity in radians/second for 20 : 1 motor
    public static double stallTorque = 2.1; // Max Torque in Newton Meters for 20 : 1 motor



    @Override
    public String toString() {
        return "Left Motor : " + leftPower + "    Right Power : " + rightPower;
    }

    public double getLeftPower() {
        return leftPower;
    }

    public double getRightPower() {
        return rightPower;
    }

    public static double setLeftPower(double secondDer, double der, double kD, double deltatime) {


        double leftVelocity = kD * (.5 * secondDer * deltatime + der);
        return (-1) * (stallTorque / noLoadSpeed) * leftVelocity
                + stallTorque * leftVelocity;
    }

    public static double setRightPower(double secondDer, double der, double kD, double deltatime) {
        double rightVelocity = kD * (der - .5 * secondDer * deltatime);
        return (-1) * (stallTorque / noLoadSpeed) * rightVelocity
                + stallTorque * rightVelocity;
    }

    public Motor_Power_Spline(double leftPower, double rightPower) {
        this.leftPower = leftPower;
        this.rightPower = rightPower;
    }
}

