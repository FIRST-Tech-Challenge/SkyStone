package org.firstinspires.ftc.teamcode.Skystone.Test;

import org.firstinspires.ftc.teamcode.Skystone.Odometry.Odometry;

// Make sure to make a new application configuration in Android Studio in order to test this function locally
// Use TeamCode module
public class OdometryTest {

    public static void main(String[] args) {
        Odometry odometry = new Odometry();

        odometry.circularOdometry(5000,5000,70);
        System.out.println("X: " + odometry.getWorldX() + ", Y: " + odometry.getWorldY() + ", Angle: " + odometry.getWorldAngle());

        odometry.circularOdometry(100,100,0);
        System.out.println("X: " + odometry.getWorldX() + ", Y: " + odometry.getWorldY() + ", Angle: " + odometry.getWorldAngle());

    }
}
