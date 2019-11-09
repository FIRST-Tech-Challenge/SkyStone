package org.firstinspires.ftc.teamcode.Control;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

public class Constants {


    //--------------------------------ENCODERS-------------------------


    public static final double COUNTS_PER_MOTOR_REV = 1120;
    public static final double COUNTS_PER_MOTOR_TETRIX = 1440;
    public static final double DRIVE_GEAR_REDUCTION = 1.0;
    public static final double WHEEL_DIAMETER_INCHES = 4.0;
    public static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);                   // Conversion: Encoder Count to Inches
    public static final double COUNTS_PER_TETRIX_INCH = (COUNTS_PER_MOTOR_TETRIX * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);               // Conversion: Encoder Counts Motor Tetrix to Inches


    //--------------------------------TELE-OP VALUES--------------------
    public static final double DEAD_ZONE_SIZE = 0.1;

    //--------------------------------VUFORIA----------------------------

    public static final String VUFORIA_KEY = "ASYtBET/////AAABmSTQiLUzLEx3qLnHm6hu7Y1aNDWPDgMBKY8lFonYrzU8M5f9mAV5KiaJ9YZWCSgoUx6/AKuobb1cLgB8R+mDHgx6FoP3XS3K8bAwShz98sojuAKmTGzJMZVUjH8mjW+9ebYjtw3oZr/ZM2F2NZuCPN4Rx+K5koMfR2IE1OQKoZbkgLJSc36yUmis7MN91L0xIgntCKhqpZkRX45VjWsZi4BcKQnK5L2YfUqueZ7qvPzpF7sWDDcWYqkLZNbxfRk+gUVdabq/uOPYR8v0O0EFONv7h2kiU3E1s7Rm8WOukfwfqa5Nsw7FSNF2kjL0PhPbGPBQ6kVbLQMsvmxM7x/AA2owHe8l1yHgzyCgd7YTFOdi";

    public static final String TENSORFLOW_KEY = "Adb83BH/////AAABmTheak2ntU3VnH1pRcX2UDVJc60lqKXP9o54kAOKZoMvggLhrVVWOQ06E0yXEF3xRwJADjy5U2N519egNSjJ+Kj6jr05a6UmqLTEXS8elc2jYhx+T5P0pbc3ByKBdqw0lwBzL15jcqFrNDmbTH5hsuZjRP0RfvE1k/S2VW3wvD8U3GNtd2wb7xdQbmysXoDrNk0s+bgyn4mCX8jNL33RvYuIYfDKkC215c+jbYjn4rDAHNyM02Va777s5mcbYTb3LAX0iVYQApbtX4MjcPyU+D5p5dRQVYTE2hVtbMVvJg66m7ZcZ8aRV1GwTEYYVhq6z/iT3+cDH2pjNXtb0mGwHwyAnCwSMVqFtpbQ4DrC/3uj";

    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    public static final float mmPerInch = 25.4f;
    public static final float mmFTCFieldWidth = (12 * 6) * mmPerInch;       // the width of the FTC field (from the center point to the outer panels)
    public static final float mmTargetHeight = (6) * mmPerInch;          // the height of the center of the target image above the floor

    // Select which camera you want use.  The FRONT camera is the one on the same side as the screen.
    // Valid choices are:  BACK or FRONT
    public static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;


    //--------------------------------CONFIGURATION VALUES--------------------

    public static final String motorFRS = "motorFR";
    public static final String motorFLS = "motorFL";
    public static final String motorBRS = "motorBR";
    public static final String motorBLS = "motorBL";
    public static final String leftLinears = "leftlinear";
    public static final String rightLinears = "rightlinear";
    public static final String racks = "rack";
    public static final String servos = "servo";
    public static final String rotationservos = "rotservo";
    public static final String rightServos = "rightservo";
    public static final String leftServos = "leftservo";
    public static final String flimits = "flimit";
    public static final String blimits = "blimit";
    public static final String foundationServos = "foundationservo";




}
