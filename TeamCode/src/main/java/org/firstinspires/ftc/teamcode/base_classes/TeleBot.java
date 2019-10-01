package org.firstinspires.ftc.teamcode.base_classes;

public class TeleBot extends Robot {

    //under this number, input won't be taken into account
    public double dead = .13;

    //stores the value of sin(45°), or sin(pi/4)
    public double sin45 = Math.sqrt(2)/2;

    /**
     * Takes controller input in order to move the robot
     */
    public void controlledDrive() {
        //stores values once so you don't have to retrieve them again within the method
        double leftX = opMode.gamepad1.left_stick_x;
        double leftY = opMode.gamepad1.left_stick_y;
        double rightX = opMode.gamepad1.right_stick_x;

        //the 'radius' of the circle this represents
        double rad = 1;

        //power for the left hand mecanum wheels (front left, back right)
        double leftPow;

        //power for the right hand mecanum wheels (front right, back left)
        double rightPow;

        //stores the power given to each motor clockwise from front left
        double[] powerValues;

        if ((Math.abs(leftX)> dead) || (Math.abs(leftY) > dead) || Math.abs(rightX) > dead) {
            //if the left stick is being used
            leftPow = rad * sin45 * ((leftX - leftY))/rad;
            rightPow = rad * sin45 * ((leftX + leftY)/rad);
        }

        if (Math.abs(rightX) > dead) {
            //if the right stick is being used
        }

    }

    public void setPower() {

    }
}