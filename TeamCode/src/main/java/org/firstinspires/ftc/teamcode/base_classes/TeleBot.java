package org.firstinspires.ftc.teamcode.base_classes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.Range;

public class TeleBot extends Robot {


    //under this number, input won't be taken into account
    public double dead = .13;

    //stores the value of sin(45°), or sin(pi/4)
    public double sin45 = Math.sqrt(2) / 2;

    public TeleBot(OpMode opMode) {
        this.opMode = opMode;
    }

    public double[] getPowers() {
        return new double[]{frontLeft.getPower(), frontRight.getPower(), backRight.getPower(), backLeft.getPower()};
    }

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
        double strafeLeftPow = 0;

        //power for the right hand mecanum wheels (front right, back left)
        double strafeRightPow = 0;

        //power difference dedicated to rotation
        double rotPow = 0;

        //stores the power given to each motor clockwise from front left
        double[] powerValues;

        if ((Math.abs(leftX) > dead) || (Math.abs(leftY) > dead) || Math.abs(rightX) > dead) {
            //if the left stick is being used
            strafeLeftPow = sin45 * ((leftX - leftY));
            strafeRightPow = sin45 * ((leftX + leftY));

            //either fully rotating one way or another, positive (1) represents clockwise
            rotPow = (rightX > 0) ? (1) : (-1);

            //sets each motor's power to combination of strafe and rotate
            powerValues = new double[]{strafeLeftPow + rotPow, strafeRightPow - rotPow,
                    strafeLeftPow - rotPow, strafeRightPow + rotPow};

            //from Davis's code: rescale values out of the highest power in the array
            double maximumPower = 0;

            for (int i = 0; i < powerValues.length; i++) {
                if (Math.abs(powerValues[i]) > maximumPower) {
                    maximumPower = powerValues[i];
                }
            }

            if (maximumPower != 0) {
                for (int i = 0; i < powerValues.length; i++) {
                    powerValues[i] = powerValues[i] / Math.abs(maximumPower);
                }
            }

            frontLeft.setPower(Range.clip(powerValues[0], -1.0, 1.0));
            frontRight.setPower(Range.clip(powerValues[0], -1.0, 1.0));
            backLeft.setPower(Range.clip(powerValues[0], -1.0, 1.0));
            backRight.setPower(Range.clip(powerValues[0], -1.0, 1.0));

        } else {
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);
        }


    }

    public void setPower() {

    }
}