package org.firstinspires.ftc.teamcode.competition;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import static java.lang.Math.abs;

/**
 * Class is for the mecanum drive code
 */
public class MecanumDrive {

    private Hardware robot;

    public double adjust = 0;

    /**
     * sets up the hardware refernce so you don't have to pass it as a parameter and sets the adjust
     *
     * @param r The hardware reference from the code
     */
    public MecanumDrive(Hardware r) {
        robot = r;
    }

    /**
     * this method is for driving the mecanum with the three inputs
     *
     * @param forward  The forward value input
     * @param sideways The sideways value input
     * @param rotation The rotation value input
     */
    public void drive(double forward, double sideways, double rotation) {

        //adds all the inputs together to get the number to scale it by
        double scale = abs(rotation) + abs(forward) + abs(sideways);

        //scales the inputs when needed
        if (scale > 1) {
            forward /= scale;
            rotation /= scale;
            sideways /= scale;
        }
        //setting the motor powers to move
        robot.leftFront.setPower(forward - rotation - sideways);
        robot.leftRear.setPower(forward - rotation + sideways);
        robot.rightFront.setPower(forward + rotation + sideways);
        robot.rightRear.setPower(forward + rotation - sideways);
        //Left Front = +Speed + Turn - Strafe      Right Front = +Speed - Turn + Strafe
        //Left Rear  = +Speed + Turn + Strafe      Right Rear  = +Speed - Turn - Strafe
    }

    /**
     * Field oriented drive for robot
     * Sets different sides to be the front of the robot
     *
     * @param forward  The forward value input
     * @param sideways The sideways value input
     * @param rotation The rotation value input
     * @param reset    Resets orientation to whichever direction the driver is facing
     */
    public void orientedDrive(double forward, double sideways, double rotation, boolean reset) {

        double P = Math.hypot(sideways, forward);
        Orientation angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);

        double robotAngle = Math.atan2(forward, -sideways);

        if (reset) adjust = angles.firstAngle;

        double v5 = P * Math.sin(robotAngle - angles.firstAngle + adjust) + P * Math.cos(robotAngle - angles.firstAngle + adjust) - rotation;
        double v6 = P * Math.sin(robotAngle - angles.firstAngle + adjust) - P * Math.cos(robotAngle - angles.firstAngle + adjust) + rotation;
        double v7 = P * Math.sin(robotAngle - angles.firstAngle + adjust) - P * Math.cos(robotAngle - angles.firstAngle + adjust) - rotation;
        double v8 = P * Math.sin(robotAngle - angles.firstAngle + adjust) + P * Math.cos(robotAngle - angles.firstAngle + adjust) + rotation;

        robot.leftFront.setPower(v5);
        robot.rightFront.setPower(v6);
        robot.leftRear.setPower(v7);
        robot.rightRear.setPower(v8);
    }

    /**
     * Sets power of motors
     *
     * @param power The power the robot is set to 0-1
     */
    public void powerSet(double power) {
        robot.leftFront.setPower(power);
        robot.rightFront.setPower(power);
        robot.leftRear.setPower(power);
        robot.rightRear.setPower(power);
    }

    /**
     * Moves robot forward fInches
     * 1 inch forward = 87 motor ticks
     *
     * @param fInches Inches forward
     */
    public void forwardInch(double fInches) {
        int fPos = (int) (fInches * 43);

        resetMotors();

        robot.leftFront.setTargetPosition(fPos);
        robot.leftRear.setTargetPosition(fPos);
        robot.rightFront.setTargetPosition(fPos);
        robot.rightRear.setTargetPosition(fPos);
    }

    /**
     * Moves the robot sideways sInches
     * 1 inch sideways left = 129 motor ticks
     *
     * @param sInches Inches sideways positive is to the left
     */
    public void sidewaysInch(int sInches) {
        int sPos = sInches * 64;

        resetMotors();

        robot.leftFront.setTargetPosition(-sPos);
        robot.leftRear.setTargetPosition(sPos);
        robot.rightFront.setTargetPosition(sPos);
        robot.rightRear.setTargetPosition(-sPos);
    }

    /**
     * Rotates robot a certain amount of degrees
     *
     * @param degrees Degrees to turn
     */
    public void rotate(int degrees) {
        int rDegrees = degrees * 9;

        resetMotors();

        robot.leftFront.setTargetPosition(rDegrees);
        robot.leftRear.setTargetPosition(rDegrees);
        robot.rightFront.setTargetPosition(-rDegrees);
        robot.rightRear.setTargetPosition(-rDegrees);
    }

    /**
     * Resets all motor positions back to 0
     */
    public void resetMotors() {
        robot.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    /**
     * Brakes the motors so robot can't move
     */
    public void brakeMotors() {
        //forwardInch(0);
        powerSet(0);
    }
}
