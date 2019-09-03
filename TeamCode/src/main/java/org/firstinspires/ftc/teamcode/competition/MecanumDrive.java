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

    private final double PID_POWER_ADJUST_THETA = .1;
    private final double PID_POWER_ADJUST_CM = .2;
    private final double PID_CM_DISTANCE = 5;
    private final double PID_THETA_DISTANCE = Math.toRadians(20);

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
     * @param reset Resets orientation to whichever direction the driver is facing
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
     * Moves robot forward fCm on Y-plane
     * 1 centimeter forward = X motor ticks
     * @param fCm Centimeters forward
     */
    public void forwardCm(double fCm) {
        fCm += robot.y;

        if(fCm > robot.x){
            while(robot.y < fCm)
                drive(1, 0, 0);
            return;
        }
        while(robot.y > fCm)
            drive(-1, 0, 0);
    }

    /**
     * Moves robot forward fCm on Y-plane
     * 1 centimeter forward = X motor ticks
     * @param fCm Centimeters forward
     * @param power Power the motors will run at
     */
    public void forwardCmAtSpeed(double fCm, double power) {
        fCm += robot.y;

        if(fCm > robot.x){
            while(robot.y < fCm)
                drive(power, 0, 0);
            powerSet(0);
            return;
        }
        while(robot.y > fCm)
            drive(-power, 0, 0);
    }

    /**
     * Moves robot forward fCm on Y-plane
     * 1 centimeter forward = X motor ticks
     * Utilizes PID to ensure the robot will come to a stop without jolting forward
     * @param fCm Centimeters forward
     */
    public void forwardCmPID(double fCm) {
        fCm += robot.y;
        double power = 1;

        if(fCm > robot.x){
            while(robot.y < fCm && power != 0) {
                // For each centimeter the robot is within 5cm of the target, subtract .2 from the power
                if(robot.y - fCm <= PID_CM_DISTANCE)
                    power = (robot.y - fCm) * PID_POWER_ADJUST_CM;

                drive(power, 0, 0);
            }
            powerSet(0);
            return;
        }
        while(robot.y > fCm && power != 0) {
            //for each cm the robot is within 5 cm of the target, subtract .2 from the power
            if(robot.y - fCm <= PID_CM_DISTANCE)
                power = (robot.y - fCm) * PID_POWER_ADJUST_CM;

            drive(-power, 0, 0);
        }
        powerSet(0);
    }

    /**
     * Moves the robot sideways sCm on the X-plane
     * 1 centimeter sideways left = X motor ticks
     * @param sCm Inches sideways positive is to the left
     */
    public void sidewaysCm(double sCm) {
        sCm += robot.x;

        if(sCm > robot.x){
            while(robot.x < sCm)
                drive(0, 1, 0);
            powerSet(0);
            return;
        }
        while(robot.x > sCm)
            drive(0, -1, 0);
        powerSet(0);
    }


    /**
     * Moves the robot sideways sCm on the X-plane
     * 1 centimeter sideways left = X motor ticks
     * @param sCm Inches sideways positive is to the left
     * @param power Power the motors will run at
     */
    public void sidewaysCmAtSpeed(double sCm, double power) {
        sCm += robot.x;

        if(sCm > 0){
            while(robot.x < sCm)
                drive(0, power, 0);
            powerSet(0);
            return;
        }
        while(robot.x > sCm)
            drive(0, -power, 0);
        powerSet(0);
    }

    /**
     * Moves the robot sideways sCm on the X-plane
     * 1 centimeter sideways left = X motor ticks
     * @param sCm Inches sideways positive is to the left
     */
    public void sidewaysCmPID(double sCm) {
        sCm += robot.x;
        double power = 1;

        if(sCm > 0){
            while(robot.x < sCm) {
                // For each centimeter the robot is within 5cm of the target, subtract .2 from the power
                if (robot.x - sCm <= PID_CM_DISTANCE)
                    power = (robot.x - sCm) * PID_POWER_ADJUST_CM;

                drive(0, power, 0);
            }
            powerSet(0);
            return;
        }
        while(robot.x > sCm) {
            // For each centimeter the robot is within 5cm of the target, subtract .2 from the power
            if (robot.x - sCm <= PID_CM_DISTANCE)
                power = (robot.x - sCm) * PID_POWER_ADJUST_CM;

            drive(0, -power, 0);
        }
        powerSet(0);
    }

    /**
     * Rotates robot a certain amount of degrees at max speed
     * @param degrees How much to turn in radians
     */
    public void rotate(double degrees) {
        double newTheta = robot.theta + Math.toRadians(degrees);

        if(newTheta > robot.theta){
            while(newTheta > robot.theta)
                drive(0, 0, 1);
            powerSet(0);
            return;
        }
        while(newTheta > robot.theta)
            drive(0, 0, -1);
        powerSet(0);
    }

    /**
     * Rotates robot a certain amount of degrees at "power" speed
     * @param degrees How much to turn in radians
     * @param power Power the motors will run at
     */
    public void rotateMaxSpeed(double degrees, double power) {
        double newTheta = robot.theta + Math.toRadians(degrees);

        if(newTheta > robot.theta){
            while(newTheta > robot.theta)
                drive(0, 0, power);
            powerSet(0);
            return;
        }
        while(newTheta > robot.theta)
            drive(0, 0, -power);
        powerSet(0);
    }

    /**
     * Rotates robot a certain amount of degrees
     * @param degrees How much to turn in radians
     */
    public void rotatePID(double degrees) {
        double newTheta = robot.theta + Math.toRadians(degrees);
        double power = 1;

        if(newTheta > robot.theta){
            while(newTheta > robot.theta) {
                // For each degree the robot is in within degrees of the target degree subtract from the power
                if(robot.theta - newTheta <= PID_THETA_DISTANCE)
                    power = (robot.y - newTheta) * PID_POWER_ADJUST_THETA;

                drive(0, 0, power);
            }
            powerSet(0);
            return;
        }
        while(newTheta > robot.theta) {
            // For each degree the robot is in within degrees of the target degree subtract from the power
            if(robot.theta - newTheta <= PID_THETA_DISTANCE)
                power = (robot.y - newTheta) * PID_POWER_ADJUST_THETA;

            drive(0, 0, -power);
        }
        powerSet(0);
    }

    public void moveToPosition(double newX, double newY){

    }

    public void moveToPosition(double newX, double newY, double newTheta){

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
        forwardCm(0);
        powerSet(0);
    }
}
