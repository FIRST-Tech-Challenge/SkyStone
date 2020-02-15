package org.firstinspires.ftc.teamcode.DutchFTCCore.SubSystems;

import org.firstinspires.ftc.teamcode.DutchFTCCore.Drivetraintypes;
import org.firstinspires.ftc.teamcode.DutchFTCCore.Robot;
import org.firstinspires.ftc.teamcode.DutchFTCCore.Robotconfig;

public class MovementSubSystem extends SubSystem {
    public static MovementSubSystem instance;
    Robot bot;
    Drivetraintypes traintypes;
    /**
     * Movement of the robot on the x axis in a value between -1 and 1;
     */
    public static double xMov;
    /**
     * Movement of the robot on the y axis in a value between -1 and 1;
     */
    public static double yMov;
    /**
     * Rotational input of the robot in a value between -1 and 1;
     */
    public static double rotation;

    @Override
    public void Start() {
        super.Start();
        bot = Robot.instance;
        instance = this;
        traintypes = bot.drivetrains;
    }

    @Override
    public void Update() {
        super.Update();
        traintypes.DriveChecks();
    }

    public void DriveChecksKiwiDrive(){
        //the variables for the motor speeds
        double right = -yMov + rotation;
        double left = yMov + rotation;
        double front = xMov + rotation;
        double back = -xMov + rotation;

        //setting the motor speeds
        bot.MotorBackLeft.setPower(left);
        bot.MotorFrontLeft.setPower(back);
        bot.MotorBackRight.setPower(front);
        bot.MotorFrontRight.setPower(right);
    }

    public void DriveChecks4WheelTankDrive(){
        //the variables for the motor speeds
        double right = -yMov + rotation;
        double left = yMov + rotation;

        //setting the motor speeds
        bot.MotorBackLeft.setPower(left);
        bot.MotorFrontLeft.setPower(left);
        bot.MotorBackRight.setPower(right);
        bot.MotorFrontRight.setPower(right);

    }

    public void DriveChecksTankDrive(){
        //the variables for the motor speeds
        double right = -yMov + rotation;
        double left = yMov + rotation;

        //setting the motor speeds
        bot.MotorBackLeft.setPower(left);
        bot.MotorBackRight.setPower(right);

    }

    public void DriveChecksHDrive5Motors(){

        //the variables for the motor speeds
        double right = -yMov + rotation;
        double left = yMov + rotation;
        double middle = xMov;

        //setting the motor speeds
        bot.MotorBackLeft.setPower(left);
        bot.MotorBackRight.setPower(right);
        bot.MotorFrontLeft.setPower(left);
        bot.MotorFrontRight.setPower(right);
        bot.MotorMiddle.setPower(middle);

    }

    public void DriveChecksHDrive3Motors(){

        //the variables for the motor speeds
        double right = -yMov + rotation;
        double left = yMov + rotation;
        double middle = xMov;

        //setting the motor speeds
        bot.MotorBackLeft.setPower(left);
        bot.MotorBackRight.setPower(right);
        bot.MotorMiddle.setPower(middle);

    }

    public void DriveChecksMechanum(){

        double stickangle;
        double speed;
        double frontLeftPower;
        double frontRightPower;
        double backLeftPower;
        double backRightPower;

        //radius of circle the stick is in
        speed = (Math.hypot(xMov, yMov));

        //inverse tangent calculate angle of stick
        stickangle = Math.atan2(xMov, yMov);

        //twisting the circle of units 45 degrees
        stickangle -= Math.PI / 4;

        frontLeftPower = (speed * Math.cos(stickangle) + rotation);
        frontRightPower = (speed * Math.sin(stickangle) - rotation);
        backLeftPower = (speed * Math.sin(stickangle) + rotation);
        backRightPower = (speed * Math.cos(stickangle) - rotation);

        bot.MotorFrontLeft.setPower(frontLeftPower);
        bot.MotorFrontRight.setPower(frontRightPower);
        bot.MotorBackLeft.setPower(backLeftPower);
        bot.MotorBackRight.setPower(backRightPower);

    }
}
