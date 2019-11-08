package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.RobotNavigator;

public class MecanumDrive {

    RobotHardware robotHardware = new RobotHardware();
    HardwareMap hardwareMap;

    double forward;
    double strafe;
    double rotation;

    public void init(){
        robotHardware.init(hardwareMap, null);
    }

    public void setHardwareMap(HardwareMap harwareMap){
        this.hardwareMap = harwareMap;
    }

    public void reset(){
        forward = 0;
        rotation = 0;
        strafe = 0;
    }

    public void drive(double forward, double strafe, double rotation){
        //claire - press the left trigger to have field orientation, otherwise it is robot orientation


        //10/28/2019, Will, Ian Athena implemented and tested the drive method
        double front_left = forward - rotation - strafe;
        double front_right = forward + rotation + strafe;
        double rear_left = forward - rotation + strafe;
        double rear_right = forward + rotation - strafe;

//        Logger.logFile("in drive, forward = " + forward);
//        Logger.logFile("in drive, strafe = " + strafe);
//        double r = Math.hypot(strafe, forward);
//        double robotAngle = Math.atan2(forward, strafe) - Math.PI / 4;
//        Logger.logFile("robotAngle = " + Math.toDegrees(robotAngle));

//        double front_left = r * Math.cos(robotAngle) + rotation;
//        double front_right = r * Math.sin(robotAngle) - rotation;
//        double rear_left = r * Math.sin(robotAngle) + rotation;
//        double rear_right = r * Math.cos(robotAngle) - rotation;
//
        Logger.logFile("FL = " + front_left);
        Logger.logFile("FR = " + front_right);
        Logger.logFile("RL = " + rear_left);
        Logger.logFile("RR = " + rear_right);


        double biggest = 1;
        if (Math.abs(front_right) > biggest){
            biggest = Math.abs(front_right);
        } else if (Math.abs(rear_left) > biggest){
            biggest = Math.abs(rear_left);
        } else if (Math.abs(rear_right) > biggest){
            biggest = Math.abs(rear_right);
        } else if (Math.abs(front_left) > biggest){
            biggest = Math.abs(front_left);
        }

        front_left /= biggest;
        front_right /= biggest;
        rear_left /= biggest;
        rear_right /= biggest;

        robotHardware.setMotorPower(front_left, front_right, rear_left, rear_right);
    }

    //10/14/2019 Reily refactor field oriented to new method
    public void switchToFieldOrientated(double heading) {
        Logger.logFile("heading in switch method = " + heading);
        Logger.logFile("before translation, forward = " + forward);
        Logger.logFile("before translation, strafe = " + strafe);
        double temp = forward * Math.cos(heading) + strafe * Math.sin(heading);
        strafe = -forward * Math.sin(heading) + strafe * Math.cos(heading);
        forward = temp;
        Logger.logFile("after translation, forward = " + forward);
        Logger.logFile("after translation, strafe = " + strafe);
    }

}
