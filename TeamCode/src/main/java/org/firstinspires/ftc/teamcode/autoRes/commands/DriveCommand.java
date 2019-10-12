package org.firstinspires.ftc.teamcode.autoRes.commands;

import org.firstinspires.ftc.teamcode.subsystems.Chassis;
import org.firstinspires.ftc.teamcode.subsystems.RobotMap;

import java.util.HashMap;

public class DriveCommand implements ICommand {
    Chassis chassis;
    double angle;
    double turn;
    double distance;
    double power;
    public DriveCommand(Chassis chassis, double angle, double turn, double distance, double power){
        this.chassis = chassis;
        this.angle = angle;
        this.turn = turn;
        this.distance = distance;
        this.power = power;
    }
    public boolean runCommand(){
        final double stickRadius = power;//Flip Y stick
        final double targetAngle = angle;
        final double turnPower = turn;
        final double frontLeftPower = stickRadius * Math.cos(targetAngle) + turnPower;
        final double frontRightPower = stickRadius * Math.sin(targetAngle) - turnPower;
        final double backLeftPower = stickRadius * Math.sin(targetAngle) + turnPower;
        final double backRightPower = stickRadius * Math.cos(targetAngle) - turnPower;
        HashMap<RobotMap.ChassisMotor, Double> chassisPowers = new HashMap<>();
        chassisPowers.put(RobotMap.ChassisMotor.FRONT_LEFT, frontLeftPower);
        chassisPowers.put(RobotMap.ChassisMotor.FRONT_RIGHT, frontRightPower);
        chassisPowers.put(RobotMap.ChassisMotor.BACK_LEFT, backLeftPower);
        chassisPowers.put(RobotMap.ChassisMotor.BACK_RIGHT, backRightPower);
        chassis.setMotors(chassisPowers);
        return true;
    }
}
