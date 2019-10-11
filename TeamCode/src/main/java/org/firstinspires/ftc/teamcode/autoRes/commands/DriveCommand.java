package org.firstinspires.ftc.teamcode.autoRes.commands;

import org.firstinspires.ftc.teamcode.subsystems.Chassis;

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
        final double frontLeftPower = power * Math.cos(angle) + turn;
        final double frontRightPower = power * Math.sin(angle) - turn;
        final double backLeftPower = power * Math.sin(angle) + turn;
        final double backRightPower = power * Math.cos(angle) - turn;
        return true;
    }
}
