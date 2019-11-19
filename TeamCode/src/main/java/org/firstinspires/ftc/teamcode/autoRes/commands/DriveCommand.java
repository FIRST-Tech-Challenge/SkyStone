package org.firstinspires.ftc.teamcode.autoRes.commands;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.subsystems.Chassis;

public class DriveCommand implements ICommand {
    Chassis chassis;
    double targetAngle;
    double turn;
    int distance;
    double power;
    int error=0;
    boolean hasRun;

    public boolean hasRun() {
        return hasRun;
    }

    public DriveCommand(Chassis chassis, double targetAngle, double turn, int distance, double power) {
        this.chassis = chassis;
        this.targetAngle = targetAngle;
        this.turn = turn;
        this.distance = distance;
        this.targetAngle = targetAngle;
        this.turn = turn;
        this.power = power;
        this.error = error;
        hasRun = false;
    }

    public void run() {
        chassis.runDistance(distance, targetAngle, turn, power);
        hasRun= true;
    }

    public boolean isDone() {
        return chassis.motorErrorCheck(error);
    }

}
