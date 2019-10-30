package org.firstinspires.ftc.teamcode.autoRes.commands;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.subsystems.Chassis;

public class DriveCommand implements ICommand {
    Chassis chassis;
    double distance;
    double targetAngle;
    int turn;
    double power;
    int error;
    boolean hasSetDistance;

    public DriveCommand(Chassis chassis, double distance, double targetAngle, int turn, double power, int error) {
        this.chassis = chassis;
        this.distance = distance;
        this.targetAngle = targetAngle;
        this.turn = turn;
        this.power = power;
        this.error = error;
        hasSetDistance = false;
    }

    public boolean runCommand() {
        if (!hasSetDistance) {
            chassis.runDistance(distance, targetAngle, turn, power);
            hasSetDistance = true;
        }
        return chassis.runDistanceCheck(error);
    }
}
