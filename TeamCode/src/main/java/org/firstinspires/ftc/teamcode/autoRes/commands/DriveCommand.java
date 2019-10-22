package org.firstinspires.ftc.teamcode.autoRes.commands;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.subsystems.Chassis;

public class DriveCommand implements ICommand {
    Chassis chassis;
    double angle;
    double turn;
    int distance;
    double power;

    public DriveCommand(Chassis chassis, double angle, double turn, int distance, double power) {
        this.chassis = chassis;
        this.angle = angle;
        this.turn = turn;
        this.distance = distance;
        this.power = power;
    }

    public boolean runCommand() {
        chassis.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        setPowers();
        return true;
    }

    public void setMode() {
    }

    public void setPowers() {
        final double stickRadius = power;//Flip Y stick
        final double targetAngle = angle;
        final double turnPower = turn;
        final double frontLeftPower = stickRadius * Math.cos(targetAngle) + turnPower;
        final double frontRightPower = stickRadius * Math.sin(targetAngle) - turnPower;
        final double backLeftPower = stickRadius * Math.sin(targetAngle) + turnPower;
        final double backRightPower = stickRadius * Math.cos(targetAngle) - turnPower;
        chassis.runChassis(angle, turn, power);
    }
}
