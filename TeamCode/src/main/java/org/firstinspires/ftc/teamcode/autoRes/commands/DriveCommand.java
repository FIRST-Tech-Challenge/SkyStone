package org.firstinspires.ftc.teamcode.autoRes.commands;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.subsystems.Chassis;
import org.firstinspires.ftc.teamcode.subsystems.RobotMap.ChassisMotor;

import java.util.HashMap;
import java.util.Map;

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
        HashMap<ChassisMotor, DcMotor> motors = chassis.getMotors();
        HashMap<ChassisMotor, Integer> positions = new HashMap<>();
        for(Map.Entry<ChassisMotor, DcMotor> motor : motors.entrySet()){
            positions.put(motor.getKey(), distance);
        }
        chassis.setTargetPosition(positions);
        chassis.setMode(DcMotor.RunMode.RUN_TO_POSITION);
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
        HashMap<ChassisMotor, Double> chassisPowers = new HashMap<>();
        chassisPowers.put(ChassisMotor.FRONT_LEFT, frontLeftPower);
        chassisPowers.put(ChassisMotor.FRONT_RIGHT, frontRightPower);
        chassisPowers.put(ChassisMotor.BACK_LEFT, backLeftPower);
        chassisPowers.put(ChassisMotor.BACK_RIGHT, backRightPower);
        chassis.runChassis(angle, turn, power);
    }
}
