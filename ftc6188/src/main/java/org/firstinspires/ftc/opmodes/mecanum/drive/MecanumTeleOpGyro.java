package org.firstinspires.ftc.opmodes.mecanum.drive;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotlib.robot.MecanumGyroRobot;

@TeleOp(name="Mecanum TeleOp V-Gyro", group="TeleOp")
public class MecanumTeleOpGyro extends OpMode
{
    private MecanumGyroRobot robot;

    @Override
    public void init()
    {
        robot = new MecanumGyroRobot(this.hardwareMap, this.telemetry, true);
    }

    @Override
    public void loop()
    {
        double course = Math.atan2(-gamepad1.right_stick_y, gamepad1.right_stick_x) - Math.PI/2;
        double velocity = Math.hypot(gamepad1.right_stick_x, -gamepad1.right_stick_y);

        robot.drivetrain.setCourse(course);
        robot.drivetrain.setVelocity(velocity);
        robot.drivetrain.setRotation(-gamepad1.left_stick_x);

        telemetry.update();
    }
}
