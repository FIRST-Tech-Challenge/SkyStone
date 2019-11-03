package org.firstinspires.ftc.teamcode.headingable;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotlib.controller.PIDController;
import org.firstinspires.ftc.robotlib.robot.HeadingableMecanumRobot;
import org.firstinspires.ftc.robotlib.util.PIDTuner;

@Disabled
@TeleOp (name="Headingable Mecanum PIDTuner", group="TeleHead")
public class HeadingableMecanumPIDTuner extends OpMode
{
    private PIDTuner tuner;

    @Override
    public void init()
    {
        HeadingableMecanumRobot robot = new HeadingableMecanumRobot(this.hardwareMap, this.telemetry, true);
        tuner = new PIDTuner(robot.drivetrain, (PIDController) robot.controller.algorithm, this.gamepad1, this.telemetry);
    }

    @Override
    public void loop()
    {
        tuner.update();
    }
}
